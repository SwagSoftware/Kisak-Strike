// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef WIN32
#	pragma implementation "ivp_object_attach.hxx"
#endif

#include <ivp_physics.hxx>
#include <string.h>

#include <ivp_templates.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_controller.hxx>
#include <ivp_calc_next_psi_solver.hxx>
#include <ivp_object_attach.hxx>
#include <ivp_hull_manager.hxx>
#include <ivp_hull_manager_macros.hxx>
#include <ivp_core_macros.hxx>
#include <ivp_mindist_intern.hxx>
#include <ivp_friction.hxx>

void IVP_Object_Attach::attach_object( IVP_Real_Object *parent, IVP_Real_Object *attached_object, 
				      IVP_DOUBLE max_distance_attached_object_to_parent ){

    if ( max_distance_attached_object_to_parent < 0.0f){
	// take current distance
	IVP_Time time = parent->get_environment()->get_current_time();
	IVP_U_Point parents_mass_center;
	parent->get_core()->inline_calc_at_position( time, &parents_mass_center);

	IVP_U_Point attached_mass_center;
	attached_object->get_core()->inline_calc_at_position( time, &attached_mass_center);

	max_distance_attached_object_to_parent = IVP_Inline_Math::sqrtd(parents_mass_center.quad_distance_to(&attached_mass_center));

    }


    // make sure both objects share the same state
    if ( IVP_MTIS_SIMULATED(parent->get_movement_state()) ){
	if ( !IVP_MTIS_SIMULATED(attached_object->get_movement_state()) ){
    	    attached_object->revive_object_for_simulation();
	}
    }else{
	if ( IVP_MTIS_SIMULATED(attached_object->get_movement_state()) ){
    	    parent->revive_object_for_simulation();
	}
    }

    IVP_Core *old_core = attached_object->get_core();

    IVP_Cache_Object *co = attached_object->get_cache_object_no_lock();
    IVP_U_Quat q_world_f_object = co->q_world_f_object;
    IVP_U_Point shift_world_f_object( co->m_world_f_object.get_position());

    IVP_Core *ncore = parent->get_core();

    // update radius
    IVP_DOUBLE radius = max_distance_attached_object_to_parent + old_core->upper_limit_radius;
    if (radius > ncore->upper_limit_radius){
	ncore->upper_limit_radius = radius;
    }

    IVP_DOUBLE max_surface_deviation = max_distance_attached_object_to_parent + old_core->max_surface_deviation;
    if (max_surface_deviation > ncore->max_surface_deviation){
	ncore->max_surface_deviation = max_surface_deviation;
    }	

    // exchange cores
    {
	old_core->unlink_obj_from_core_and_maybe_destroy( attached_object );
	attached_object->physical_core = ncore;
	attached_object->friction_core = ncore;
	attached_object->original_core = ncore;
	ncore->core_add_link_to_obj(attached_object);
    }
    {
	attached_object->flags.object_movement_state = parent->get_movement_state();
    }

    // position object
    reposition_object_Ros( NULL, attached_object, &q_world_f_object,  &shift_world_f_object, IVP_FALSE);

    // now update radius

    ncore->values_changed_recalc_redundants();
}

void IVP_Object_Attach::detach_object( IVP_Real_Object *attached_object, IVP_Template_Real_Object *t){

    // get rid of all contact points as they reference the core
    attached_object->unlink_contact_points(IVP_TRUE);


    IVP_Environment *env = attached_object->get_environment();
    IVP_Core *old_core = attached_object->get_core();

    // orientation
    IVP_U_Quat q_world_f_object;
    IVP_U_Point position_ws;

    {
        IVP_Cache_Object *co = attached_object->get_cache_object_no_lock();
	q_world_f_object = co->q_world_f_object;
	position_ws.set(co->m_world_f_object.get_position());
    }

    // velocity 
    IVP_U_Float_Point speed_ws;
    old_core->get_surface_speed_ws( &position_ws, &speed_ws);

    // angular velocity
    IVP_U_Float_Point rot_speed_cs;
    rot_speed_cs.set ( &old_core->rot_speed );

    // get the old center offset
    IVP_DOUBLE moved_distance_val = attached_object->get_shift_core_f_object()->real_length();


    // reset object values
    if(t->get_nocoll_group_ident()[0]){
	strncpy(attached_object->nocoll_group_ident, t->get_nocoll_group_ident(),IVP_NO_COLL_GROUP_STRING_LEN);
    }
    {  // get rid of old core
		int n_objs = old_core->objects.len();
			old_core->unlink_obj_from_core_and_maybe_destroy(attached_object);
		if (n_objs <= 1) old_core = 0; // mark core as deleted
		if (old_core) {
			old_core->values_changed_recalc_redundants();
		}
    }
    { // build new core
	attached_object->flags.object_movement_state = IVP_MT_NOT_SIM;
	attached_object->physical_core = new IVP_Core(attached_object, &q_world_f_object, &position_ws, IVP_FALSE /*physical_unmoveable*/,t->enable_piling_optimization);

	attached_object->friction_core = attached_object->physical_core;
	attached_object->original_core = attached_object->physical_core;

	if (t->material){
	    attached_object->l_default_material = t->material;
	}

	if (t->client_data){
	    attached_object->client_data = t->client_data;
	}
    
	attached_object->extra_radius = t->extra_radius;
	attached_object->init_object_core( env, t);
    }

    // set the new speed for next psi and wakeup object
    if ( IVP_MTIS_SIMULATED(old_core->movement_state)) {
	attached_object->get_core()->fire_event_object_frozen();		 // tell them that we went to sleep
	attached_object->ensure_in_simulation_now(); // wakeup and tell everybody
						  
	attached_object->get_core()->speed.set( &speed_ws );
	attached_object->get_core()->rot_speed.set( &rot_speed_cs );

	IVP_DOUBLE delta_time_till_next_PSI = env->get_next_PSI_time() - env->get_current_time();
	IVP_Vector_of_Hulls_128 active_hull_managers;
	IVP_Core *my_core= attached_object->get_core();
	if(!my_core->physical_unmoveable) {

   	    IVP_Hull_Manager *h_manager = attached_object->get_hull_manager();
	    h_manager->jump_add_hull(moved_distance_val,0.0f);


	    IVP_Event_Sim es(env, delta_time_till_next_PSI);
	    IVP_Calc_Next_PSI_Solver nps(my_core);
	    nps.calc_next_PSI_matrix(&es, &active_hull_managers);
	    my_core->tmp_null.old_sync_info=NULL;
   	    IVP_Calc_Next_PSI_Solver::commit_all_hull_managers( env,  &active_hull_managers);
	    attached_object->recalc_exact_mindists_of_object();
	    attached_object->update_exact_mindist_events_of_object();
	}
    }
}
    
    /********************************************************************************
    *	Name:	       	reposition_object_Ros.hxx
    *	Parameter:	q_Ros_f_Aos is a quat that transforms from Attached object space
    *			into Reference object space == matrix defining the attached object in Ros space
    *	Parameter:      check_before_moving if set movement will be limited if there
    *			might be penetration problems
    ********************************************************************************/
IVP_RETURN_TYPE IVP_Object_Attach::reposition_object_Ros( IVP_Real_Object *parent, IVP_Real_Object *attached_object,
						const IVP_U_Quat *q_Ros_f_Aos,
						const IVP_U_Point *shift_Ros_f_Aos,
						IVP_BOOL check_before_moving){

    IVP_U_Matrix m_world_f_Ros;
    if (parent){
	IVP_Cache_Object *co = parent->get_cache_object_no_lock();
	m_world_f_Ros = co->m_world_f_object;
    }else{
	m_world_f_Ros.init();
    }

    IVP_U_Matrix m_Ros_F_Aos;
    q_Ros_f_Aos->set_matrix( & m_Ros_F_Aos );
    m_Ros_F_Aos.get_position()->set(shift_Ros_f_Aos);

    IVP_U_Matrix m_ws_f_Aos;
    m_world_f_Ros.mmult4(&m_Ros_F_Aos, &m_ws_f_Aos);


    IVP_U_Matrix m_world_f_core;
    { // get current m_world_f_core
	IVP_Time current_time = attached_object->get_environment()->get_current_time();
	IVP_U_Quat q_world_f_core;
	attached_object->get_core()->inline_calc_at_quaternion(current_time, &q_world_f_core);
	q_world_f_core.set_matrix( & m_world_f_core );
	attached_object->get_core()->inline_calc_at_position(current_time, &m_world_f_core.vv);
    }

    IVP_U_Matrix m_Aos_f_Rcs;   m_ws_f_Aos.mimult4( &m_world_f_core, &m_Aos_f_Rcs);

    attached_object->set_new_m_object_f_core(&m_Aos_f_Rcs);

    IVP_DOUBLE diff = 0.01f;		    // hopefully we wont get an endless loop
    IVP_DOUBLE moved_distance_val = 0.0f; // center movement 

	IVP_Hull_Manager *h_manager = attached_object->get_hull_manager();
	h_manager->jump_add_hull(diff,moved_distance_val);

	IVP_Cache_Object_Manager::invalid_cache_object(attached_object);
	attached_object->recalc_exact_mindists_of_object();
	attached_object->recalc_invalid_mindists_of_object();

	h_manager->check_hull_synapses();
	h_manager->check_for_reset();

    return IVP_OK;
}

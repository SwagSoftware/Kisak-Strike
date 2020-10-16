// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#ifndef WIN32
#	pragma implementation "ivp_environment.hxx"
#endif

#include <ivp_physics.hxx>

#if defined(WIN32) || defined(PSXII)
//#	include <sys\types.h>
//#	include <sys\stat.h>
//#	include <windows.h>
#	include <time.h>
#endif

#if defined(PSXII)
#include <libcdvd.h>
#define NULL 0
#endif

#include <ivp_debug.hxx>
#include <ivp_debug_manager.hxx>
#include <ivp_physic_private.hxx>
#include <ivp_sim_unit.hxx>
#include <ivp_time.hxx>
#include <ivu_memory.hxx>
#include <ivu_active_value.hxx>
#include <ivp_actuator.hxx>
#include <ivp_templates.hxx>
#include <ivp_controller_motion.hxx>

#include <ivp_material.hxx>
#include <ivp_collision_filter.hxx>
#include <ivp_compact_ledge.hxx>
#include <ivp_clustering_longrange.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_cache_ledge_point.hxx>
#include <ivp_range_manager.hxx>
#include <ivp_anomaly_manager.hxx>

#include <ivp_great_matrix.hxx>
#include <ivp_constraint_local.hxx>

#include <ivp_mindist_intern.hxx>
#include <ivp_mindist_minimize.hxx>
#include <ivp_mindist_event.hxx>
#include <ivp_friction.hxx>

#include <ivp_listener_collision.hxx>
#include <ivp_listener_object.hxx>
#include <ivp_listener_psi.hxx>
#include <ivp_calc_next_psi_solver.hxx>

#include <ivp_merge_core.hxx>
#include <ivu_set.hxx>
#include <ivp_controller_buoyancy.hxx>
#include <ivp_performancecounter.hxx>

#include <ivp_betterstatisticsmanager.hxx>

#include <ivp_authenticity.hxx>

#ifndef _IVP_TIME_INCLUDED
    #include <ivp_time.hxx>
#endif
#if !defined(IVP_VERSION_SDK) && !defined(IVP_VERSION_EVAL)
#   pragma error("You have to define IVP_VERSION_SDK or IVP_VERSION_EVAL")
#endif

IVP_Freeze_Manager::IVP_Freeze_Manager(){
    init_freeze_manager();
}

void IVP_Freeze_Manager::init_freeze_manager(){
    freeze_check_dtime = 0.3f;
}

IVP_Environment::IVP_Environment(IVP_Environment_Manager *manager,IVP_Application_Environment *appl_env,
				 const char *company_name,unsigned int auth_code)
{
  IVP_ASSERT( sizeof(IVP_Compact_Edge) == 4);
  IVP_ASSERT( sizeof(IVP_Compact_Triangle) == 16);
  
    P_MEM_CLEAR(this);
    get_freeze_manager()->init_freeze_manager();

    state=IVP_ES_AT;
    statistic_manager.l_environment=this;
    
    {
	l_material_manager = appl_env->material_manager;
	if (!l_material_manager){
	    l_material_manager = new IVP_Material_Manager(IVP_TRUE);
	}

	l_active_value_manager = appl_env->env_active_float_manager;
	if (!l_active_value_manager){
	    l_active_value_manager = new IVP_U_Active_Value_Manager(IVP_TRUE);
	}
	
	collision_filter = appl_env->collision_filter;
	if (!collision_filter){
	    collision_filter = new IVP_Collision_Filter_Coll_Group_Ident(IVP_TRUE);
	}

	anomaly_manager = appl_env->anomaly_manager;
	if (!anomaly_manager){
	    anomaly_manager = new IVP_Anomaly_Manager(IVP_TRUE);
	}

	anomaly_limits = appl_env->anomaly_limits;
	if (!anomaly_limits){
	    anomaly_limits = new IVP_Anomaly_Limits(IVP_TRUE);
	}
	
	if (!appl_env->default_collision_delegator_root){
	  collision_delegator_roots.add ( new IVP_Collision_Delegator_Root_Mindist() );
	}else{
	  collision_delegator_roots.add(appl_env->default_collision_delegator_root);
	}
	
	universe_manager = appl_env->universe_manager;
    }

    {
	standard_gravity_controller = new IVP_Standard_Gravity_Controller();
	IVP_U_Point temp_grav(0.0f,9.83f,0.0f);
	set_gravity(&temp_grav);
	standard_gravity_controller->grav_vec.set(&temp_grav);
	manager->environments.add(this);
    }
    
    controller_manager = new IVP_Controller_Manager(this);
    
    set_delta_PSI_time( 1.0f/66.0f );
    cluster_manager  = new IVP_Cluster_Manager(this);
    sim_units_manager= new IVP_Sim_Units_Manager(this);
    time_manager     = new IVP_Time_Manager();
    mindist_manager  = new IVP_Mindist_Manager(this);
    ov_tree_manager  = new IVP_OV_Tree_Manager();
    this->better_statisticsmanager = new IVP_BetterStatisticsmanager();

    this->range_manager = appl_env->range_manager;
    if(!appl_env->range_manager){
	this->range_manager = new IVP_Range_Manager(this, IVP_TRUE);
    }

    if (appl_env->performancecounter){
	this->performancecounter = appl_env->performancecounter;
    }else{
	this->performancecounter = new IVP_PerformanceCounter_Simple();
    }
    cache_object_manager = new IVP_Cache_Object_Manager(appl_env->n_cache_object );
    
    environment_manager = manager;
    current_time_code = 1;
    current_time = 0.0f;
    time_of_next_psi = get_delta_PSI_time();

    {
	short_term_mem = new IVP_U_Memory();
	sim_unit_mem = new IVP_U_Memory();
	
	int ssizeh = appl_env->scratchpad_size>>1;
	short_term_mem->init_mem_transaction_usage(appl_env->scratchpad_addr, ssizeh);
	sim_unit_mem->init_mem_transaction_usage  (ssizeh + appl_env->scratchpad_addr, ssizeh);
    }

    next_movement_check=IVP_MOVEMENT_CHECK_COUNT;
    
    IVP_IF( 1 ) {
	this->debug_information=new IVP_Debug_Manager();
	this->delete_debug_information = IVP_TRUE;
    }

    integrated_energy_damp = IVP_Inline_Math::ivp_expf ( IVP_FLOAT(log(0.9f)) * get_delta_PSI_time() );

    IVP_Mindist_Minimize_Solver::init_mms_function_table();
    IVP_Mindist_Event_Solver::init_mim_function_table();

    this->auth_costumer_name = p_strdup(company_name);
    this->auth_costumer_code = auth_code;
    this->pw_count = 10;
   
    { // create a static ball
	IVP_Template_Ball b;
	b.radius = 1.0f;
	IVP_Template_Real_Object t;
	t.set_name( "static_object");
	t.physical_unmoveable = IVP_TRUE;
	IVP_U_Matrix mco;
	mco.init();
	mco.vv.set_to_zero();
	t.mass_center_override = &mco;

	IVP_U_Point position; position.set_to_zero();
	this->static_object = this->create_ball( &b, &t, NULL, &position );
    }

	environment_magic_number=IVP_Environment_Magic_Number;
}

void IVP_Environment::set_delta_PSI_time(IVP_DOUBLE psi_time){
    IVP_ASSERT(psi_time >= IVP_MIN_DELTA_PSI_TIME);
    IVP_ASSERT(psi_time <= IVP_MAX_DELTA_PSI_TIME);
    delta_PSI_time = psi_time;
    inv_delta_PSI_time = 1.0f / delta_PSI_time;
}

IVP_Environment::~IVP_Environment(){
    // make shure that you have deleted first all IVP_Real_Object s

    if(environment_magic_number!=IVP_Environment_Magic_Number) {
        return;
	}

    {
	for (int i = psi_listeners.len()-1; i>=0; i--){
	    IVP_Listener_PSI *lis = psi_listeners.element_at(i);
	    lis->environment_will_be_deleted(this);
	}
    }
    
    static_object->delete_silently();
    static_object = NULL;
    P_DELETE(cluster_manager);   // deletes all objects recursively

    P_DELETE(standard_gravity_controller);
    P_DELETE(controller_manager);
    P_DELETE(time_manager);
    P_DELETE(sim_units_manager);
    P_DELETE(mindist_manager);
    P_DELETE(ov_tree_manager);
    P_DELETE(this->better_statisticsmanager);
    
    IVP_IF( delete_debug_information == IVP_TRUE ) {
	P_DELETE(this->debug_information);
    }
    
    performancecounter->environment_is_going_to_be_deleted(this);

    collision_filter->environment_will_be_deleted(this);
    collision_filter = NULL;

    anomaly_manager->environment_will_be_deleted(this);
    anomaly_manager = NULL;
    
    anomaly_limits->environment_will_be_deleted(this);
    anomaly_limits = NULL;
    
    l_active_value_manager->environment_will_be_deleted(this);
    l_active_value_manager = NULL;
    
    l_material_manager->environment_will_be_deleted(this);
    l_material_manager = NULL;
    
    range_manager->environment_will_be_deleted(this);
    range_manager = NULL;

    for (int k = collision_delegator_roots.len()-1;k>=0;k--){
      IVP_Collision_Delegator_Root *cdr = collision_delegator_roots.element_at(k);
      cdr->environment_is_going_to_be_deleted_event(this);
    }
    collision_delegator_roots.clear();
    
    P_DELETE(short_term_mem);
    P_DELETE(sim_unit_mem);
    P_DELETE(cache_object_manager);
    P_DELETE(auth_costumer_name);
    
    environment_manager->environments.remove(this);
    this->delete_draw_vector_debug();
} 

//IVP_BOOL IVP_Environment::must_perform_movement_check() {
//    next_movement_check--;
//    if(next_movement_check==0) {
//#ifdef IVP_ENCRYPT_EXISTS
//        pw_count--;
//        if(pw_count==0) {
//	    IVP_UINT32 crypt_val = IVP_Encrypt::encrypt_strings(auth_costumer_name,IVP_IPION_AUTH_CHECK);
//	    if(auth_costumer_code != crypt_val) {
//	        mindist_manager=NULL;
//	    }
//	    }
//
//	    time_t t, e;
//		t = time(NULL);
//		e = 3181552896; // october 26th 2000, on a Mac
//		printf(" Now: %.1f   Target: %.1f\n\n", (float)t, (float)e);
//		if (t > e)
//	        mindist_manager=NULL;
//#endif	
//	next_movement_check=IVP_MOVEMENT_CHECK_COUNT*3/2 + (short)(ivp_rand()* (IVP_MOVEMENT_CHECK_COUNT/2));
//	return IVP_TRUE;
//    } else {
//	return IVP_FALSE;
//    }
//}

void IVP_Environment::fire_object_is_removed_from_collision_detection(IVP_Real_Object *obj){
    for (int k = collision_delegator_roots.len()-1;k>=0;k--){
      IVP_Collision_Delegator_Root *cdr = collision_delegator_roots.element_at(k);
      cdr->object_is_removed_from_collision_detection(obj);
    }
}

void IVP_Environment::set_gravity(IVP_U_Point *gravity_){
    this->gravity.set(gravity_);
    this->gravity_scalar=gravity_->real_length();
    this->standard_gravity_controller->set_standard_gravity(gravity_);
}

 
IVP_Environment_Manager::IVP_Environment_Manager()
{
    environments.elems=NULL;
	ivp_willamette_optimization=0;
    //    static_environment_manager=this;
}


IVP_Environment_Manager::~IVP_Environment_Manager()
{
/*
    for (int i = environments.len()-1; i>=0; i--){
	IVP_Environment *env = environments.element_at(i);
	//delete(env);
    }
*/
}


IVP_Actuator_Rot_Mot *IVP_Environment::create_rotmot(IVP_Template_Rot_Mot *templ)
{
    if (templ->active_float_max_torque ||
	templ->active_float_max_rotation_speed ||
	templ->active_float_power){
	return new IVP_Actuator_Rot_Mot_Active(this, templ);
    }
    return new IVP_Actuator_Rot_Mot(this, templ);
}

IVP_Actuator_Torque *IVP_Environment::create_torque(IVP_Template_Torque *templ)
{
    if (templ->active_float_torque || templ->active_float_max_rotation_speed){
			return new IVP_Actuator_Torque_Active(this, templ);
    }
    return new IVP_Actuator_Torque(this, templ);
}

IVP_Controller_Motion *IVP_Environment::create_controller_motion(IVP_Real_Object *obj,const IVP_Template_Controller_Motion *templ){
    return new IVP_Controller_Motion( obj, templ);
}

IVP_Actuator_Force *IVP_Environment::create_force(IVP_Template_Force *templ)
{
    if (templ->active_float_force){
	return new IVP_Actuator_Force_Active(this, templ);
    }else{
	return new IVP_Actuator_Force(this,templ);
    }
}

IVP_Actuator_Spring *IVP_Environment::create_spring(IVP_Template_Spring *templ)
{
    if (templ->active_float_spring_len
	|| templ->active_float_spring_constant
	|| templ->active_float_spring_damp
	|| templ->active_float_spring_rel_pos_damp){
	return new IVP_Actuator_Spring_Active(this,templ);
    }else{
	return new IVP_Actuator_Spring(this, templ, IVP_ACTUATOR_TYPE_SPRING);
    }
}

IVP_Actuator_Suspension *IVP_Environment::create_suspension(IVP_Template_Suspension *templ)
{
	return new IVP_Actuator_Suspension(this, templ);
}

IVP_Actuator_Stabilizer *IVP_Environment::create_stabilizer(IVP_Template_Stabilizer *templ)
{
    return new IVP_Actuator_Stabilizer(this, templ);
}

IVP_Actuator_Check_Dist *IVP_Environment::create_check_dist(IVP_Template_Check_Dist *templ)
{
    return new IVP_Actuator_Check_Dist(this, templ);
}


IVP_Cluster *IVP_Environment::get_root_cluster()
{
    return this->cluster_manager->get_root_cluster();
}


IVP_Environment *IVP_Environment_Manager::create_environment(IVP_Application_Environment *appl_env,
							     const char *costumer_name,unsigned int auth_code) {
#ifdef IVP_ENCRYPT_EXISTS
    IVP_UINT32 crypt_val = IVP_Encrypt::encrypt_strings(costumer_name,IVP_IPION_AUTH_CHECK);
    if(auth_code != crypt_val) {
	return NULL;
    }
#endif
    IVP_Environment *new_en=new IVP_Environment(this,appl_env,costumer_name,auth_code);
    return new_en;
}


IVP_Environment_Manager IVP_Environment_Manager::static_environment_manager;
IVP_Environment_Manager *IVP_Environment_Manager::get_environment_manager()
{
    //    if (static_environment_manager == NULL){
    //	static_environment_manager = new IVP_Environment_Manager();
    //    }
    
    return(&static_environment_manager);
}


void IVP_Environment::simulate_until(IVP_Time until_time){
    this->get_betterstatisticsmanager()->set_simulation_time(until_time.get_time());
    time_manager->event_loop(this,until_time);
    //time_manager->simulate_variable_time_step(this,until_time - get_current_time());
}

void IVP_Environment::simulate_variable_time_step(IVP_FLOAT delta_time){
    this->get_betterstatisticsmanager()->set_simulation_time(get_current_time().get_time() + delta_time);
    time_manager->simulate_variable_time_step(this,delta_time);
}

void IVP_Environment::reset_time(){
    time_manager->reset_time(this->get_old_time_of_last_PSI());
    sim_units_manager->reset_time(this->get_old_time_of_last_PSI());
    
    set_current_time( IVP_Time(get_current_time() - get_old_time_of_last_PSI()));
    time_of_last_psi = IVP_Time(0.0f);
    time_of_next_psi = IVP_Time(0.0f) + get_delta_PSI_time();
}

void IVP_Environment::simulate_dtime(IVP_DOUBLE dtime)
{
    IVP_Time new_time =  get_current_time();
    new_time += dtime;
    time_manager->event_loop(this,new_time);
}

void IVP_Environment::simulate_time_step( IVP_FLOAT sub_psi_time){
    IVP_Time new_time =  get_old_time_of_last_PSI();
    new_time += get_delta_PSI_time() * sub_psi_time;
    time_manager->event_loop(this,new_time);
}

void IVP_Environment::set_current_time(IVP_Time time){
    current_time_code++;
    current_time = time;
}

IVP_Draw_Vector_Debug::IVP_Draw_Vector_Debug(){
    P_MEM_CLEAR(this);
}

IVP_Draw_Vector_Debug::~IVP_Draw_Vector_Debug(){
    if (debug_text) P_FREE(debug_text);
}

void IVP_Debug_Manager::clear_debug_manager() {
#if !defined(PSXII)
    if(out_deb_file) {
      if(out_deb_file!=stdout) {
	fclose(out_deb_file);
      }
    }
#endif
}

IVP_Debug_Manager::~IVP_Debug_Manager() {
    clear_debug_manager();
}

void IVP_Debug_Manager::init_debug_manager() {
    P_MEM_CLEAR(this);
    //out_deb_file = fopen("debugout0","w");
    //file_nr=0;
    //file_out_impacts=IVP_TRUE;
    //check_fs=IVP_TRUE;
    //out_deb_file=stdout;
    revolve_deb_file=IVP_FALSE;
}

IVP_Debug_Manager::IVP_Debug_Manager()
{
    init_debug_manager();
}


void IVP_Environment::delete_draw_vector_debug(){
    IVP_Draw_Vector_Debug *dv,*next_dv;

    dv=this->draw_vectors;
    while(dv!=NULL)    {
	if(dv->debug_text)    P_FREE(dv->debug_text);
	next_dv=dv->next;
	P_DELETE(dv);
	dv=next_dv;
    }
    this->draw_vectors=NULL;

}

IVP_Polygon *IVP_Environment::create_polygon(IVP_SurfaceManager *vic, const IVP_Template_Real_Object *templ,
					     IVP_U_Quat const *rotation, const IVP_U_Point *position)
{
    return new IVP_Polygon(get_root_cluster(), vic, templ, rotation, position);
}

IVP_Ball *IVP_Environment::create_ball( const IVP_Template_Ball *templ_ball, const IVP_Template_Real_Object *templ,
					     IVP_U_Quat const *rotation, const IVP_U_Point *position)
{
    return new IVP_Ball(get_root_cluster(), templ_ball, templ, rotation, position);
}

void IVP_Environment::add_listener_object_private(IVP_Real_Object *ro, IVP_Listener_Object *lo)
{
    this->get_cluster_manager()->add_listener_object(ro, lo);
}

void IVP_Environment::remove_listener_object_private(IVP_Real_Object *ro, IVP_Listener_Object *lo)
{
    this->get_cluster_manager()->remove_listener_object(ro, lo);
}

void IVP_Environment::add_listener_collision_private(IVP_Real_Object *ro, IVP_Listener_Collision *lo)
{
    this->get_cluster_manager()->add_listener_collision(ro, lo);
}

void IVP_Environment::remove_listener_collision_private(IVP_Real_Object *ro, IVP_Listener_Collision *lo)
{
    this->get_cluster_manager()->remove_listener_collision(ro, lo);
}



void IVP_Environment::add_listener_object_global(IVP_Listener_Object *listener)
{
    this->global_object_listeners.add(listener);
}

void IVP_Environment::install_listener_object_global(IVP_Listener_Object *listener)
{
    this->global_object_listeners.install(listener);
}

void IVP_Environment::remove_listener_object_global(IVP_Listener_Object *listener)
{
    this->global_object_listeners.remove(listener);
}

void IVP_Environment::fire_event_object_created(IVP_Event_Object *obj_event)
{
    int i;
    for (i=this->global_object_listeners.len()-1; i>=0; i--)
	{
		IVP_Listener_Object *lis = this->global_object_listeners.element_at(i);
		lis->event_object_created(obj_event);
    }
}

void IVP_Environment::fire_event_object_deleted(IVP_Event_Object *obj_event)
{
    int i;
    for (i=this->global_object_listeners.len()-1; i>=0; i--)
	{
		IVP_Listener_Object *lis = this->global_object_listeners.element_at(i);
		lis->event_object_deleted(obj_event);
    }
}


void IVP_Environment::fire_event_object_frozen(IVP_Event_Object *obj_event)
{
    int i;
    for (i=this->global_object_listeners.len()-1; i>=0; i--)
	{
		IVP_Listener_Object *lis = this->global_object_listeners.element_at(i);
		lis->event_object_frozen(obj_event);
    }
}

void IVP_Environment::fire_event_object_revived(IVP_Event_Object *obj_event)
{
    int i;
    for (i=this->global_object_listeners.len()-1; i>=0; i--)
	{
		IVP_Listener_Object *lis = this->global_object_listeners.element_at(i);
		lis->event_object_revived(obj_event);
    }
}

void IVP_Environment::fire_event_pre_collision( IVP_Event_Collision *coll)
{
    int i;
    for ( i = collision_listeners.len()-1; i>=0; i--){
		IVP_Listener_Collision *lis = collision_listeners.element_at(i);
        if (!(lis->get_enabled_callbacks() & IVP_LISTENER_COLLISION_CALLBACK_PRE_COLLISION)) 
			continue;
		lis->event_pre_collision(coll);
    }
}

void IVP_Environment::fire_event_post_collision( IVP_Event_Collision *coll)
{
    int i;
    for ( i = collision_listeners.len()-1; i>=0; i--){
		IVP_Listener_Collision *lis = collision_listeners.element_at(i);
        if (!(lis->get_enabled_callbacks() & IVP_LISTENER_COLLISION_CALLBACK_POST_COLLISION)) 
			continue;
		lis->event_post_collision(coll);
    }
}

void IVP_Environment::fire_event_friction_created( IVP_Event_Friction *coll)
{
    int i;
    for ( i = collision_listeners.len()-1; i>=0; i--){
		IVP_Listener_Collision *lis = collision_listeners.element_at(i);
        if (!(lis->get_enabled_callbacks() & IVP_LISTENER_COLLISION_CALLBACK_FRICTION)) 
			continue;
		lis->event_friction_created(coll);
    }
}

void IVP_Environment::fire_event_friction_deleted( IVP_Event_Friction *coll)
{
    int i;
    for ( i = collision_listeners.len()-1; i>=0; i--)
	{
		IVP_Listener_Collision *lis = collision_listeners.element_at(i);
		if (!(lis->get_enabled_callbacks() & IVP_LISTENER_COLLISION_CALLBACK_FRICTION)) 
			continue;
		lis->event_friction_deleted(coll);
    }
}

void IVP_Environment::fire_event_friction_pair_created( class IVP_Friction_Core_Pair *pair )
{
    int i;
    for ( i = collision_listeners.len()-1; i>=0; i--)
	{
		IVP_Listener_Collision *lis = collision_listeners.element_at(i);
		if (!(lis->get_enabled_callbacks() & IVP_LISTENER_COLLISION_CALLBACK_FRICTION)) 
			continue;
		lis->event_friction_pair_created(pair);
    }
}

void IVP_Environment::fire_event_friction_pair_deleted( class IVP_Friction_Core_Pair *pair  )
{
    int i;
    for ( i = collision_listeners.len()-1; i>=0; i--)
	{
		IVP_Listener_Collision *lis = collision_listeners.element_at(i);
		if (!(lis->get_enabled_callbacks() & IVP_LISTENER_COLLISION_CALLBACK_FRICTION)) 
			continue;
		lis->event_friction_pair_deleted(pair);
    }
}

void IVP_Environment::fire_event_PSI()
{
    int i;
    IVP_Event_PSI event;
    event.environment = this;
    for ( i = psi_listeners.len()-1; i>=0; i--)
	{
		IVP_Listener_PSI *lis = psi_listeners.element_at(i);
		lis->event_PSI(&event);
    }
}

// begin: Constraint listeners
void IVP_Environment::fire_event_constraint_broken(IVP_Constraint *constraint)
{
    int i;
    for (i=this->constraint_listeners.len()-1; i>=0; i--)
	{
		IVP_Listener_Constraint *lis = this->constraint_listeners.element_at(i);
		lis->event_constraint_broken(constraint);
    }
}

#ifdef HAVANA_CONSTRAINTS
void IVP_Environment::fire_event_constraint_broken(hk_Breakable_Constraint* constraint)
{
	int i;
	
	for(i = constraint_listeners.len() - 1; i >= 0; i--)
	{
		IVP_Listener_Constraint *lis = constraint_listeners.element_at(i);
		lis->event_constraint_broken(constraint);
	}
}
#endif // HAVANA_CONSTRAINTS

// end: Constraint listeners

void IVP_Environment::add_listener_collision_global(IVP_Listener_Collision *listener)
{
    collision_listeners.add(listener);
}

void IVP_Environment::remove_listener_collision_global(IVP_Listener_Collision *listener)
{
    collision_listeners.remove(listener);
}

void IVP_Environment::add_listener_PSI(IVP_Listener_PSI *listener)
{
    psi_listeners.add(listener);
}

void IVP_Environment::remove_listener_PSI(IVP_Listener_PSI *listener)
{
    psi_listeners.remove(listener);
}

// Constraint listener
void IVP_Environment::add_listener_constraint_global(IVP_Listener_Constraint *listener)
{
	constraint_listeners.add(listener);
}
void IVP_Environment::remove_listener_constraint_global(IVP_Listener_Constraint *listener)
{
	constraint_listeners.remove(listener);
}

//lwss add
void IVP_Environment::force_psi_on_next_simulation()
{
    // unsure if get_next_event() is correct.
    this->time_manager->update_event( this->time_manager->get_next_event(), this->current_time );
}
//lwss end
void IVP_Environment::merge_objects(IVP_U_Vector<IVP_Real_Object> *objs_to_merge) 
{
	IVP_Merge_Core merge_all_objs;
	int obj_num=objs_to_merge->len();
	merge_all_objs.n_cores=obj_num;
	IVP_Real_Object *my_obj=objs_to_merge->element_at(0);
	IVP_Core_Merged *merged_core_for_all=new IVP_Core_Merged(my_obj);
	merge_all_objs.mcore=merged_core_for_all;
	merge_all_objs.cores=(IVP_Core **)p_malloc(obj_num*sizeof(IVP_Core*));
	int i;
	for(i=0; i< objs_to_merge->len(); i++) {
		my_obj = objs_to_merge->element_at(i);
		IVP_Core *old_core = my_obj->get_core();
		if (IVP_MTIS_SIMULATED(old_core->movement_state)){
			//	        old_core->stop_physical_movement();
			//		this->core_sim_manager->remove_sim_core(old_core);
		}
		
		merge_all_objs.cores[i]=my_obj->physical_core;
		merge_all_objs.core_stack[i]=my_obj->physical_core;
		my_obj->physical_core=merged_core_for_all;
		my_obj->friction_core=merged_core_for_all;
		my_obj->original_core=merged_core_for_all;
		merged_core_for_all->core_add_link_to_obj(my_obj);
	}
	
	merge_all_objs.calc_calc(); //now the merged core is valid
	
	IVP_Event_Sim es(this, get_next_PSI_time() - get_current_time());
	if (!IVP_MTIS_SIMULATED(merge_all_objs.movement_type) ){
		merged_core_for_all->calc_next_PSI_matrix_zero_speed( &es);
	}else{
		//	    this->core_sim_manager->add_sim_core(merged_core_for_all,0);
		merged_core_for_all->init_core_for_simulation();
		IVP_Calc_Next_PSI_Solver nps(merged_core_for_all);
		
		IVP_Vector_of_Hull_Managers_1 active_hull_managers;
		nps.calc_next_PSI_matrix( &es, &active_hull_managers );
		nps.commit_one_hull_manager( this, &active_hull_managers);
	}
	
	for(i=0;i<obj_num;i++) {
		delete merge_all_objs.cores[i];
	}
	P_FREE( merge_all_objs.cores );
}
// needed for sun to initialize templates classes
void ivp_dummy_func(){
    IVP_U_Set_Active<IVP_Core> ivp_class_dummy1(16);
    IVP_U_Set_Active<IVP_Real_Object> ivp_class_dummy2(16);
    IVP_U_Set_Active<IVP_Mindist_Base> ivp_class_dummy3(16);
    IVP_Attacher_To_Cores<IVP_Controller_Buoyancy> *ivp_class_dummy8 = new IVP_Attacher_To_Cores<IVP_Controller_Buoyancy>(NULL);
    ivp_class_dummy8 = ivp_class_dummy8;
}

void IVP_Time_Event_D::simulate_time_event(IVP_Environment *env) {
#if 1
	int nr=number_of_sons;
    int i;
    for(i=0;i<=nr;i++) {
        IVP_Time_Event_D *delay_event=new IVP_Time_Event_D(env->get_current_time());
        delay_event->number_of_sons=nr-1;
        env->get_time_manager()->insert_event(delay_event,env->get_current_time());
    }
    P_DELETE_THIS(this);
#else
    P_DELETE_THIS(this);    
#endif
}

void IVP_Time_Event_N::simulate_time_event(IVP_Environment *env) {
    //env->get_time_manager()->event_function=time_man_sim_until;
    P_DELETE(env->get_time_manager()->event_manager);
    env->get_time_manager()->event_manager=new IVP_Event_Manager_Standard();
    env->get_time_manager()->event_manager->mode=0;
    P_DELETE_THIS(this);
}


void IVP_Environment::do_d_events() {
#ifdef PSXII
    for(int i=0;i<20;i++) {
	IVP_Time_Event_D *delay_event=new IVP_Time_Event_D(this->get_current_time());
	delay_event->number_of_sons=7;
	this->get_time_manager()->insert_event(delay_event,this->get_current_time());
    }
#else
    for(int i=0;i<5;i++) {
	IVP_Time_Event_D *delay_event=new IVP_Time_Event_D(this->get_current_time());
	delay_event->number_of_sons=8;
	this->get_time_manager()->insert_event(delay_event,this->get_current_time());
    }
#endif    
}


IVP_Time_Event_D::IVP_Time_Event_D(IVP_Time time) {
    IVP_USE(time);
}

IVP_Time_Event_N::IVP_Time_Event_N(IVP_Time time) {
    IVP_USE(time);
}


 
void IVP_Environment::add_draw_vector(const IVP_U_Point *start_p, const IVP_U_Float_Point *dir_vec,const char *debug_text_in,int v_color)
{
	IVP_IF(1){
    IVP_Draw_Vector_Debug *draw_vector=new IVP_Draw_Vector_Debug();
    draw_vector->first_point=*start_p;
    draw_vector->direction_vec=*dir_vec;
    draw_vector->debug_text=p_strdup(debug_text_in);
    draw_vector->color=v_color;
    draw_vector->next=this->draw_vectors;
    this->draw_vectors=draw_vector;
	}
}

#ifdef PSXII
	int ivp_bcd2dec(int bcd)
	{
		int dec = 0;
		int sign = (bcd < 0 ? -1 : 1);
		for (int order = 1; bcd; order*=10)
		{
			int i = bcd % 16;
			IVP_ASSERT(i >= 0 && i <= 9);
			bcd /= 16;
			dec += order * i;
		}
		return dec*sign;
	}

	tm ivp_ps2_getTime()
	{
		sceCdCLOCK rtc;
		int success = sceCdReadClock(&rtc);
		IVP_ASSERT(success);
		tm mytime;
		mytime.tm_sec  = ivp_bcd2dec(rtc.second);
		mytime.tm_min  = ivp_bcd2dec(rtc.minute);
		mytime.tm_hour = ivp_bcd2dec(rtc.hour);
		mytime.tm_mday = ivp_bcd2dec(rtc.day);
		mytime.tm_mon  = ivp_bcd2dec(rtc.month)-1;
		int year       = ivp_bcd2dec(rtc.year);
		mytime.tm_year = (year < 70 ? year + 100 : year);
		mytime.tm_isdst = -1;
		return mytime;
	}

	int ivp_ps2_sec_since_1970() {
		tm mytime = ivp_ps2_getTime();
		return (int)mktime(&mytime);		
	}
#endif

void IVP_Environment::simulate_psi(IVP_Time /*psi_time*/){

    IVP_IF(1 || this->debug_information->display_statistic)
    {
	if( get_current_time()- get_statistic_manager()->last_statistic_output >= 1.0f ) {
	    get_statistic_manager()->output_statistic();
	    get_statistic_manager()->last_statistic_output = get_current_time();
	}
    }
    IVP_IF(1) {
	get_debug_manager()->psi_counter += 1.0f;
        delete_draw_vector_debug();
    }


#ifdef IVP_ENABLE_PERFORMANCE_COUNTER
    get_performancecounter()->pcount(IVP_PE_PSI_UNIVERSE);
#endif
    
    state = IVP_ES_PSI;	
    if ( core_revive_list.len() ){
    	revive_cores_PSI();
    }

    IVP_Universe_Manager *um = this->get_universe_manager();
    if (um){
	    IVP_Cluster_Manager *cm = get_cluster_manager();
	    cm->check_for_unused_objects(um);
    }

    l_active_value_manager->refresh_psi_active_values(this);	// lots of side effects, bombs ....
    fire_event_PSI();		// inform psi listeners //???

	get_mindist_manager()->recalc_all_exact_wheel_mindist();    

#ifdef IVP_ENABLE_PERFORMANCE_COUNTER
    get_performancecounter()->pcount(IVP_PE_PSI_CONTROLLERS);
#endif
    IVP_Vector_of_Cores_128 touched_cores;
    get_sim_units_manager()->simulate_sim_units_psi(this, &touched_cores);


#ifdef IVP_ENABLE_PERFORMANCE_COUNTER
    get_performancecounter()->pcount(IVP_PE_PSI_INTEGRATORS);
#endif    
    IVP_Vector_of_Hulls_128 active_hulls;
    IVP_Calc_Next_PSI_Solver::commit_all_calc_next_PSI_matrix(this, &touched_cores, &active_hulls);



#ifdef IVP_ENABLE_PERFORMANCE_COUNTER
    get_performancecounter()->pcount(IVP_PE_PSI_HULL);
#endif
    state = IVP_ES_PSI_HULL;
    IVP_Calc_Next_PSI_Solver::commit_all_hull_managers( this, &active_hulls);

#ifdef IVP_ENABLE_PERFORMANCE_COUNTER
    get_performancecounter()->pcount(IVP_PE_PSI_SHORT_MINDISTS);
#endif
    state = IVP_ES_PSI_SHORT;
    get_mindist_manager()->recalc_all_exact_mindists();


#ifdef IVP_ENABLE_PERFORMANCE_COUNTER
    get_performancecounter()->pcount(IVP_PE_PSI_CRITICAL_MINDISTS);
#endif
    state = IVP_ES_PSI_CRITIC;
    get_mindist_manager()->recalc_all_exact_mindists_events();


#ifdef IVP_ENABLE_PERFORMANCE_COUNTER
	get_performancecounter()->pcount(IVP_PE_PSI_END);
#endif


#if (defined(WIN32) || defined(PSXII)) && defined(IVP_VERSION_EVAL)  /* blocking only if original ivp_authenticity.hxx is used */
    {
        // IVP_BLOCKING_EVERY_MIN    
	time_since_last_blocking += get_delta_PSI_time();
	//do some blocking from time to time
	if( time_since_last_blocking>121.1f) {
	    time_since_last_blocking=0.0f;

		int t;

#ifdef IVP_BLOCKING_ALWAYS
	    this->do_d_events();
#else

#ifdef WIN32
	    time_t tt = time(NULL);
		t = (int)tt;
#endif
#ifdef PSXII
		t = ivp_ps2_sec_since_1970();
#endif

	    if ( t > 983404800 // 1 Mar 01 @@CB
		+ 60*60*24* (
		31 + 30 + 31 ) ){ // expiration date 31 May 01 @@CB
		this->do_d_events();
	    }
#endif
	}
    }
#endif


    this->state = IVP_ES_AT;
    return;
}

//lwss add
float IVP_Environment::get_global_collision_tolerance()
{
    return ivp_mindist_settings.min_coll_dists;
}
void IVP_Environment::set_global_collision_tolerance( IVP_DOUBLE tolerance, IVP_DOUBLE gravLen ){
    ivp_mindist_settings.set_collision_tolerance( tolerance, gravLen );
}
//lwss end


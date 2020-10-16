// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#ifndef WIN32
#	 pragma implementation "ivp_controller_motion.hxx"
#	 pragma implementation "ivp_controller_golem.hxx"
#endif

#include <ivp_controller_motion.hxx>
#include <ivp_controller_golem.hxx>

IVP_Template_Controller_Motion::IVP_Template_Controller_Motion(){
    P_MEM_CLEAR(this);
    max_translation_force.set( 1e6f, 1e6f, 1e6f);
    max_torque = 1e6f;
    force_factor = 0.8f;
    damp_factor = 1.0f;

    torque_factor = 0.8f;
    angular_damp_factor = 1.0f;
}

IVP_Controller_Motion::IVP_Controller_Motion(IVP_Real_Object *obj, const IVP_Template_Controller_Motion *templ)
{
    this->target_pos_ws.set_to_zero();
    this->target_q_world_f_core.init();

    max_translation_force.set( &templ->max_translation_force);
    max_torque.set(templ->max_torque, templ->max_torque, templ->max_torque);
    force_factor = templ->force_factor;
    damp_factor = templ->damp_factor;

    torque_factor = templ->torque_factor;
    angular_damp_factor = templ->angular_damp_factor;
    
    this->l_environment = obj->get_environment();
    this->core = obj->get_core();
    this->real_object = obj;
    this->l_environment->get_controller_manager()->add_controller_to_core(this, core);
}



void IVP_Controller_Motion::set_max_torque(const IVP_U_Float_Point *max_t) {
    max_torque=*max_t;
}

void IVP_Controller_Motion::set_max_translation_force(const IVP_U_Float_Point *max_tf) {
    max_translation_force=max_tf;
}

void IVP_Controller_Motion::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> *core_list) {
    // ------------------------------------------------------------------------
    // Try to push object so that it reaches specified positions within d_time
    //
    // Note: This function is called during PSI
    // ------------------------------------------------------------------------
    
    // Little entry checks
    IVP_ASSERT(core_list->len() == 1); // Multiple cores don't make much sense here, do they?

    // current situation
    const IVP_U_Matrix *m_world_f_core = core->get_m_world_f_core_PSI();
    const IVP_U_Point *cur_pos_ws = m_world_f_core->get_position();

    // ---------------------------------------------------------
    // Translation
    // ---------------------------------------------------------
    
    // extrapolate position

    IVP_U_Float_Point delta_position;  delta_position.inline_subtract_and_mult( &this->target_pos_ws, cur_pos_ws, this->force_factor);
    delta_position.add_multiple(&core->speed, -es->delta_time * this->damp_factor);

    // calc necessary speed change
    IVP_DOUBLE inv_d_time = es->i_delta_time;
    {

	IVP_DOUBLE qidt = es->i_delta_time * es->i_delta_time;
	for(int i=2; i>=0; i--){
	    IVP_FLOAT axis_force = delta_position.k[i] * core->get_mass() * qidt;
	    if(IVP_Inline_Math::fabsd(axis_force) < max_translation_force.k[i]) continue;
	    
	    if (delta_position.k[i] < 0){
		delta_position.k[i] =  - max_translation_force.k[i] * core->get_inv_mass() * es->delta_time * es->delta_time;
	    }else{
		delta_position.k[i] = max_translation_force.k[i] * core->get_inv_mass() * es->delta_time * es->delta_time;
	    }
	    // clip force
	}
	core->speed.add_multiple(&delta_position, inv_d_time);
    }
    
    // ---------------------------------------------------------
    // Rotation
    // ---------------------------------------------------------
    {

	IVP_U_Quat target_q_core_f_world;	target_q_core_f_world.set_invert_unit_quat(&target_q_world_f_core);
	IVP_U_Quat delta_quat_core_f_core;	delta_quat_core_f_core.set_mult_quat(&target_q_core_f_world, &core->q_world_f_core_next_psi);

	IVP_U_Float_Point delta_angles_now_cs;
	delta_quat_core_f_core.get_angles(&delta_angles_now_cs); 

	if ( target_q_world_f_core.acos_quat(&core->q_world_f_core_next_psi) < 0 ){
	    delta_angles_now_cs.mult(this->torque_factor * inv_d_time);
	}else{
	    delta_angles_now_cs.mult(-this->torque_factor * inv_d_time);	    
	}
	

	IVP_U_Float_Point rot_speed_change_cs;
	rot_speed_change_cs.add_multiple(&delta_angles_now_cs, &core->rot_speed, -this->angular_damp_factor);

	for(int i=2; i>=0; --i){
	    IVP_FLOAT rot_force = rot_speed_change_cs.k[i] * inv_d_time * core->get_rot_inertia()->k[i];
	    if(IVP_Inline_Math::fabsd(rot_force) < max_torque.k[i]) continue;
	    	    
	    // clip rot change so that resulting rot force doesn't exceed max rot force
	    if ( rot_force < 0){
	      rot_speed_change_cs.k[i] = -max_torque.k[i] * core->get_inv_rot_inertia()->k[i] * es->delta_time;
	    }else{
	      rot_speed_change_cs.k[i] = max_torque.k[i] * core->get_inv_rot_inertia()->k[i] * es->delta_time;
	    }
	}
	core->rot_speed.add(&rot_speed_change_cs);
    }
}

void IVP_Controller_Motion::core_is_going_to_be_deleted_event(IVP_Core *core_i){
    IVP_ASSERT(core == core_i);
    P_DELETE_THIS(this);
}
    
IVP_Controller_Motion::~IVP_Controller_Motion()
{
    this->l_environment->get_controller_manager()->remove_controller_from_core(this, core);
}

void IVP_Controller_Motion::set_target_position_ws(const IVP_U_Point * position){
	if (position->quad_distance_to(&target_pos_ws) == 0) return;

	target_pos_ws = *position;
	
	l_environment->get_controller_manager()->ensure_core_in_simulation(core);
}

void IVP_Controller_Motion::set_target_object_position_ws(IVP_Real_Object *ro, const IVP_U_Quat * desired_orientation,const  IVP_U_Point *position_of_object_space_center_ws){
    if ( !real_object->flags.shift_core_f_object_is_zero ){
	IVP_U_Matrix3 m;
	desired_orientation->set_matrix(&m);

	IVP_U_Float_Point shift_world_f_object;
	m.vmult3( ro->get_shift_core_f_object(), & shift_world_f_object);

	IVP_U_Point shift_wo(shift_world_f_object);
        IVP_U_Point pos_ws;	pos_ws.subtract( position_of_object_space_center_ws, &shift_wo);
        set_target_position_ws( &pos_ws );
    }else{
        set_target_position_ws( position_of_object_space_center_ws );
    }
}

void IVP_Controller_Motion::set_target_q_world_f_core(const IVP_U_Quat * orientation){
	if (IVP_Inline_Math::fabsd(orientation->acos_quat(&target_q_world_f_core) - 1.0f) < P_DOUBLE_EPS) return;

	target_q_world_f_core = *orientation;
	l_environment->get_controller_manager()->ensure_core_in_simulation(core);
}

IVP_Template_Controller_Golem::IVP_Template_Controller_Golem(){
    max_delta_position = 10.0f;
    max_delta_orientation = IVP_FLOAT(0.5f * IVP_PI);
    max_integrated_delta_position = 10.0f;
    max_golem_force = P_FLOAT_MAX;
    filter_dtime = 5.0f;
}


IVP_Controller_Golem::IVP_Controller_Golem(IVP_Real_Object *o,const IVP_Template_Controller_Golem *t):
IVP_Controller_Motion(o, t){
    integrated_delta_position = 0.0f;


    max_delta_position = t->max_delta_position;
    max_delta_orientation = t->max_delta_orientation;
    max_integrated_delta_position = t->max_integrated_delta_position;
    max_golem_force = t->max_golem_force;
    filter_dtime = t->filter_dtime;

    angular_velocity_set = IVP_FALSE;
    i_delta_prime_orientation_time = 1.0f;
    prime_position_ws.set_to_zero();
    velocity_ws.set_to_zero();
    prime_orientation_0.init();
    prime_orientation_1.init();

}

IVP_Controller_Golem::~IVP_Controller_Golem(){

}

void IVP_Controller_Golem::beam_object_to_target_position(IVP_Event_Sim *es){
    real_object->beam_object_to_new_position( &target_q_world_f_core, &target_pos_ws, IVP_TRUE);
}

void IVP_Controller_Golem::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> *cores){
        // Little entry checks
    IVP_ASSERT(cores->len() == 1); // Multiple cores don't make much sense here, do they?

    IVP_Time current_time = es->environment->get_current_time();

    // position
    {
	IVP_FLOAT dt = current_time - time_of_prime_position;
	target_pos_ws.add_multiple( &prime_position_ws, &velocity_ws, dt);
    }

    // orientation
    if ( !angular_velocity_set ){
	target_q_world_f_core = prime_orientation_0;
    }else{
	IVP_FLOAT dt = current_time - time_of_prime_orientation_0;
	target_q_world_f_core.set_interpolate_smoothly( &prime_orientation_0, &prime_orientation_1, dt * i_delta_prime_orientation_time);
    }

    // current situation
    const IVP_U_Matrix *m_world_f_core = core->get_m_world_f_core_PSI();
    const IVP_U_Point *cur_pos_ws = m_world_f_core->get_position();

    // ---------------------------------------------------------
    // Translation
    // ---------------------------------------------------------
    
    IVP_U_Float_Point delta_position;  delta_position.subtract( &this->target_pos_ws, cur_pos_ws);


    if (!real_object->flags.shift_core_f_object_is_zero){
	IVP_U_Float_Point shift_cs_os_ws;
	m_world_f_core->vmult3( real_object->get_shift_core_f_object(), &shift_cs_os_ws);
	delta_position.subtract( &shift_cs_os_ws );
    }

    
    IVP_DOUBLE qdist = delta_position.quad_length();
    integrated_delta_position = integrated_delta_position * 0.9f  + qdist;

    if ( qdist > max_delta_position * max_delta_position){
	resolve_for_problem(es,IVP_GP_FAR_DISTANCE);
	return;
    }

    // extrapolate position
    delta_position.mult(this->force_factor);
    delta_position.add_multiple(&velocity_ws,  es->delta_time * this->damp_factor);
    delta_position.add_multiple(&core->speed, -es->delta_time * this->damp_factor);

    // calc necessary speed change
    IVP_DOUBLE inv_d_time = es->i_delta_time;
    {
	IVP_DOUBLE qidt = es->i_delta_time * es->i_delta_time;
	for(int i=2; i>=0; i--){
	    IVP_FLOAT axis_force = delta_position.k[i] * core->get_mass() * qidt;
	    if(IVP_Inline_Math::fabsd(axis_force) < max_translation_force.k[i]) continue;
	    
	    if (delta_position.k[i] < 0){
		delta_position.k[i] =  - max_translation_force.k[i] * core->get_inv_mass() * es->delta_time * es->delta_time;
	    }else{
		delta_position.k[i] = max_translation_force.k[i] * core->get_inv_mass() * es->delta_time * es->delta_time;
	    }
	    // clip force
	}
	core->speed.add_multiple(&delta_position, inv_d_time);
    }
    
    // ---------------------------------------------------------
    // Rotation
    // ---------------------------------------------------------
    {

	IVP_U_Quat target_q_core_f_world;	target_q_core_f_world.set_invert_unit_quat(&target_q_world_f_core);
	IVP_U_Quat delta_quat_core_f_core;	delta_quat_core_f_core.set_mult_quat(&target_q_core_f_world, &core->q_world_f_core_next_psi);

	IVP_U_Float_Point delta_angles_now_cs;
	delta_quat_core_f_core.get_angles(&delta_angles_now_cs);
	
	if ( delta_angles_now_cs.quad_length() > max_delta_orientation * max_delta_orientation){
    	    resolve_for_problem(es,IVP_GP_BIG_ANGLE);
	    return;
	}

	if ( target_q_world_f_core.acos_quat(&core->q_world_f_core_next_psi) < 0 ){
	    delta_angles_now_cs.mult(this->torque_factor * inv_d_time);
	}else{
	    delta_angles_now_cs.mult(-this->torque_factor * inv_d_time);	    
	}
	

	IVP_U_Float_Point rot_speed_change_cs;
	rot_speed_change_cs.add_multiple(&delta_angles_now_cs, &core->rot_speed, -this->damp_factor);

	for(int i=2; i>=0; --i){
	    IVP_FLOAT rot_force = rot_speed_change_cs.k[i] * inv_d_time * core->get_rot_inertia()->k[i];
	    if(IVP_Inline_Math::fabsd(rot_force) < max_torque.k[i]) continue;
	    	    
	    // clip rot change so that resulting rot force doesn't exceed max rot force
	    if ( rot_force < 0){
	      rot_speed_change_cs.k[i] = -max_torque.k[i] * core->get_inv_rot_inertia()->k[i] * es->delta_time;
	    }else{
	      rot_speed_change_cs.k[i] = max_torque.k[i] * core->get_inv_rot_inertia()->k[i] * es->delta_time;
	    }
	}
	core->rot_speed.add(&rot_speed_change_cs);
    }
}


void IVP_Controller_Golem::set_prime_position( const IVP_U_Point * position, const IVP_U_Float_Point * velocity ,const IVP_Time & time ){
    time_of_prime_position = time;
    prime_position_ws.set(position);
    velocity_ws.set(velocity);
    l_environment->get_controller_manager()->ensure_core_in_simulation(core);
}

void IVP_Controller_Golem::reset_time( IVP_Time offset){
    time_of_prime_position -= offset;
}

void IVP_Controller_Golem::set_prime_orientation( const IVP_U_Quat * orientation0,const  IVP_Time & time0,const IVP_U_Quat * orientation1, IVP_FLOAT dt ){
    prime_orientation_0 = *orientation0;

    if (orientation1){
	prime_orientation_1 = *orientation1;
	angular_velocity_set = IVP_TRUE;
        i_delta_prime_orientation_time = 1.0f / dt;
    }else{
	angular_velocity_set = IVP_FALSE;
    }
    time_of_prime_orientation_0 = time0;
    l_environment->get_controller_manager()->ensure_core_in_simulation(core);
}

// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


// IVP_EXPORT_PROTECTED

#include <ivp_physics.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_template_constraint.hxx>
#include <ivp_solver_core_reaction.hxx>
#ifndef WIN32
#	 pragma implementation "ivp_constraint_fixed_keyed.hxx"
#endif

#include <ivp_constraint_fixed_keyed.hxx>

IVP_Template_Constraint_Fixed_Keyframed::IVP_Template_Constraint_Fixed_Keyframed(){
}

    
IVP_Constraint_Fixed_Keyframed::~IVP_Constraint_Fixed_Keyframed(){
	IVP_Controller_Manager::remove_controller_from_environment(this,IVP_TRUE); //delete silently
}

void IVP_Constraint_Fixed_Keyframed::core_is_going_to_be_deleted_event(IVP_Core *) {
    P_DELETE_THIS(this);
}

IVP_DOUBLE IVP_Constraint_Fixed_Keyframed::get_minimum_simulation_frequency(){
	return 1.0f;
}

void IVP_Constraint_Fixed_Keyframed::ensure_in_simulation(){
	get_environment()->get_controller_manager()->ensure_controller_in_simulation(this);
}

/*
IVP_Template_Constraint &ivp_create_template_for_constraint_local(IVP_Real_Object *reference_object,
								 IVP_Real_Object *attached_object,
								 const IVP_Template_Constraint_Fixed_Keyframed *tin){
    static IVP_Template_Constraint t;
    t.set_fixed( reference_object, attached_object);

	IVP_Cache_Object *co_attached = attached_object->get_cache_object_no_lock();
	IVP_U_Point co_pos_ws( co_attached->m_world_f_object.get_position());

	IVP_U_Point co_pos_Ros;
	IVP_Cache_Object *co_reference = reference_object->get_cache_object_no_lock();
	co_reference->transform_position_to_object_coords( & co_pos_ws, &co_pos_Ros);

    t.set_fixing_point_Ros( &co_pos_Ros);
    return t;
}
*/
//#define WATCHPOINT 0x56b56f0
//#define WATCHPOINT 0x381e810


IVP_Constraint_Fixed_Keyframed::IVP_Constraint_Fixed_Keyframed(IVP_Real_Object *reference_object, IVP_Real_Object *attached_object,
							       const IVP_Template_Constraint_Fixed_Keyframed *t)
	//:IVP_Constraint_Local( ivp_create_template_for_constraint_local(reference_object,attached_object,t ))
	{
    if (!reference_object->get_core()->physical_unmoveable){
		cores_of_constraint_system.add(reference_object->get_core());
    }
    if (!attached_object->get_core()->physical_unmoveable){
		cores_of_constraint_system.add(attached_object->get_core());
    }
    reference_obj = reference_object;
    attached_obj = attached_object;


    max_translation_force.set( &t->max_translation_force);
    max_torque.set(t->max_torque, t->max_torque, t->max_torque);
    force_factor = t->force_factor;
    damp_factor = t->damp_factor;

    torque_factor = t->torque_factor;
    angular_damp_factor = t->angular_damp_factor;
    
	l_environment = reference_object->get_environment();

    time_of_prime_orientation_0 = 0.0;
    velocity_Ros.set_to_zero();

	IVP_U_Matrix m_Ros_f_Aos;
	IVP_Cache_Object *co_attached = attached_object->get_cache_object();
	IVP_Cache_Object *co_reference = reference_object->get_cache_object();
	co_reference->m_world_f_object.mimult4(&co_attached->m_world_f_object, &m_Ros_f_Aos);
	co_attached->remove_reference();
	co_reference->remove_reference();

	//if (int(this) == WATCHPOINT){
		//IVP_DOUBLE x = velocity_Ros.quad_length();

    prime_position_Ros.set( m_Ros_f_Aos.get_position());
    prime_orientation_0.set_quaternion( &m_Ros_f_Aos );

    angular_velocity_set = IVP_FALSE;
    get_environment()->get_controller_manager()->announce_controller_to_environment(this);
}



void IVP_Constraint_Fixed_Keyframed::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> *cores){
    IVP_Time current_time = es->environment->get_current_time();

	//if (int(this) == WATCHPOINT)
	//	IVP_DOUBLE x = current_time.get_time();

    IVP_Cache_Object *co_Ref = reference_obj->get_cache_object();
    IVP_Cache_Object *co_Att = attached_obj->get_cache_object();

    const IVP_U_Matrix *m_Rws_f_Ros = &co_Ref->m_world_f_object;
    const IVP_U_Matrix *m_Aws_f_Aos = &co_Att->m_world_f_object;


    IVP_U_Float_Point x_rot_axis_world(1,0,0);
    IVP_U_Float_Point y_rot_axis_world(0,1,0);
    IVP_U_Float_Point z_rot_axis_world(0,0,1);

    IVP_Core *core_A = (reference_obj->get_core()->physical_unmoveable)? NULL : reference_obj->get_core();
    IVP_Core *core_B = (attached_obj->get_core()->physical_unmoveable)? NULL : attached_obj->get_core();

    // orientation
    {
	IVP_FLOAT dt = current_time - time_of_prime_orientation_0;

	IVP_U_Quat new_q_Ros_f_Rrs;
	if (angular_velocity_set){
		new_q_Ros_f_Rrs.set_interpolate_smoothly( &prime_orientation_0, &prime_orientation_1,
		(dt + es->delta_time ) * i_delta_prime_orientation_time);
	}else{
		new_q_Ros_f_Rrs = prime_orientation_0;
	}


	IVP_U_Matrix3 m_Ros_f_Rrs;	new_q_Ros_f_Rrs.set_matrix( &m_Ros_f_Rrs );
	IVP_U_Matrix3 m_Rws_f_Rrs;	m_Rws_f_Ros->mmult3( & m_Ros_f_Rrs, &m_Rws_f_Rrs );
	IVP_U_Matrix3 m_Rws_f_Aws;	m_Aws_f_Aos->mi2mult3( &m_Rws_f_Rrs, &m_Rws_f_Aws );
	IVP_U_Quat q;	q.set_quaternion(&m_Rws_f_Aws);
	IVP_U_Float_Point drRA_rs(2.0f * IVP_Inline_Math::fast_asin(q.x),
				  2.0f * IVP_Inline_Math::fast_asin(q.y),
				  2.0f * IVP_Inline_Math::fast_asin(q.z));

	if ( q.w > 0.0f){
	    drRA_rs.mult(-1.0f);
	}


	IVP_Solver_Core_Reaction tcb;
	tcb.init_reaction_solver_rotation_ws(core_B, core_A, &x_rot_axis_world, &y_rot_axis_world, &z_rot_axis_world);

	//// invert matrix
	IVP_RETURN_TYPE r = tcb.invert_3x3_matrix();
	if(r==IVP_FAULT){
	printf("do_constraint_system: Couldn't invert rot matrix!\n");
	co_Ref->remove_reference();
	co_Att->remove_reference();
	return;
	}
	IVP_U_Matrix3 &tpm = tcb.m_velocity_ds_f_impulse_ds;
	IVP_U_Float_Point delta_angles;
	delta_angles.set_multiple( &drRA_rs, es->i_delta_time * this->torque_factor);
	delta_angles.add_multiple( &tcb.delta_velocity_ds, -this->angular_damp_factor );

	IVP_U_Float_Point rot_impulse_ws;
	tpm.vmult3( & delta_angles, &rot_impulse_ws);
	tcb.exert_angular_impulse_dim3(core_B, core_A, rot_impulse_ws);
    }

    // position
    {
	IVP_FLOAT dt = current_time - time_of_prime_position;
	IVP_U_Point new_target_position_Ros;
	new_target_position_Ros.add_multiple( &prime_position_Ros, &velocity_Ros, dt + es->delta_time);

	IVP_U_Point target_pos_Ws;
	m_Rws_f_Ros->vmult4( & new_target_position_Ros, &target_pos_Ws );



	IVP_U_Float_Point delta_position_ws; delta_position_ws.inline_subtract_and_mult( 
		&target_pos_Ws, m_Aws_f_Aos->get_position(), this->force_factor * es->i_delta_time);

	delta_position_ws.add_multiple( &reference_obj->get_core()->speed, damp_factor );
	delta_position_ws.add_multiple( &attached_obj->get_core()->speed, -damp_factor );

	co_Ref->remove_reference();
	co_Att->remove_reference();

	IVP_Solver_Core_Reaction tcb;
	tcb.init_reaction_solver_translation_ws(core_B, core_A, target_pos_Ws, &x_rot_axis_world, &y_rot_axis_world, &z_rot_axis_world);

	//// invert matrix
	IVP_RETURN_TYPE r = tcb.invert_3x3_matrix();
	if(r==IVP_FAULT){
	printf("do_constraint_system: Couldn't invert rot matrix!\n");
	return;
	}
	IVP_U_Matrix3 &tpm = tcb.m_velocity_ds_f_impulse_ds;	
	IVP_U_Float_Point impulse_ws;	  tpm.vmult3( & delta_position_ws, &impulse_ws);
	tcb.exert_impulse_dim3(core_B, core_A, impulse_ws);
   }


}

void IVP_Constraint_Fixed_Keyframed::set_prime_position_Ros(const IVP_U_Point * position,const IVP_U_Float_Point * velocity ,const  IVP_Time & time ){
    time_of_prime_position = time;
    prime_position_Ros.set(position);
    velocity_Ros.set(velocity);
//	if (int(this) == WATCHPOINT)
//		IVP_DOUBLE x = velocity->quad_length();
    this->ensure_in_simulation();
}

void IVP_Constraint_Fixed_Keyframed::set_prime_orientation_Ros( const IVP_U_Quat * orientation0,const IVP_Time & time0, const IVP_U_Quat * orientation1,const IVP_FLOAT dt ){
    prime_orientation_0 = *orientation0;
	if ( orientation1 ){
	    prime_orientation_1 = *orientation1;
	    i_delta_prime_orientation_time = 1.0f / (dt);
		angular_velocity_set = IVP_TRUE;
	}else{
		angular_velocity_set = IVP_FALSE;
	}

    time_of_prime_orientation_0 = time0;
	this->ensure_in_simulation();
}

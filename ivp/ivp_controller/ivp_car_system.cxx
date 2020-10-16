// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *	Name:	     	IVP_Car_System
 *	Description:	
 ********************************************************************************/
// includes for API
#include <ivp_physics.hxx>

#ifndef WIN32
#	pragma implementation "ivp_car_system.hxx"
#endif

#include <ivp_material.hxx>
#include <ivp_actuator.hxx>

#include <ivp_template_constraint.hxx>
#include <ivp_constraint_car.hxx>
#include <ivp_car_system.hxx>


/////////////////////////////////


/********************************************************************************
 *	Name:	       		change_spring_dampening
 *	Description:		object is ensured to be in simulation
 ********************************************************************************/
void IVP_Car_System_Real_Wheels::change_spring_dampening(IVP_POS_WHEEL wheel_nr, IVP_FLOAT damp_factor)
{
  IVP_Actuator_Suspension *spring = this->car_spring[(int)wheel_nr];
  spring->set_damp(damp_factor);
}

/********************************************************************************
 *	Name:	       		change_spring_dampening
 *	Description:		object is ensured to be in simulation
 ********************************************************************************/
void IVP_Car_System_Real_Wheels::change_max_body_force(IVP_POS_WHEEL wheel_nr, IVP_FLOAT mforce)
{
  IVP_Actuator_Suspension *spring = this->car_spring[(int)wheel_nr];
  spring->set_max_body_force(mforce);
}

/********************************************************************************
 *	Name:	       		change_spring_dampening_compression
 *	Description:		object is ensured to be in simulation
 ********************************************************************************/
void IVP_Car_System_Real_Wheels::change_spring_dampening_compression(IVP_POS_WHEEL wheel_nr, IVP_FLOAT damp_factor)
{
  IVP_Actuator_Suspension *spring = this->car_spring[(int)wheel_nr];
  spring->set_spring_damp_compression(damp_factor);
}

/********************************************************************************
 *	Name:	       		change_spring_pre_tension
 *	Description:		object is ensured to be in simulation
 ********************************************************************************/
void IVP_Car_System_Real_Wheels::change_spring_pre_tension(IVP_POS_WHEEL wheel_nr, IVP_FLOAT pre_tension_len)
{
  IVP_Actuator_Suspension *spring = this->car_spring[(int)wheel_nr];
  // IVP_FLOAT length= spring->get_spring_length_zero_force();
  spring->set_len(500.0f - pre_tension_len);
}

/********************************************************************************
 *	Name:	       		change_spring_length
 *	Description:		object is ensured to be in simulation
 ********************************************************************************/
void IVP_Car_System_Real_Wheels::change_spring_length(IVP_POS_WHEEL wheel_nr, IVP_FLOAT spring_length)
{
  IVP_Actuator_Suspension *spring = this->car_spring[(int)wheel_nr];
  spring->set_len(spring_length);
}

/********************************************************************************
 *	Name:	       		change_stabilizer_constant
 *	Description:		object is ensured to be in simulation
 ********************************************************************************/
void IVP_Car_System_Real_Wheels::change_stabilizer_constant(IVP_POS_AXIS axis_nr, IVP_FLOAT stabi_constant)
{
  IVP_Actuator_Stabilizer *stabi = this->car_stabilizer[(int)axis_nr];
  stabi->set_stabi_constant(stabi_constant);
}

void IVP_Car_System_Real_Wheels::change_fast_turn_factor( IVP_FLOAT fast_turn_factor_ ){
	fast_turn_factor = fast_turn_factor_;
}

/********************************************************************************
 *	Name:	       		change_wheel_torque
 *	Description:		object is ensured to be in simulation
 ********************************************************************************/
void IVP_Car_System_Real_Wheels::change_wheel_torque(IVP_POS_WHEEL wheel_nr, IVP_FLOAT torque)
{
  IVP_Actuator_Torque *actuator = this->car_act_torque[(int)wheel_nr];
  actuator->set_torque(torque);
}

/********************************************************************************
 *	Name:	       		change_wheel_dampening
 *	Description:		object is ensured to be in simulation
 ********************************************************************************/
void IVP_Car_System_Real_Wheels::change_wheel_speed_dampening( IVP_POS_WHEEL wheel_nr, IVP_FLOAT dampening )
{
	IVP_Real_Object *pWheel = car_wheel[(int)wheel_nr];
	IVP_Core *pWheelCore = pWheel->get_core();
	pWheelCore->speed_damp_factor = dampening;
}

/********************************************************************************
 *	Name:	       		change_body_downforce
 *	Description:		object is ensured to be in simulation
 ********************************************************************************/
void IVP_Car_System_Real_Wheels::change_body_downforce(IVP_FLOAT force)
{
  IVP_Actuator_Force *act = this->car_act_down_force;
  act->set_force(force);
}

/********************************************************************************
 *	Name:	       		update_body_countertorque
 *	Description:		Updates countertorque resulting from the torques produced by the wheels
 ********************************************************************************/
void IVP_Car_System_Real_Wheels::update_body_countertorque()
{
    IVP_FLOAT counter_torque = 0.0f;
    
    // sum up all (negative) wheel torques
    for( int iWheel = 0; iWheel < n_wheels; ++iWheel )
	{
		IVP_Actuator_Torque *pTorqueAct = car_act_torque[( int )iWheel];
		counter_torque -= pTorqueAct->get_torque();
    }
    
    car_act_torque_body->set_torque( counter_torque * body_counter_torque_factor );
}

void IVP_Car_System_Real_Wheels::do_steering_wheel(IVP_POS_WHEEL wheel_pos, IVP_FLOAT s_angle)
{
    int wheel_nr = (int)wheel_pos;

    IVP_Constraint_Solver_Car *cs_car=this->car_constraint_solver;

    IVP_U_Matrix *target_mat = &cs_car->wheel_objects.element_at(wheel_nr)->target_position_bs;
    IVP_U_Point old_translation;
    old_translation.set(&target_mat->vv); // remember
    
    IVP_U_Point hp;
    hp.k[cs_car->x_idx] = 0.0f;
    hp.k[cs_car->y_idx] = s_angle;
    hp.k[cs_car->z_idx] = 0.0f;
    
    if(this->wheel_reversed_sign[wheel_nr]<0.0f){
	hp.k[cs_car->y_idx] += IVP_PI;
    }
    target_mat->init_rot_multiple(&hp, 1.0f);
    target_mat->vv.set(&old_translation);
}

void IVP_Car_System_Real_Wheels::do_steering(IVP_FLOAT s_angle)
{
    // tell constraint system new steering positions of wheels
    if (  steering_angle == s_angle) return;

    IVP_Constraint_Solver_Car *cs_car=this->car_constraint_solver;

	// start to spin the object 
	IVP_DOUBLE d_alpha = s_angle - steering_angle;

    IVP_FLOAT dx_front_wheels = 1.0f;
    IVP_FLOAT dz_axles = 1.0f;

	if( n_wheels >=4 ) {
		dx_front_wheels = this->get_orig_front_wheel_distance();
		dz_axles = this->get_orig_axles_distance();
	}

	IVP_DOUBLE angular_spin = d_alpha * get_body_speed(IVP_COORDINATE_INDEX(cs_car->z_idx)) / dz_axles;
	car_body->get_core()->rot_speed_change.k[IVP_COORDINATE_INDEX(cs_car->y_idx)] -= angular_spin * this->fast_turn_factor;

    this->steering_angle = s_angle;

    environment->get_controller_manager()->ensure_controller_in_simulation( cs_car );

    int wheels_per_axis = n_wheels / n_axis;
    for (int i=0; i < wheels_per_axis; i++){ /* turn front wheels */
	if( (i&1) == (s_angle>0.0f)){
    	    this->do_steering_wheel(IVP_POS_WHEEL(i), s_angle);
	    // ivp_message("%d regular angle: %g\n", i, s_angle);
	}else{
		IVP_FLOAT s_angle_2;
		if( n_wheels >=4 ) {
	        s_angle_2 = this->calc_ackerman_angle(steering_angle, dx_front_wheels, dz_axles);
		} else {
			s_angle_2 = steering_angle;
		}
    	    this->do_steering_wheel(IVP_POS_WHEEL(i), s_angle_2);
	    // ivp_message("%d ackerman angle: %g\n\n", i, s_angle_2);
	}
    }

    // hack for middle wheels
    if ( this->n_wheels > 4){ 
	this->do_steering_wheel(IVP_POS_WHEEL(4), s_angle  * .5f);
	this->do_steering_wheel(IVP_POS_WHEEL(5), s_angle  * .5f);
    }
}


/********************************************************************************
 *	Name:	       		change_spring_constant
 *	Description:		object is ensured to be in simulation
 ********************************************************************************/
void IVP_Car_System_Real_Wheels::change_spring_constant(IVP_POS_WHEEL wheel_nr, IVP_FLOAT constant)
{
  IVP_Actuator_Suspension *spring = this->car_spring[(int)wheel_nr];
  spring->set_constant(constant);
}

IVP_DOUBLE IVP_Car_System_Real_Wheels::get_body_speed(IVP_COORDINATE_INDEX idx_z)
{
    // return (IVP_FLOAT)car_body->get_geom_center_speed();
    IVP_U_Float_Point *vec_ws;
    vec_ws = &car_body->get_core()->speed;
    // works well as we do not use merged cores
    const IVP_U_Matrix *mat_ws = car_body->get_core()->get_m_world_f_core_PSI();
    IVP_U_Point orientation;
    mat_ws->get_col(idx_z, &orientation);

    return (IVP_FLOAT)orientation.dot_product(vec_ws);
}

IVP_DOUBLE IVP_Car_System_Real_Wheels::get_orig_front_wheel_distance()
{
    IVP_U_Matrix *m_left_wheel_bs = &this->car_constraint_solver->wheel_objects.element_at(0)->target_position_bs;
    IVP_U_Matrix *m_right_wheel_bs = &this->car_constraint_solver->wheel_objects.element_at(1)->target_position_bs;

    IVP_DOUBLE dist = m_left_wheel_bs->get_position()->k[this->car_constraint_solver->x_idx] -
 	              m_right_wheel_bs->get_position()->k[this->car_constraint_solver->x_idx];

    return -dist;
}

IVP_DOUBLE IVP_Car_System_Real_Wheels::get_orig_axles_distance()
{
    IVP_U_Matrix *m_front_wheel_bs =&this->car_constraint_solver->wheel_objects.element_at(0)->target_position_bs;
    IVP_U_Matrix *m_rear_wheel_bs = &this->car_constraint_solver->wheel_objects.element_at(2)->target_position_bs;

    IVP_DOUBLE dist = m_front_wheel_bs->get_position()->k[this->car_constraint_solver->z_idx] -
 	              m_rear_wheel_bs->get_position()->k[this->car_constraint_solver->z_idx];

    return IVP_Inline_Math::fabsd(dist);  // was fabs, which was a sml call
}

IVP_DOUBLE IVP_Car_System_Real_Wheels::get_wheel_angular_velocity(IVP_POS_WHEEL i){
    return this->car_act_torque[i]->rot_speed_out;
}


IVP_Car_System_Real_Wheels::IVP_Car_System_Real_Wheels( IVP_Environment *env, IVP_Template_Car_System *templ )
{
	// Builds up car system, using values from the template.
    n_wheels = templ->n_wheels;
    n_axis = templ->n_axis;
	
    booster_actuator[0] = 0;
	booster_actuator[1] = 0;
    booster_seconds_to_go = 0;
    booster_seconds_until_ready = 0;
    
	fast_turn_factor = templ->fast_turn_factor;

    environment = env;

	/////////////////////////////////////////////////////////////
	////////////  OBJECTS  //////////////////////////////////////
	/////////////////////////////////////////////////////////////
	this->car_body = templ->car_body;
	int i;
	for ( i = 0; i < n_wheels; i++ )
	{
		this->car_wheel[i] = templ->car_wheel[i];
		this->wheel_reversed_sign[i] = templ->wheel_reversed_sign[i];
	} 

	///////////////////////////////////////////////////////////////////
	////////    CONSTRAINT SYSTEM    //////////////////////////////////
	///////////////////////////////////////////////////////////////////
	this->car_constraint_solver = new IVP_Constraint_Solver_Car( templ->index_x, templ->index_y, templ->index_z, templ->is_left_handed );	
	// (IVP_INDEX_X, IVP_INDEX_Y, IVP_INDEX_Z, IVP_FALSE);

	IVP_U_Vector<IVP_Real_Object> wheels;
	IVP_U_Vector<IVP_U_Float_Point> hard_points;

	for ( i = 0; i< n_wheels; i++ )
	{
		wheels.add( car_wheel[i]);
		hard_points.add( &templ->wheel_pos_Bos[i] );
	}
	
	car_constraint_solver->init_constraint_system( environment, car_body, wheels, hard_points );
	IVP_Constraint_Solver_Car *cs_car = this->car_constraint_solver;

	/////////////////////////////////////////////////////////////
	//////// SPRINGS / SHOCK ABSORBERS //////////////////////////
	/////////////////////////////////////////////////////////////    
	int wheel_nr;
	IVP_Template_Anchor anchor_body_template[IVP_CAR_SYSTEM_MAX_WHEELS];
	IVP_Template_Anchor anchor_wheel_template[IVP_CAR_SYSTEM_MAX_WHEELS];	

	for ( wheel_nr = 0; wheel_nr <n_wheels; wheel_nr++ ) 
	{    
		IVP_Template_Suspension spring_template;

		// defaults
		spring_template.spring_values_are_relative=IVP_FALSE;
		spring_template.spring_constant =templ->spring_constant[wheel_nr];
		spring_template.spring_damp = templ->spring_dampening[wheel_nr];
		spring_template.spring_dampening_compression = templ->spring_dampening_compression[wheel_nr];
		spring_template.max_body_force = templ->max_body_force[wheel_nr];
		spring_template.rel_pos_damp = 0.00f;
      
	    // @@@OG be careful with car sizes around 500m/feet ;-) !
		spring_template.spring_len = 500.0f;
      
		// Create spring
		IVP_U_Float_Point hp( &templ->wheel_pos_Bos[wheel_nr] );
		hp.k[cs_car->y_idx] -= spring_template.spring_len;
		anchor_body_template[wheel_nr].set_anchor_position_os( car_body, &hp );
		
		// @@@OG os origin does not need to be the center! Better use core system's origin!
		anchor_wheel_template[wheel_nr].set_anchor_position_os( car_wheel[wheel_nr], 0.0f, 0.0f, 0.0f );
		
		spring_template.anchors[0] = &anchor_body_template[wheel_nr];
		spring_template.anchors[1] = &anchor_wheel_template[wheel_nr];
		
		spring_template.spring_len -= templ->spring_pre_tension[wheel_nr];
		this->car_spring[wheel_nr] = environment->create_suspension( &spring_template );

		///////////////////////////////////////////////////////////////////
		////////   ROTATION MOTORS (some wheel drive)   //////////////////////
		///////////////////////////////////////////////////////////////////
		IVP_Template_Anchor anchor_left_template;
		IVP_Template_Anchor anchor_right_template;
		IVP_Template_Torque torque_template;
			
		// defaults
		torque_template.max_rotation_speed = templ->wheel_max_rotation_speed[(wheel_nr&2)?1:0];
		
		// create torque actuator
		
		// @@@OG Should better use core system.
		hp.set_to_zero();
		hp.k[cs_car->x_idx] = templ->wheel_reversed_sign[wheel_nr];
		anchor_left_template.set_anchor_position_os( car_wheel[wheel_nr], &hp );
		
		hp.k[cs_car->x_idx] = -templ->wheel_reversed_sign[wheel_nr];
		anchor_right_template.set_anchor_position_os( car_wheel[wheel_nr], &hp );
		
		torque_template.anchors[0]  = &anchor_left_template;
		torque_template.anchors[1] = &anchor_right_template;
		torque_template.torque = 0.0f;							// This wasn't here, is this why we started with force?
		
		this->car_act_torque[wheel_nr] = environment->create_torque( &torque_template );
		this->fix_wheel_constraint[wheel_nr] = NULL;
	}	

	// Wheel rotation counterforce to body
	IVP_Template_Anchor anchor_left_template;
	IVP_Template_Anchor anchor_right_template;
	IVP_Template_Torque torque_template;
      
	// countertorque default
	torque_template.max_rotation_speed = ( IVP_FLOAT )IVP_PI * 100.0f;
	
	// create torque actuator
	IVP_U_Float_Point hp; 
	hp.set_to_zero();
	hp.k[cs_car->x_idx] = 1.0f;
	anchor_left_template.set_anchor_position_os( car_body, &hp );
	
	hp.k[cs_car->x_idx] = -1.0f;
	anchor_right_template.set_anchor_position_os( car_body, &hp );
	
	torque_template.anchors[0]  = &anchor_left_template;
	torque_template.anchors[1] = &anchor_right_template;
	torque_template.torque = 0.0f;
	
	this->car_act_torque_body = environment->create_torque( &torque_template );	
	this->body_counter_torque_factor = templ->body_counter_torque_factor;

	///////////////////////////////////////////////////////////////////
	////////   STABILIZERS    /////////////////////////////////////////
	///////////////////////////////////////////////////////////////////
	
	// equip vehicle axles with stabilizers
	if ( n_wheels != n_axis )
	{
		for( int i = 0; i < 2; i++ )
		{
			// i=0: front stabi, i=1: rear stabi
			IVP_Template_Stabilizer stabi_template;
			
			// set default
			stabi_template.stabi_constant = templ->stabilizer_constant[i]; // Newton/meter
			
			stabi_template.anchors[0] = &anchor_body_template[i*2];
			stabi_template.anchors[1] = &anchor_wheel_template[i*2];
			stabi_template.anchors[2] = &anchor_body_template[i*2+1];
			stabi_template.anchors[3] = &anchor_wheel_template[i*2+1];
			this->car_stabilizer[i] = environment->create_stabilizer( &stabi_template );
		}
	}

	///////////////////////////////////////////////////////////////////
	////////             DOWNFORCE               //////////////////////
	///////////////////////////////////////////////////////////////////
	IVP_Real_Object *static_object = environment->get_static_object(); 

    IVP_Template_Anchor anchor_center_template;
	IVP_Template_Anchor anchor_down_template;
	IVP_Template_Force force_template;
	
	// create force actuator

	// @@@OG Should better use core system.
//	IVP_U_Float_Point hp; 
	hp.set_to_zero();
	hp.k[cs_car->y_idx] += templ->body_down_force_vertical_offset; // offset for arbitrary rotation (e.g. when in air)
	anchor_center_template.set_anchor_position_cs(car_body, &hp);
      
	// Real far downwards so it appears vertical    
	hp.k[cs_car->y_idx] = 1e8f; 
    if ( environment->get_gravity()->k[cs_car->y_idx] > 0 )
	{
		hp.k[cs_car->y_idx] *= -1; // Check gravity
	}

	anchor_down_template.set_anchor_position_os( static_object, &hp );
	
	force_template.anchors[0] = &anchor_center_template;
	force_template.anchors[1] = &anchor_down_template;
	
	force_template.force = 0.0f;

	force_template.push_first_object = IVP_TRUE;
	force_template.push_second_object = IVP_FALSE;
	
	this->car_act_down_force = environment->create_force( &force_template );	

	///////////////////////////////////////////////////////////////////
	////////             EXTRA GRAVITY               //////////////////
	///////////////////////////////////////////////////////////////////
//	IVP_Template_Anchor anchor_center_template;
//	IVP_Template_Anchor anchor_down_template;
//	IVP_Template_Force force_template;
	IVP_U_Float_Point center; 
	center.set_to_zero();
      
	center.k[cs_car->y_idx] = templ->extra_gravity_height_offset;
	
	anchor_center_template.set_anchor_position_cs( car_body, &center );
	
//	IVP_U_Float_Point hp; 
	hp.set_to_zero();

	// Real far downwards so it appears vertical
	hp.k[cs_car->y_idx] = 1e8f; 
	if ( environment->get_gravity()->k[cs_car->y_idx] > 0 )
	{
		hp.k[cs_car->y_idx] *= -1; // Check direction of gravity
	}
	
	anchor_down_template.set_anchor_position_os( static_object, &hp );
	
	force_template.anchors[0] = &anchor_center_template;
	force_template.anchors[1] = &anchor_down_template;
	
	force_template.force = templ->extra_gravity_force_value;
	
	force_template.push_first_object = IVP_TRUE;
	force_template.push_second_object = IVP_FALSE;
	
	this->car_act_extra_gravity = environment->create_force( &force_template );	

	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////
	for ( int w = 0; w < n_wheels; w++ )
	{
		do_steering_wheel( IVP_POS_WHEEL( w ), 0.0f);
	}

	this ->steering_angle = -1.0f;		// make sure next call is not optimized
	this->do_steering( 0.0f );			// make sure next call gets through
	
	// register to environment as PSI listener
	//environment->add_listener_PSI(this);
	return;   
}


void IVP_Car_System_Real_Wheels::get_skid_info( IVP_Wheel_Skid_Info *array_of_skid_info_out){
  for ( int w = 0; w < n_wheels; w++){
	IVP_Wheel_Skid_Info &info = array_of_skid_info_out[w];
	IVP_Constraint_Car_Object *wheel = car_constraint_solver->wheel_objects.element_at(w);
		info.last_contact_position_ws = wheel->last_contact_position_ws;
		info.last_skid_value = wheel->last_skid_value;
		info.last_skid_time = wheel->last_skid_time;
	}
}

// stop wheel completely (e.g. handbrake )
void IVP_Car_System_Real_Wheels::fix_wheel( IVP_POS_WHEEL wheel_nr, IVP_BOOL stop_wheel )
{ 
    if ( !stop_wheel )
	{
		car_constraint_solver->wheel_objects.element_at( wheel_nr ) -> fix_wheel_constraint = NULL;
		P_DELETE( fix_wheel_constraint[wheel_nr] );
		return;
    }

    IVP_Real_Object *wheel = this->car_wheel[wheel_nr];

	// Already activated.
    if ( fix_wheel_constraint[wheel_nr] ) 
		return; 

    // Attach constraint to block wheel rotation
    IVP_Template_Constraint constraint;
    constraint.set_reference_object( wheel );   // set reference object
    constraint.set_attached_object( car_body );     // set attached object
    constraint.fix_rotation_axis( IVP_INDEX_X );
    constraint.free_rotation_axis( IVP_INDEX_Y );
    constraint.free_rotation_axis( IVP_INDEX_Z );
    constraint.free_translation_axis( IVP_INDEX_X );
    constraint.free_translation_axis( IVP_INDEX_Y );
    constraint.free_translation_axis( IVP_INDEX_Z );
	   	    
    // Magic Trick: slow down wheel to avoid effects that would normally
    // result from such an immediate blocking (inertias!)
    IVP_Core *wheel_core = wheel->get_core();
	wheel_core->rot_speed.add_multiple( &car_body->get_core()->rot_speed, 0.25f );
//    wheel_core->rot_speed = car_body->get_core()->rot_speed;
	    
    IVP_Environment *env = this->environment;
    this->fix_wheel_constraint[wheel_nr] = env->create_constraint( &constraint );
    car_constraint_solver->wheel_objects.element_at( wheel_nr ) ->fix_wheel_constraint = this->fix_wheel_constraint[wheel_nr];
}



/********************************************************************************
 *	Name:	      	activate_booster
 *	Description:	'Rocket' propulsion (in contrary to the standard wheel drive)
 ********************************************************************************/
void IVP_Car_System_Real_Wheels::activate_booster( IVP_FLOAT thrust, IVP_FLOAT duration, IVP_FLOAT delay ) 
{
	// Do we have an booster?
    if( booster_actuator[0] ) 
		return;

	// Booster not ready yet.
    if( booster_seconds_until_ready > 0.0f )
		return; 

    // Ignite booster by adding an actuator force to the car body 
    float gravity = environment->get_gravity()->real_length();
    set_booster_acceleration( thrust * gravity );

	// seconds
    booster_seconds_to_go = duration; 

	// time when next ignition is possible
    booster_seconds_until_ready = duration + delay; 
}


void IVP_Car_System_Real_Wheels::set_booster_acceleration( IVP_FLOAT acceleration ) 
{
    if ( acceleration )
	{
        IVP_Real_Object *pCarBody = car_body;
		
		if ( !booster_actuator[0] ) 
		{
			IVP_Template_Anchor anchorTempForward[2];
			IVP_Template_Anchor anchorTempUp[2];
			IVP_Template_Force forceTempForward;
			IVP_Template_Force forceTempUp;
			
			IVP_Constraint_Solver_Car *pCarConstraint = car_constraint_solver;
			IVP_Real_Object *pStaticObject = environment->get_static_object(); 

			IVP_U_Float_Point frontPoint, backPoint;
			frontPoint.set_to_zero();
			backPoint.set_to_zero();
			frontPoint.k[pCarConstraint->z_idx] = 1.0f;
			backPoint.k[pCarConstraint->z_idx] = -1.0f;
			anchorTempForward[0].set_anchor_position_cs( pCarBody, &frontPoint );
			anchorTempForward[1].set_anchor_position_cs( pCarBody, &backPoint );

			IVP_U_Float_Point centerPoint, downPoint;
			centerPoint.set_to_zero();
			downPoint.set_to_zero();
			downPoint.k[pCarConstraint->y_idx] = -1e8f; 
			anchorTempUp[0].set_anchor_position_cs( pCarBody, &centerPoint );
			anchorTempUp[1].set_anchor_position_os( pStaticObject, &downPoint );

			forceTempForward.anchors[0] = &anchorTempForward[0];
			forceTempForward.anchors[1] = &anchorTempForward[1];
			forceTempForward.active_float_force = NULL;
			forceTempForward.push_first_object = IVP_TRUE;
			forceTempForward.push_second_object = IVP_FALSE;
			forceTempForward.force = acceleration * pCarBody->get_core()->get_mass();

			forceTempUp.anchors[0] = &anchorTempUp[0];
			forceTempUp.anchors[1] = &anchorTempUp[1];
			forceTempUp.active_float_force = NULL;
			forceTempUp.push_first_object = IVP_TRUE;
			forceTempUp.push_second_object = IVP_FALSE;
			forceTempUp.force = -1.0f * pCarBody->get_core()->get_mass() * environment->get_gravity()->k[pCarConstraint->y_idx];

			booster_actuator[0] = environment->create_force( &forceTempForward );
			booster_actuator[1] = environment->create_force( &forceTempUp );
		}
		else
		{
			booster_actuator[0]->set_force( acceleration * pCarBody->get_core()->get_mass() );
		}
    }
	else
	{
		P_DELETE( booster_actuator[0] );
		P_DELETE( booster_actuator[1] );
    }
}

/********************************************************************************
 *	Name:	      	update_booster
 *	Description:	Check and update booster burn status
 ********************************************************************************/
void IVP_Car_System_Real_Wheels::update_booster(IVP_FLOAT delta_time) 
{

    if ( booster_seconds_until_ready > 0.0f )
	{
		booster_seconds_until_ready -= delta_time;
    }

    if ( booster_seconds_to_go > 0.0f )
	{
		this->booster_seconds_to_go -= delta_time;

		// booster shut down?
		if ( booster_seconds_to_go <= 0.0f )
		{
			set_booster_acceleration( 0.0f );
		}
    }
}

IVP_FLOAT IVP_Car_System_Real_Wheels::get_booster_delay() 
{
	return booster_seconds_until_ready; 
}

IVP_Car_System_Real_Wheels::~IVP_Car_System_Real_Wheels()
{
    //environment->remove_listener_PSI(this);
    P_DELETE(car_constraint_solver);
}

void IVP_Car_System_Real_Wheels::environment_will_be_deleted(IVP_Environment *){
    delete this;
}

IVP_FLOAT IVP_Car_System::calc_ackerman_angle(IVP_FLOAT alpha, IVP_FLOAT dx, IVP_FLOAT dz){

    // We got the steering angle for that front wheel towards
    // the curve center (alpha). Now lets compute the Ackerman angle
    // for the outer front wheel.

    // dx means distance between the two front wheels,
    // dz means "Radstand" (distance between front and rear 'axles').

    IVP_FLOAT a = IVP_Inline_Math::fabsd(alpha); // was fabs, which was a sml call
    
    if(a<0.001f) return alpha; // numerical reasons

    IVP_DOUBLE tan_alpha = tan(a);
    IVP_DOUBLE h = (dz*tan_alpha)/(dx*tan_alpha+dz);
    IVP_DOUBLE beta = atan(h);
    IVP_DOUBLE signum = (alpha<0.0f)?-1.0f:1.0f;
    return (IVP_FLOAT)(beta * signum);
}

IVP_Car_System::~IVP_Car_System(){
    ;
}

IVP_Car_System::IVP_Car_System(){
    ;
}

//-----------------------------------------------------------------------------
// Purpose: Debug data for use in vphysics and the engine to visualize car data.
//-----------------------------------------------------------------------------
void IVP_Car_System_Real_Wheels::SetCarSystemDebugData( const IVP_CarSystemDebugData_t &carSystemDebugData )
{
	return;
}

//-----------------------------------------------------------------------------
// Purpose: Debug data for use in vphysics and the engine to visualize car data.
//-----------------------------------------------------------------------------
void IVP_Car_System_Real_Wheels::GetCarSystemDebugData( IVP_CarSystemDebugData_t &carSystemDebugData )
{
	return;
}

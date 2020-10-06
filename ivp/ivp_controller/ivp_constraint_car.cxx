// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <ivp_solver_core_reaction.hxx>
#include <ivp_great_matrix.hxx>
#include <math.h>
#include <ivu_vector.hxx>
#include <ivp_constraint_local.hxx>
#include <ivp_constraint_car.hxx>
#include <ivp_template_constraint.hxx>

//=============================================================================
//
// Car constraint object functions.
//

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
IVP_Constraint_Car_Object::IVP_Constraint_Car_Object( IVP_Constraint_Solver_Car *solver_car_, 
													  IVP_Real_Object *i_real_obj_app,
													  IVP_Real_Object *i_real_obj_body, 
													  IVP_U_Float_Point *target_Bos_override )
{
    this->real_object = i_real_obj_app;
    this->solver_car = solver_car_;
    this->fix_wheel_constraint = NULL;

    i_real_obj_app->get_core()->car_wheel = this;

	last_skid_value = 0.0f;
	last_contact_position_ws.set_to_zero();
	last_skid_time = 0.0f;

    // missing: backward ref from obj to this, to manage obj deletion... @@@OG

    /*** remember as default: target_position and rotation in body system ***/
    if (i_real_obj_body )
	{
		//// treat appendix
		IVP_Core *core_A = i_real_obj_app->get_core();
		IVP_Core *core_B = i_real_obj_body->get_core();
		
		// translation and rotation
		const IVP_U_Matrix *m_world_f_B = core_B->get_m_world_f_core_PSI();
		const IVP_U_Matrix *m_world_f_A = core_A->get_m_world_f_core_PSI();

		if ( target_Bos_override )
		{
			this->target_position_bs.init();
			IVP_U_Matrix m_core_f_object;
			i_real_obj_body->calc_m_core_f_object(&m_core_f_object);
			m_core_f_object.vmult4(target_Bos_override, &this->target_position_bs.vv);
		}
		else
		{
			m_world_f_B->mimult4(m_world_f_A, &this->target_position_bs);
		}
    }
	else
	{
		//// body itself: gets unit pos and rot
		this->target_position_bs.init();
    }
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
IVP_Constraint_Car_Object::~IVP_Constraint_Car_Object()
{
    real_object->get_core()->car_wheel = NULL;
}

//=============================================================================
//
// Car constraint solver functions.
//

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
IVP_Constraint_Solver_Car::IVP_Constraint_Solver_Car( IVP_COORDINATE_INDEX right, 
													  IVP_COORDINATE_INDEX up, 
													  IVP_COORDINATE_INDEX forward, 
													  IVP_BOOL is_left_hand )
{
    // P_MEM_CLEAR(this); NO !!!
	
    this->x_idx = right;
    this->y_idx = up;
    this->z_idx = forward;
    this->angle_sign = ( is_left_hand ) ? -1.0f : 1.0f;
	
    this->max_delta_speed = 80.0f * 3.0f;				// greater ??
    this->local_translation_in_use = IVP_FALSE;

    // constraint_is_disabled is inited in Builder.
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
IVP_Constraint_Solver_Car::~IVP_Constraint_Solver_Car()
{
	//delete silently
    IVP_Controller_Manager::remove_controller_from_environment( this, IVP_TRUE ); 

    P_DELETE( body_object );
    for ( int app_nr = 0; app_nr < wheel_objects.len(); app_nr++ )
	{
		delete wheel_objects.element_at( app_nr );
    }
    
    for ( int k = 0; k < wheel_objects.len(); k++ )
	{
		P_DELETE( this->c_local_ballsocket[k] );
    }

    wheel_objects.clear();

    P_FREE( this->co_matrix.matrix_values );
    P_FREE( this->co_matrix.result_vector );
    P_FREE( this->co_matrix.desired_vector );
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
IVP_RETURN_TYPE IVP_Constraint_Solver_Car::init_constraint_system( IVP_Environment *env, IVP_Real_Object *body,
																   IVP_U_Vector<IVP_Real_Object> &wheels,	
																   IVP_U_Vector<IVP_U_Float_Point> &p_Bos )
{ 
    environment = env;

    body_object = new IVP_Constraint_Car_Object( this, body, 0, NULL );
    cores_of_constraint_system.add( body->get_core() );

    for ( int i = 0; i < wheels.len(); i++ )
	{
		wheel_objects.add( new IVP_Constraint_Car_Object( this, wheels.element_at(i), body, p_Bos.element_at( i ) ) );
		cores_of_constraint_system.add( wheels.element_at(i)->get_core() );
		c_local_ballsocket[i] = NULL;
    }

    env->get_controller_manager()->announce_controller_to_environment( this );
    
    IVP_Constraint_Solver_Car_Builder *builder;
    builder = new IVP_Constraint_Solver_Car_Builder( this );

    builder->disable_constraint( y_idx );	// free Y trans
    builder->disable_constraint( x_idx+3 ); // free X rot
    builder->disable_constraint( y_idx+3 ); // free y rot
    builder->disable_constraint( z_idx+3 ); // free z rot

    IVP_RETURN_TYPE res =  builder->calc_constraint_matrix();

    P_DELETE( builder );

    return res;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void IVP_Constraint_Solver_Car::do_simulation_controller_rotation( IVP_Event_Sim *es, 
																   IVP_Core *core_B, 
																   const IVP_U_Matrix *m_world_f_B )
{
    // For all appendices (wheels).
	for (int app_cnt = 0; app_cnt < wheel_objects.len(); app_cnt++ )
	{
		IVP_Constraint_Car_Object *app = wheel_objects.element_at( app_cnt );
		IVP_Core *core_A = app->get_core();
		const IVP_U_Matrix *m_world_f_A = core_A->get_m_world_f_core_PSI();
	
		// calc y and z angles from vector
		if (1) 
		{ // new version try
			// calc target vector in A
			IVP_U_Point target_vec_as;
			IVP_U_Point tv_bs, tv_ws; 
			app->target_position_bs.get_col( IVP_COORDINATE_INDEX( x_idx ), &tv_bs );
			m_world_f_B->vmult3( &tv_bs, &tv_ws );
			m_world_f_A->vimult3( &tv_ws, & target_vec_as );
	
			IVP_DOUBLE cur_angles_B[2];
			{
				// turn back target vec depending on rot speed
				// thus improving behavior with high rotation speed
#ifdef IVP_FAST_WHEELS_ENABLED
				IVP_DOUBLE ny = target_vec_as.k[z_idx];
				IVP_DOUBLE nx = target_vec_as.k[y_idx];
#else
				IVP_DOUBLE alpha = core_A->rot_speed.k[x_idx] * es->delta_time * 0.45f;				// no angle sign needed
				IVP_DOUBLE sina = angle_sign * IVP_Inline_Math::approx5_sin( alpha );
				IVP_DOUBLE cosa = IVP_Inline_Math::approx5_cos( alpha );
				IVP_DOUBLE ny = target_vec_as.k[z_idx] * cosa - target_vec_as.k[y_idx] * sina;
				IVP_DOUBLE nx = target_vec_as.k[y_idx] * cosa + target_vec_as.k[z_idx] * sina;
#endif
	  
				cur_angles_B[0] = angle_sign * -IVP_Inline_Math::atan2d( ny, target_vec_as.k[x_idx] );
				cur_angles_B[1] = angle_sign * IVP_Inline_Math::atan2d( nx, target_vec_as.k[x_idx] );
			}

			IVP_U_Float_Point y_rot_axis_world2; 
			m_world_f_A->get_col( IVP_COORDINATE_INDEX( y_idx ), &y_rot_axis_world2 );
			IVP_U_Float_Point z_rot_axis_world2; 
			m_world_f_A->get_col( IVP_COORDINATE_INDEX( z_idx ), &z_rot_axis_world2 );
			
			IVP_Solver_Core_Reaction tcb;
			tcb.init_reaction_solver_rotation_ws( core_B, core_A, &y_rot_axis_world2, &z_rot_axis_world2, NULL );
			
			//// invert matrix
			IVP_DOUBLE ia1, ia2, ib1, ib2;
			IVP_RETURN_TYPE r;
			IVP_U_Matrix3 &tpm = tcb.m_velocity_ds_f_impulse_ds;
			
			const IVP_DOUBLE tpm00 = tpm.get_elem( 0, 0 );
			const IVP_DOUBLE tpm01 = tpm.get_elem( 0, 1 );
			const IVP_DOUBLE tpm10 = tpm.get_elem( 1 ,0 );
			const IVP_DOUBLE tpm11 = tpm.get_elem( 1, 1 );
	  
			r = IVP_Inline_Math::invert_2x2_matrix( tpm00, tpm01, tpm01 /*@@CB FIXME?*/,tpm11, &ia1, &ib1, &ia2, &ib2 );
			if ( r == IVP_FAULT)
			{
				ivp_message("do_constraint_system: Couldn't invert rot matrix!\n");
				return;
			}

			IVP_DOUBLE a = -es->i_delta_time * cur_angles_B[0] - tcb.delta_velocity_ds.k[0];
			IVP_DOUBLE b = -es->i_delta_time * cur_angles_B[1] - tcb.delta_velocity_ds.k[1];
			
			//// mult with inv matrix
			IVP_U_Float_Point rot_impulse_ds;
			rot_impulse_ds.k[0] = a * ia1 + b * ib1;
			rot_impulse_ds.k[1] = a * ia2 + b * ib2;
	  
			tcb.exert_angular_impulse_dim2(core_B, core_A, rot_impulse_ds);

#if 0
			// 4-wheel - debugging!
			if ( app_cnt >= 0 && app_cnt < 4 )
			{
				m_wheelRotationTorque[app_cnt][0] = rot_impulse_ds.k[0];
				m_wheelRotationTorque[app_cnt][1] = rot_impulse_ds.k[1];
				m_wheelRotationTorque[app_cnt][2] = rot_impulse_ds.k[2];
			}
#endif
		}	
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void IVP_Constraint_Solver_Car::do_simulation_controller( IVP_Event_Sim *es, 
														  IVP_U_Vector<IVP_Core> * /*core_list*/ ) 
{
    IVP_DOUBLE inv_dtime = es->i_delta_time;    

    // Pushes body and appendices so that total impulse is the same
    // and all parts will fulfil the specified constraints in next PSI
    IVP_Constraint_Car_Object *body = body_object;
    IVP_Core *core_B = body->get_core();
    const IVP_U_Matrix *m_world_f_B = core_B->get_m_world_f_core_PSI();

    /*********** ROTATION ************/
	do_simulation_controller_rotation( es, core_B, m_world_f_B );
      
    /*********** TRANSLATION ************/
//	move the below stuff into - do_simulation_controller_translation( );

    IVP_DOUBLE *input_vec_ptr = this->co_matrix.desired_vector;
    IVP_ASSERT( this->co_matrix.columns == wheel_objects.len() * 2 );

    int init_local_translation = 0;					// flag
    int invalid_count= 0;							// for plan B removal
    IVP_FLOAT delta_time = es->delta_time;
    
    // for all appendices
    for(int app_cnt=0; app_cnt < wheel_objects.len(); app_cnt++){
	IVP_Constraint_Car_Object *app = wheel_objects.element_at(app_cnt);
	IVP_Core *core_A = app->get_core();
	// core_A->ensure_core_to_be_in_simulation(); // already done for rotation
	const IVP_U_Matrix *m_world_f_A = core_A->get_m_world_f_core_PSI();
	
	/**** translation divergence (after delta time) ***/
	IVP_U_Point delta_speed_vec;
	{
	    // calc current positions in B
	    IVP_U_Float_Point cur_pos_A_in_B;
	    IVP_U_Float_Point cur_pos_B_in_B(app->target_position_bs.get_position());
	    m_world_f_B->vimult4(&m_world_f_A->vv, &cur_pos_A_in_B); // takes center of app

	    // calc speed and rotspeed in B
	    IVP_U_Float_Point surspeed_A_in_B;
	    IVP_U_Float_Point surspeed_B_in_world;
	    IVP_U_Float_Point surspeed_B_in_B;

	    m_world_f_B->vimult3(&core_A->speed, &surspeed_A_in_B); // takes app center speed
	
	    core_B->get_surface_speed(&cur_pos_B_in_B, &surspeed_B_in_world);
	    m_world_f_B->vimult3(&surspeed_B_in_world, &surspeed_B_in_B);

	    // calc positions at next PSI in B
	    IVP_U_Float_Point next_pos_A_in_B;
	    IVP_U_Float_Point next_pos_B_in_B;

	    next_pos_A_in_B.add_multiple(&cur_pos_A_in_B, &surspeed_A_in_B, delta_time);
	    next_pos_B_in_B.add_multiple(&cur_pos_B_in_B, &surspeed_B_in_B, delta_time);
	
	    // calc delta speed vec that has to be realized
	    delta_speed_vec.subtract(&next_pos_A_in_B, &next_pos_B_in_B); 
	    delta_speed_vec.mult(inv_dtime * 1.0f);

	    // difference too high?
	    {
		IVP_DOUBLE quad_speed = delta_speed_vec.quad_length();
		if( quad_speed > (this->max_delta_speed*this->max_delta_speed)){
		  invalid_count ++;
		  // IVP_DOUBLE factor = this->max_delta_speed / delta_speed_vec.fast_real_length();
		  // delta_speed_vec.mult(factor);
		    psis_left_for_plan_B = 10;
		  if(this->local_translation_in_use == IVP_FALSE){
		    // switch to ball and socket local solution
		    init_local_translation = 1;
		    break;
		  }
		}
	    }
	}

	/**** add to input vector for matrix mult ***/
	// supercar special solution without if's!
	IVP_ASSERT(this->constraint_is_disabled[y_idx] == IVP_TRUE);
	IVP_ASSERT(this->constraint_is_disabled[x_idx+3] == IVP_TRUE);
	IVP_ASSERT(this->constraint_is_disabled[y_idx+3] == IVP_TRUE);
	IVP_ASSERT(this->constraint_is_disabled[z_idx+3] == IVP_TRUE);
	(*input_vec_ptr++) = delta_speed_vec.k[x_idx];
	(*input_vec_ptr++) = delta_speed_vec.k[z_idx];	
    }

    if(invalid_count == 0 && (this->local_translation_in_use==IVP_TRUE)){
      // switch back to regular translation
	psis_left_for_plan_B--;
	if (psis_left_for_plan_B <0){
	      int app_nr;
	      for(app_nr=0; app_nr< wheel_objects.len(); app_nr++){
		P_DELETE(this->c_local_ballsocket[app_nr]);
		this->c_local_ballsocket[app_nr] = NULL;
	      }
	      this->local_translation_in_use = IVP_FALSE;
	      IVP_IF(1){
		printf("plan B deactivated.\n");
	      }
	}
    }
    
    if(init_local_translation==IVP_TRUE){
	IVP_IF(1){
	    printf("plan B activated.\n");
	}


#if 1
        // appendices are too far away -> start plan B (solve problem with local constraints)
        init_local_translation = 0;
	int app_nr;
	for(app_nr=0; app_nr < wheel_objects.len(); app_nr++){
	    IVP_Constraint_Car_Object *app = wheel_objects.element_at(app_nr);
	    
	    IVP_U_Matrix m_core_f_object;
	    body->real_object->calc_m_core_f_object(&m_core_f_object);
	    const IVP_U_Matrix *pm_ws_f_Bcs;
	    IVP_U_Matrix m_ws_f_Aos;
	    pm_ws_f_Bcs = body->real_object->get_core()->get_m_world_f_core_PSI();
	    app->real_object->get_m_world_f_object_AT(&m_ws_f_Aos);
	    
	    // fill in template
	    IVP_U_Point anchorB_Bos, anchorB_ws, anchorB_Aos;
	    anchorB_Bos.set(app->target_position_bs.get_position());
	    pm_ws_f_Bcs->vmult4(&anchorB_Bos, &anchorB_ws);
	    m_ws_f_Aos.vimult4(&anchorB_ws, &anchorB_Aos);
	    IVP_U_Point anchorA_Aos; anchorA_Aos.set_to_zero();
	    
	    IVP_Template_Constraint templ;
	    templ.set_ballsocket_tense_Ros(app->real_object, &anchorA_Aos, body->real_object, &anchorB_Aos);
	    
	    IVP_Environment *env = core_B->get_environment();
	    this->c_local_ballsocket[app_nr] = env->create_constraint(&templ);
	}
	this->local_translation_in_use = IVP_TRUE;
#endif
    }


#if 0
    {
	int k;
	printf("Inputvector:\n");
	for(k=0; k<co_matrix.columns; k++){
	    if(k%4 == 0)printf("(%d)", k);
	    printf("%2.2f  ", co_matrix.desired_vector[k]);
	}
	printf("\n");
    }
#endif
    
    // calc pushes that have to be performed
    co_matrix.mult();

#if 0
    {
	int k;
	printf("Outputvector:\n");
	for(k=0; k<co_matrix.columns; k++){
	    if(k%4 == 0)printf("(%d)", k);
	    printf("%2.2f  ", co_matrix.result_vector[k]);
	}
	printf("\n\n");
    }
#endif    

    if(this->local_translation_in_use == IVP_FALSE){
	/*** now push objects ***/
	IVP_DOUBLE *res_vec_ptr = this->co_matrix.result_vector;
	int a_cnt;
	for( a_cnt=0; a_cnt < wheel_objects.len(); a_cnt++){
	    IVP_Constraint_Car_Object *app = wheel_objects.element_at(a_cnt);
	    IVP_Core *core_A = app->get_core();
	    const IVP_U_Matrix *m_world_f_A = core_A->get_m_world_f_core_PSI();

	    //// translation push 
	    IVP_U_Float_Point t_impulse_ws, t_impulse_bs;
	    t_impulse_bs.k[x_idx] = res_vec_ptr[0];
	    t_impulse_bs.k[y_idx] = 0.0f;
	    t_impulse_bs.k[z_idx] = res_vec_ptr[1];

	    m_world_f_B->vmult3(&t_impulse_bs, &t_impulse_ws);
	    res_vec_ptr += 2;

	    IVP_U_Point target_position_ws;
	    m_world_f_B->vmult4(&app->target_position_bs.vv, &target_position_ws);

	    // body	
	    core_B->push_core_ws(&target_position_ws, &t_impulse_ws);

	    // appendix
	    t_impulse_ws.mult(-1.0f);
	    core_A->push_core_ws(&m_world_f_A->vv, &t_impulse_ws); // @@@OG use center push
	}
	/*** translation done ***/
    }
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void IVP_Constraint_Solver_Car::core_is_going_to_be_deleted_event(IVP_Core *) 
{
    P_DELETE_THIS( this );
}

/////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
IVP_Constraint_Solver_Car_Builder::IVP_Constraint_Solver_Car_Builder( IVP_Constraint_Solver_Car *i_car_solver )
{
    P_MEM_CLEAR(this);
    
    this->car_solver = i_car_solver;
    this->n_appends = this->car_solver->get_num_of_appending_terminals();

	// Default: trans (x, y, z) and rot(x, y, z )
    this->n_constraints = 6;
    for ( int i = 0; i < 6; i++ )
	{
		this->car_solver->constraint_is_disabled[i] = IVP_FALSE;
    }
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void IVP_Constraint_Solver_Car_Builder::disable_constraint(int idx)
{
    IVP_ASSERT( ( idx >= 0 ) && ( idx < 6 ) );

    if( this->car_solver->constraint_is_disabled[idx] == IVP_FALSE )
	{
		this->car_solver->constraint_is_disabled[idx] = IVP_TRUE;
		this->n_constraints--;
    }
}

//-----------------------------------------------------------------------------
// Purpose: Performs bilateral test pushes in all 3 coord axis and all 3 rot axis
//          using coord system of constraint_obj 'co_obj[0]'.  The resulting delta 
//          speed changes are written into this->tmp_matrix.
//-----------------------------------------------------------------------------
void IVP_Constraint_Solver_Car_Builder::calc_pushing_behavior( int A_obj_idx, int push_vec_idx )
{
	// Verify incoming data.
    IVP_ASSERT( A_obj_idx >= 0 && A_obj_idx < n_appends );
    IVP_ASSERT( push_vec_idx >= 0 && push_vec_idx < 6 );
    IVP_ASSERT( this->car_solver->constraint_is_disabled[push_vec_idx] == IVP_FALSE );
	
    IVP_Constraint_Car_Object *A_obj = car_solver->wheel_objects.element_at( A_obj_idx );
    IVP_Core *core_A = A_obj->get_core();

    IVP_Constraint_Car_Object *B_obj = car_solver->body_object;    
    IVP_Core *core_B = B_obj->get_core();
    const IVP_U_Matrix *m_world_f_B = core_B->get_m_world_f_core_PSI();

	IVP_U_Matrix h_matrix = (*m_world_f_B);
    IVP_U_Matrix *m_world_f_A = &h_matrix;
	m_world_f_B->mmult4( &A_obj->target_position_bs, m_world_f_A ); // m_world_f_A refers to ideal position of A

    IVP_U_Matrix m_B_from_A;
    m_world_f_B->mimult4( m_world_f_A, &m_B_from_A );
    
    IVP_U_Float_Point impulse_A, impulse_B, impulse_world;
    impulse_B.set_to_zero();
    impulse_B.k[push_vec_idx%3] = 1.0f;
    m_world_f_B->vmult3( &impulse_B, &impulse_world );
    m_world_f_A->vimult3( &impulse_world, &impulse_A );    

    IVP_U_Float_Point inv_impulse_A, inv_impulse_B, inv_impulse_world;
    inv_impulse_B.set_negative( &impulse_B );
    m_world_f_B->vmult3( &inv_impulse_B, &inv_impulse_world );
    m_world_f_A->vimult3( &inv_impulse_world, &inv_impulse_A );    

    IVP_U_Float_Point A_push_pos_in_A, B_push_pos_in_B;
    A_push_pos_in_A.set_to_zero();								// push in appendix center
    m_B_from_A.vmult4( &A_push_pos_in_A, &B_push_pos_in_B );	// means: objects are regarded as placed at the desired positions
    
    IVP_DOUBLE rot_impulse_strength = 1.0f;
    IVP_DOUBLE inv_rot_impulse_strength = -rot_impulse_strength;

    IVP_U_Float_Point speed_change_A_in_B, rot_change_A_in_B;
    IVP_U_Float_Point sur_speed_B_in_B, rot_change_B_in_B;
    IVP_U_Float_Point sur_speed_B_world;
    IVP_U_Float_Point speed_change_B_world;

	// Bilateral Translation Test Pushes of the specified impulse vec in core system of B
    if ( push_vec_idx <= 2 )
	{
		/** Push A **/
		IVP_U_Float_Point speed_change_A_world, rot_change_A_in_A;
		
		core_A->test_push_core( &A_push_pos_in_A, &impulse_A, &impulse_world, &speed_change_A_world, &rot_change_A_in_A );
		m_world_f_B->vimult3( &speed_change_A_world, &speed_change_A_in_B );
		// core speed suffices, because push was at center of wheel	
		// @@@ test: rot_change_A_in_A should be nearly zero, now
//		rot_change_A_in_A.print("rot_change_A_in_A should be zero:");

		rot_change_A_in_B.set_to_zero();			// push in center should not result in rotation
		
		/**** Push B (contrary) ****/	
		core_B->test_push_core( &B_push_pos_in_B, &inv_impulse_B, &inv_impulse_world, &speed_change_B_world, &rot_change_B_in_B );
		
		// calc speed at push pos
		core_B->get_surface_speed_on_test( &B_push_pos_in_B, &speed_change_B_world, &rot_change_B_in_B, &sur_speed_B_world );
		m_world_f_B->vimult3( &sur_speed_B_world, &sur_speed_B_in_B );
    }

	// Bilateral Rotation Test Pushes of the specified impulse vec in core system of B
    if ( push_vec_idx >= 3)
	{
		/** Rot push A **/
		IVP_U_Float_Point rot_change_A_in_A;
	
		core_A->test_rot_push_core_multiple_cs( &impulse_A, rot_impulse_strength, &rot_change_A_in_A );
		m_B_from_A.vmult3( &rot_change_A_in_A, &rot_change_A_in_B );
		
		speed_change_A_in_B.set_to_zero();			// rot of a in center should not result in translation!
		
		/**** Rot push B (contrary) ****/	
		core_B->test_rot_push_core_multiple_cs( &impulse_B, inv_rot_impulse_strength, &rot_change_B_in_B );

		// calc resulting sur speed
		speed_change_B_world.set_to_zero(); // no core trans
		core_B->get_surface_speed_on_test( &B_push_pos_in_B, &speed_change_B_world, &rot_change_B_in_B, &sur_speed_B_world );
		m_world_f_B->vimult3( &sur_speed_B_world, &sur_speed_B_in_B );	
    }

    // Calc push_vec_idx for this (reduced) matrix
    int pv_mat_idx = 0;
	for ( int dim = 0; dim < push_vec_idx; dim++ )
	{
		if ( this->car_solver->constraint_is_disabled[dim] == IVP_FALSE )
		{
			pv_mat_idx++;
		}
	}
    IVP_ASSERT( pv_mat_idx >= 0 ); // all constraints disabled !?
    
    /// Calc and insert deltas in great tmp matrix
    IVP_Great_Matrix_Many_Zero *g_mat = &this->tmp_matrix;
    int gm_col = n_constraints * A_obj_idx + pv_mat_idx; // col to fill
    int gm_row = 0;

    for ( int fill_app_idx = 0; fill_app_idx < car_solver->wheel_objects.len(); fill_app_idx++ )
	{
		IVP_U_Float_Point delta_speed_change, delta_rot_change;
		if ( fill_app_idx == A_obj_idx )
		{
			// calc direct changes with already tested appendix object
			delta_speed_change.subtract(&speed_change_A_in_B, &sur_speed_B_in_B);
			delta_rot_change.subtract(&rot_change_A_in_B, &rot_change_B_in_B);
		}
		else
		{
			// calc indirect changes (through 'motion' of B object)
			const IVP_U_Point *app_center_world = &this->car_solver->wheel_objects.element_at( fill_app_idx )->get_core()->get_m_world_f_core_PSI()->vv;
			IVP_U_Float_Point app_center_B, B_at_app_speed_world;
			m_world_f_B->vimult4( app_center_world, &app_center_B );
			core_B->get_surface_speed_on_test( &app_center_B, &speed_change_B_world, &rot_change_B_in_B, &B_at_app_speed_world );
			m_world_f_B->vimult3( &B_at_app_speed_world, &delta_speed_change );
			delta_speed_change.mult(-1);
			delta_rot_change.set_negative( &rot_change_B_in_B );
		}

		// fill in
		if ( this->car_solver->constraint_is_disabled[0] == IVP_FALSE )
		{
			g_mat->set_value(delta_speed_change.k[0], gm_col, gm_row++);
		}

		if ( this->car_solver->constraint_is_disabled[1] == IVP_FALSE )	    
		{
			g_mat->set_value(delta_speed_change.k[1], gm_col, gm_row++);
		}

		if ( this->car_solver->constraint_is_disabled[2] == IVP_FALSE )
		{
			g_mat->set_value(delta_speed_change.k[2], gm_col, gm_row++);
		}

		if ( this->car_solver->constraint_is_disabled[3] == IVP_FALSE )	    
		{
			g_mat->set_value(delta_rot_change.k[0], gm_col, gm_row++);
		}

		if ( this->car_solver->constraint_is_disabled[4] == IVP_FALSE )
		{
			g_mat->set_value(delta_rot_change.k[1], gm_col, gm_row++);
		}

		if ( this->car_solver->constraint_is_disabled[5] == IVP_FALSE )
		{
			g_mat->set_value(delta_rot_change.k[2], gm_col, gm_row++);
		}
    }
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
IVP_RETURN_TYPE IVP_Constraint_Solver_Car_Builder::calc_constraint_matrix()
{
    // Malloc matrix value vec (temp).
    int gm_size = n_constraints * n_appends;
    this->tmp_matrix.columns = gm_size;
    this->tmp_matrix.MATRIX_EPS = P_DOUBLE_EPS;
    this->tmp_matrix.matrix_values = ( IVP_DOUBLE* )p_malloc( gm_size * gm_size * sizeof( IVP_DOUBLE ) );
    this->tmp_matrix.desired_vector = ( IVP_DOUBLE* )p_malloc( gm_size * sizeof(IVP_DOUBLE ) );
    this->tmp_matrix.result_vector = ( IVP_DOUBLE* )p_malloc( gm_size * sizeof( IVP_DOUBLE ) );
    
    // Build up pushing/reaction matrix.
    for( int app_idx = 0; app_idx < n_appends; app_idx++ )
	{
		for( int vec_idx = 0; vec_idx < 6; vec_idx++ )
		{
			if( this->car_solver->constraint_is_disabled[vec_idx] == IVP_TRUE ) 
				continue;

			calc_pushing_behavior( app_idx, vec_idx );
		}
    }

//  printf( "Constraint solver: calc_pushing_behavior done.\n" );
//  this->tmp_matrix.print( "Pushing behavior matrix.\n" );
    
    // Invert matrix into Solver.
    IVP_DOUBLE *double_vec = ( IVP_DOUBLE* )p_malloc( gm_size * gm_size * sizeof( IVP_DOUBLE ) );
    this->car_solver->co_matrix.matrix_values = double_vec;
    this->car_solver->co_matrix.columns = gm_size;

    IVP_RETURN_TYPE ret_val = this->tmp_matrix.invert( &this->car_solver->co_matrix );
//  printf("Constraint solver: matrix inversion done.\n");

	// Free the temp data.
    P_FREE( this->tmp_matrix.matrix_values );
    P_FREE( this->tmp_matrix.result_vector );
    P_FREE( this->tmp_matrix.desired_vector );
    
    // Some mallocs, so that they don't have to be done in simulation...
    int vec_size = gm_size * sizeof( IVP_DOUBLE );
    this->car_solver->co_matrix.desired_vector = ( IVP_DOUBLE* )p_malloc( vec_size );
    this->car_solver->co_matrix.result_vector = ( IVP_DOUBLE* )p_malloc( vec_size );
    this->car_solver->co_matrix.MATRIX_EPS = P_DOUBLE_EPS;

//  this->car_solver->co_matrix.print( "Pushing behavior matrix.\n" );

    return ret_val;
}


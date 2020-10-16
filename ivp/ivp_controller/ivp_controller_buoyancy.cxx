// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#if defined(LINUX) || defined(SUN) || (defined(__MWERKS__) && defined(__POWERPC__))
#   include <alloca.h>
#endif

#ifndef WIN32
#	pragma implementation "ivp_controller_buoyancy.hxx"
#endif

#include <ivp_cache_object.hxx>
#include <ivp_controller_buoyancy.hxx>
#include <ivp_liquid_surface_descript.hxx>
#include <ivp_buoyancy_solver.hxx>


#define DAMPENING_WITH_MOVEVECTOR               1
#define DAMPENING_WITH_PARTICLE_ACCELERATION    1

//#define WITH_DEBUG_OUTPUT                       1


/**************************************************************************
 * Name:        calculate_future_extrapolation(...)
 * Description: extrapolates a new input vector for the multidimensional
 *              interpolator from the current input and the previous input
 **************************************************************************/
IVP_RETURN_TYPE IVP_Controller_Buoyancy::calculate_future_extrapolation(const IVP_Controller_Buoyancy::Attacher_Interpolator::Last_IO *last_io_vectors,
							     const IVP_MI_Vector *new_input,
							     const IVP_MI_Vector *solution_values,
							     const IVP_DOUBLE d_time,
							     const IVP_Time current_time,
							     IVP_MI_Vector *future_input,
							     IVP_MI_Vector *future_solution) {
    
    IVP_DOUBLE future_time = current_time.get_time() + attacher_buoyancy->template_buoyancy.nr_future_psi_for_extrapolation*d_time;
    IVP_DOUBLE divisor = current_time - last_io_vectors->last_psi_time;
    if (divisor > P_DOUBLE_RES) {
	IVP_FLOAT time_quotient = (IVP_FLOAT) (future_time - last_io_vectors->last_psi_time) / divisor;
	
	future_input->set(new_input);
	future_input->subtract(last_io_vectors->last_input_vector);
	future_input->mult( time_quotient );
	future_input->add(last_io_vectors->last_input_vector);
	
	future_solution->set(solution_values);
	future_solution->subtract(last_io_vectors->last_solution_vector);
	future_solution->mult( time_quotient );
	future_solution->add(last_io_vectors->last_solution_vector);

	return(IVP_OK);
    }

    IVP_IF(1) {
	printf("No extrapolation possible!\n");
    }
    return(IVP_FAULT);
}





/*****************************************************************************
 * Name:        use_buoyancy_solver(...)
 * Description: calls the 'IVP_Buoyancy_Solver' and puts the results into
 *              'solution_values_out'
 *****************************************************************************/
IVP_BOOL IVP_Controller_Buoyancy::use_buoyancy_solver(const IVP_Buoyancy_Input *b_input,
						      const IVP_Template_Buoyancy *temp_buoyancy,
						      IVP_Buoyancy_Output *solution_values_out,
						      const IVP_U_Float_Point *resulting_speed_of_current_ws,
						      int index_attacher_interpolator) {

    IVP_Buoyancy_Solver bs( core, this, temp_buoyancy, resulting_speed_of_current_ws );

    IVP_U_Float_Point rel_speed_of_current_os_aligned; rel_speed_of_current_os_aligned.set(&b_input->rel_speed_of_current_os);
    //IVP_U_Float_Point surface_os_aligned; surface_os_aligned.set(&b_input->surface_os);
    IVP_BOOL in_water = bs.compute_forces(
	&rel_speed_of_current_os_aligned,
	&b_input->surface_os,
	attacher_interpolator[index_attacher_interpolator].object);

    if (in_water) {
		
	solution_values_out->volume_under = bs.volume_under;
	solution_values_out->volume_center_under.set(&bs.volume_center_under);
	solution_values_out->object_visible_surface_content_under= bs.object_visible_surface_content_under;
	solution_values_out->sum_impulse.set(&bs.sum_impulse);
	solution_values_out->sum_impulse_x_point.set(&bs.sum_impulse_x_point);
	solution_values_out->sum_impulse_x_movevector.set(&bs.sum_impulse_x_movevector);
	
#if 0
	printf("Buoyancy-Solver's results:\n");
	((IVP_MI_Vector*) solution_values_out)->print();
#endif
		
    }
    return(in_water);
}



/**********************************************************************************
 * Name:        apply_dampening(...)
 * Description: applies the dampening forces to the object provided as a parameter
 **********************************************************************************/
void IVP_Controller_Buoyancy::apply_dampening( IVP_Real_Object *object,
					       IVP_FLOAT object_visible_surface_content_under,
					       IVP_DOUBLE delta_time,
					       IVP_U_Float_Point *sum_impulse_x_movevector,
					       IVP_U_Float_Point *sum_impulse_x_point,
					       IVP_U_Float_Point *sum_impulse) {	
  
#ifdef DAMPENING_WITH_MOVEVECTOR

    //mult the (user definable) torque_factor with the object_visible_surface_content_under, also turns around the move vector (multiply with -1.0f)
    IVP_FLOAT move_factor = -attacher_buoyancy->template_buoyancy.torque_factor * IVP_Inline_Math::ivp_sqrtf(object_visible_surface_content_under) * (0.1f);
	
    //mult sum_impulse_x_movevector with the modified move_factor and delta_PSI_time
    sum_impulse_x_movevector->mult(move_factor * delta_time);
#endif
	
    //mult the other global variables with delta_PSI_time
    sum_impulse_x_point->mult(delta_time);
    sum_impulse->mult(delta_time);
	
	
    //convert sum_impulse into world coordinate system
    IVP_U_Float_Point sum_impulse_ws;
    {
	IVP_Cache_Object *cache_object = object->get_cache_object_no_lock();
	IVP_U_Float_Point imp; imp.set(sum_impulse);
	cache_object->transform_vector_to_world_coords(&imp, &sum_impulse_ws);
    }
	
    IVP_U_Matrix m_core_f_object;
    object->calc_m_core_f_object(&m_core_f_object);
	
    //get the core center
    IVP_U_Point object_center_ws;
    {
	IVP_Cache_Object *co = object->get_cache_object_no_lock();
	object_center_ws.set( co->m_world_f_object.get_position());
    }
	
    //push the core center with the sum of impulses => translation (and a rotation component if the object center is not the same as the core system)
    IVP_U_Float_Point non_float_sum_impulse_ws(&sum_impulse_ws);
		
    core->async_push_core_ws( &object_center_ws, &non_float_sum_impulse_ws);
    IVP_IF(0) {
	printf("core->speed_change.real_length = %f\n", core->speed_change.real_length());
	IVP_ASSERT(core->speed_change.real_length() < 70000.0f);
	core->core_plausible_check();
    }
	
    //convert rotation related values to core system
    IVP_U_Float_Point sum_impulse_x_point_cs;
    IVP_U_Float_Point sum_impulse_x_movevector_cs;
    {
	m_core_f_object.vmult3(sum_impulse_x_point, &sum_impulse_x_point_cs);
	m_core_f_object.vmult3(sum_impulse_x_movevector, &sum_impulse_x_movevector_cs);
    }
	
    {
	//calc new rot_speed
	IVP_U_Float_Point delta_rot_component1_cs;
	delta_rot_component1_cs.set_pairwise_mult(&sum_impulse_x_point_cs, core->get_inv_rot_inertia());
#ifdef DAMPENING_WITH_MOVEVECTOR
	IVP_U_Float_Point delta_rot_component2_cs;
	delta_rot_component2_cs.set_pairwise_mult(&sum_impulse_x_movevector_cs, core->get_inv_rot_inertia());
	delta_rot_component1_cs.add(&delta_rot_component2_cs);
#endif
		
	//apply rotation to object
	core->rot_speed_change.add(&delta_rot_component1_cs);
	IVP_IF(1) {
	    core->core_plausible_check();
	}

	//if object is a ball then apply dampening of rotation which can result e.g. from the ball
	//falling not straightly into the medium
	if (object->get_type() == IVP_BALL) {
	    IVP_Ball *ball = (IVP_Ball *) object;
	    IVP_FLOAT radius = ball->get_radius();
	    IVP_U_Float_Point rotation_dampening;
	    rotation_dampening.set_multiple(&core->rot_speed,
					    attacher_buoyancy->template_buoyancy.ball_rot_dampening_factor *
					    object_visible_surface_content_under / (2*IVP_PI*radius*radius));
	    core->rot_speed_change.subtract(&rotation_dampening);
	    IVP_IF(1) {
		core->core_plausible_check();
	    }
	}
    }

#if 0
    printf("In apply_dampening:\n");
    sum_impulse_x_movevector->print("sum_impulse_x_movevector");
    sum_impulse_x_point->print("sum_impulse_x_point");
#endif
    
    //core->rot_speed_change.print("rot_speed_change");
}



/*********************************************************************************
 * Name:        apply_buoyancy_impulse(...)
 * Description: applies the buoyancy forces to the object provided as a parameter
 *********************************************************************************/
void IVP_Controller_Buoyancy::apply_buoyancy_impulse( IVP_Real_Object *object,
						      IVP_Template_Buoyancy *temp_buoyancy,
						      IVP_DOUBLE delta_time,
						      IVP_FLOAT volume_under,
						      IVP_U_Float_Point *volume_center_under) {
	
    //vector indicating the direction the forces will push
    IVP_U_Float_Point dir_ws(core->environment->get_gravity());  //the buoyancy impulse works in the opposite direction of the gravity
    
    //compute the buoyancy of the part under the surface
    if (volume_under > temp_buoyancy->buoyancy_eps) {
		
	//convert the IVP_U_Float_Point volume_center_under to IVP_U_Point (ws)
	//to preserve precision in the world system
	IVP_U_Point volume_center_under_ws;
	{
	    IVP_Cache_Object *cache_object = object->get_cache_object_no_lock();  //needed to transform coordinates into other coord. systems
	    cache_object->transform_position_to_world_coords(volume_center_under, &volume_center_under_ws);
	}
		
	IVP_DOUBLE force_under = volume_under * (-temp_buoyancy->medium_density) * (delta_time);
	IVP_U_Float_Point impulse_ws;
	impulse_ws.set_multiple(&dir_ws, force_under);

	IVP_IF(0) {
	    impulse_ws.print("buoyancy-impulse");
	    volume_center_under_ws.print("volume_center_under_ws");
	    volume_center_under->print("volume_center_under_os");
	    printf("volume_under = %f\n\n", volume_under);
	}
		
	core->async_push_core_ws(&volume_center_under_ws, &impulse_ws);

	IVP_IF(1) {
	    core->core_plausible_check();
	}
    }
}



//prints the values calculated by the buoyancy solver to stdout
void ivp_debug_show_real_values(const IVP_Buoyancy_Input * /*b_input*/,
				IVP_Controller_Buoyancy *cntrl,
				const IVP_Template_Buoyancy *temp_buoyancy,
				IVP_Core *core,
				IVP_Real_Object *object,
				const IVP_U_Float_Point *resulting_speed_of_current_ws) {
  IVP_USE(object);
    IVP_Buoyancy_Output solution_values_out;
    IVP_Buoyancy_Solver bs( core, cntrl, temp_buoyancy, resulting_speed_of_current_ws );
    //    IVP_BOOL in_water = bs.compute_forces(&b_input->rel_speed_of_current_os, &b_input->surface_os, object);

	solution_values_out.volume_under = bs.volume_under;
	solution_values_out.object_visible_surface_content_under= bs.object_visible_surface_content_under;
	solution_values_out.sum_impulse.set(&bs.sum_impulse);
	solution_values_out.sum_impulse_x_point.set(&bs.sum_impulse_x_point);
	solution_values_out.sum_impulse_x_movevector.set(&bs.sum_impulse_x_movevector);
	
	printf("Buoyancy-Solver's results:\n");
	((IVP_MI_Vector*)&solution_values_out)->print();
		
}



/*************************************************************************************
     * Name:        provide_new_input_solution_combination(...)
     * Description: combines various ways of inserting new vectors of input and solution
     *              values into 'previous_inputs' and 'previous_solutions' of the current
     *              'IVP_Multidimensional_Interpolator' instance
     *************************************************************************************/
void IVP_Controller_Buoyancy::provide_new_input_solution_combination(Attacher_Interpolator *ai,
								     IVP_Template_Buoyancy *temp_buoyancy,
								     const IVP_MI_Vector *weighted_new_input,
								     const IVP_MI_Vector *solution_values,
								     const IVP_DOUBLE d_time,
								     const IVP_Time current_time) {

	float input_data[ INPUT_VECTOR_LENGTH ];
	float solution_data[ SOLUTION_VECTOR_LENGTH ];
	IVP_MI_Vector *future_input = (IVP_MI_Vector *)&input_data[0];
	IVP_MI_Vector *future_solution = (IVP_MI_Vector *)&solution_data[0];

	IVP_MI_VECTOR_CLEAR(future_input, INPUT_VECTOR_LENGTH);
	IVP_MI_VECTOR_CLEAR(future_solution, SOLUTION_VECTOR_LENGTH);

    switch(attacher_buoyancy->template_buoyancy.use_stochastic_insertion) {

    case IVP_TRUE: {

		    //use stochastic
		    if (ai->mi->get_nr_occupied() < ai->mi->get_nr_of_vectors()) {
			//weighted_new_input->set_time_stamp(current_time);
			//solution_values->set_time_stamp(current_time);
			if(!(temp_buoyancy->insert_extrapol_only)) {
			    ai->mi->add_new_input_solution_combination_conventional(weighted_new_input, solution_values);
			}

			//extrapolate into the future
			IVP_RETURN_TYPE success = calculate_future_extrapolation(&ai->last_io_vectors,
										 weighted_new_input,
										 solution_values,
										 d_time,
										 current_time,
										 future_input,
										 future_solution);
			if (success) {
			    future_input->set_time_stamp(current_time);
			    future_solution->set_time_stamp(current_time);
			    ai->mi->add_new_input_solution_combination_conventional(future_input, future_solution);
			}
		    } else {
			//weighted_new_input->set_time_stamp(current_time);
			//solution_values->set_time_stamp(current_time);
			if(!(temp_buoyancy->insert_extrapol_only)) {
			    ai->mi->add_new_input_solution_combination_stochastic(weighted_new_input, solution_values);
			}

			//extrapolate into the future
			IVP_RETURN_TYPE success = calculate_future_extrapolation(&ai->last_io_vectors,
										 weighted_new_input,
										 solution_values,
										 d_time,
										 current_time,
										 future_input,
										 future_solution);
			if (success) {
			    future_input->set_time_stamp(current_time);
			    future_solution->set_time_stamp(current_time);
			    ai->mi->add_new_input_solution_combination_stochastic(future_input, future_solution);
			}
		    }

		    break;
    }
    case IVP_FALSE: {
	//weighted_new_input->set_time_stamp(current_time);
	//solution_values->set_time_stamp(current_time);
	if(!(temp_buoyancy->insert_extrapol_only)) {
	    ai->mi->add_new_input_solution_combination_conventional(weighted_new_input, solution_values);
	}
	    
	//extrapolate into the future
	IVP_RETURN_TYPE success = calculate_future_extrapolation(&ai->last_io_vectors,
								 weighted_new_input,
								 solution_values,
								 d_time,
								 current_time,
								 future_input,
								 future_solution);
	if (success) {
	    future_input->set_time_stamp(current_time);
	    future_solution->set_time_stamp(current_time);
	    ai->mi->add_new_input_solution_combination_conventional(future_input, future_solution);
	}
	break;
    }
	
    } //end switch statement
    
#if 0
    if (ai->mi->get_nr_occupied() < ai->mi->get_nr_of_vectors()) {
	//the array of input vectors in the multidim. interpolator is not yet fully occupied
	ai->mi->add_new_input_solution_combination_conventional(new_input, solution_values);
    } else {
	//array of input vectors is already fully occupied, so a member has to be replaced by a new one
	ai->mi->add_new_input_solution_combination_stochastic(new_input, solution_values);
    }
#endif    
    }
    
    
    
/********************************************************************
 * Name:        do_simulation_controller(...)
 * Description: Simulates the fluid dynamics in the current PSI step
 *              for the core in its class variable 'core'
 * Note:        do_simulation_controller is called by the internal
 *              simulation unit of the physics engine
 ********************************************************************/
void IVP_Controller_Buoyancy::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> *) {

    IVP_Environment *environment = es->environment;
    IVP_U_Float_Hesse surface_hesse_ws;
    IVP_U_Float_Point abs_speed_of_current_ws;

    //calc the surface of the medium the current core (perhaps) lies in
    attacher_buoyancy->liquid_surface_descriptor->calc_liquid_surface( environment, core, &surface_hesse_ws, &abs_speed_of_current_ws );

    // if area is increased, reduce mean velocity of water
    if (core_visible_surface_content_under > core_visible_surface_content_under_old) {
	relative_speed_of_current_in_objects_vicinity_old.mult(core_visible_surface_content_under_old / core_visible_surface_content_under);
    }

    //update core_visible_surface_content_under(_old)
    core_visible_surface_content_under_old = core_visible_surface_content_under;
    core_visible_surface_content_under = 0.0f;

    
    //calc resulting_speed_of_current
    IVP_U_Float_Point resulting_speed_of_current_ws;
#ifdef DAMPENING_WITH_PARTICLE_ACCELERATION
    resulting_speed_of_current_ws.set(&abs_speed_of_current_ws);
    resulting_speed_of_current_ws.add(&relative_speed_of_current_in_objects_vicinity_old);
#else
    resulting_speed_of_current_ws.set(&abs_speed_of_current_ws);
#endif
    IVP_U_Float_Point rel_speed_of_current_ws;
    rel_speed_of_current_ws.subtract(&resulting_speed_of_current_ws, &core->speed);

    IVP_Template_Buoyancy *temp_buoyancy = attacher_buoyancy->get_parameters_per_core( core);
    
    /*** calc buoyancy and dampening for every object in the core ***/

    int nr_of_objects = core->objects.len();

    for(int i=0; i<nr_of_objects; i++) {
	
	IVP_U_Float_Point rel_speed_of_current_os;
	IVP_U_Float_Hesse surface_os;  //surface in object's coordinate system
	{
	//calc surface_os
	    //translate resulting_speed_of_current_ws into object coordinate system
	    IVP_Cache_Object *cache_object = attacher_interpolator[i].object->get_cache_object_no_lock();
	    cache_object->transform_vector_to_object_coords(&rel_speed_of_current_ws, &rel_speed_of_current_os);
	    cache_object->transform_vector_to_object_coords(&surface_hesse_ws, &surface_os);
	    IVP_U_Matrix m_world_f_object;
	    attacher_interpolator[i].object->get_m_world_f_object_AT(&m_world_f_object);
	    IVP_DOUBLE dist = m_world_f_object.get_position()->dot_product(&surface_hesse_ws);
	    surface_os.hesse_val = surface_hesse_ws.hesse_val + dist;
	}

	//create new input vector without using weights, needed for the buoyancy solver
	IVP_Buoyancy_Input b_input;
	//IVP_MI_Vector *new_input = (IVP_MI_Vector *)&b_input;
	b_input.rel_speed_of_current_os.set(&rel_speed_of_current_os);
	b_input.surface_os.set(&surface_os);
	b_input.surface_os.hesse_val = surface_os.hesse_val;
	b_input.rot_speed.set(&core->rot_speed);

	//create new input vector and use the provided weights
	IVP_Buoyancy_Input weighted_b_input;
	IVP_MI_Vector *weighted_new_input = (IVP_MI_Vector *)&weighted_b_input;
	weighted_b_input.rel_speed_of_current_os.set_multiple(&rel_speed_of_current_os, temp_buoyancy->mi_weights.weight_current_speed);
	weighted_b_input.surface_os.set_multiple(&surface_os, temp_buoyancy->mi_weights.weight_surface);
	weighted_b_input.surface_os.hesse_val = surface_os.hesse_val * temp_buoyancy->mi_weights.weight_surface;
	weighted_b_input.rot_speed.set_multiple(&core->rot_speed, temp_buoyancy->mi_weights.weight_rot_speed);

	//reserve memory on the stack for the new solutions
	IVP_Buoyancy_Output b_output;
	IVP_MI_Vector *solution_values = (IVP_MI_Vector *)&b_output;
	
#if !defined(IVP_NO_MD_INTERPOLATION)
	//check if interpolation should be used or not
	if (temp_buoyancy->use_interpolation) {   //use interpolation
	    IVP_RETURN_TYPE result;

	    //force a new buoyancy solver run if the interpolation_counter has reached the maximum value allowed
	    if (interpolation_counter++ >= temp_buoyancy->max_interpolation_tries) {
		result = IVP_FAULT;          //force new buoyancy_solver run
		interpolation_counter = 0;   //reinitialize counter
	    } else {
		result = attacher_interpolator[i].mi->check_interpolation(weighted_new_input,
									  temp_buoyancy->max_tries_nr_of_vectors_involved,
									  temp_buoyancy->max_res,
									  solution_values);
	    }

	    if (result) {
		//interpolation was successful
		//fetch new solution values

#ifdef WITH_DEBUG_OUTPUT
		printf("Interpolated!\n");
		ivp_debug_show_real_values(&b_input,
					   temp_buoyancy,
					   core,
					   attacher_interpolator->object,
					   &resulting_speed_of_current_ws);
		attacher_interpolator[i].nr_interpolated++;
#endif
	    } else {
		//interpolation failed
		//call the buoyancy solver

#ifdef WITH_DEBUG_OUTPUT
		printf("Not Interpolated!\n");
		attacher_interpolator[i].nr_not_interpolated++;
#endif
		IVP_BOOL in_water = use_buoyancy_solver(&b_input, temp_buoyancy, &b_output, &resulting_speed_of_current_ws, i);
		if (in_water) {
		    //if the buoyancy solver did compute values (if object is in water) then copy the new
		    //input/solution-combination into the interpolator
		    provide_new_input_solution_combination(&attacher_interpolator[i],
							   temp_buoyancy,
							   weighted_new_input,
							   solution_values,
							   es->delta_time,
							   environment->get_current_time());
		}
#ifdef WITH_DEBUG_OUTPUT
		else {
		    printf("Not yet in water!\n");
		}
#endif
	    }
#ifdef WITH_DEBUG_OUTPUT
	    printf("times_interpolated : times_calculated = %d : %d\n", attacher_interpolator[i].nr_interpolated, attacher_interpolator[i].nr_not_interpolated);
	    printf("# of one vector already sufficient = %d\n", attacher_interpolator[i].mi->nr_one_vector_sufficient);
	    for (int q=0; q<attacher_interpolator[i].mi->get_nr_of_vectors(); q++) {
		printf("%d vectors involved:  # of res over limit = %d     ", q+1, attacher_interpolator[i].mi->nr_res_over_limit[q]);
		printf("# of interpol. weigh over limit = %d     ", attacher_interpolator[i].mi->nr_int_weight_over_limit[q]);
		printf("# of times LINFIT failed = %d     ", attacher_interpolator[i].mi->nr_of_linfit_failure[q]);
		printf("# of interpolation successes = %d\n", attacher_interpolator[i].mi->nr_of_success[q]);
	    }
	    printf("\n\n");
#endif

	    //update last_input structure, is needed in 'calculate_future_extrapolation'
	    attacher_interpolator[i].last_io_vectors.last_input_vector->set(weighted_new_input);
	    attacher_interpolator[i].last_io_vectors.last_solution_vector->set(solution_values);
	    //	    attacher_interpolator[i].last_io_vectors.last_psi_time = environment->get_current_time();
	    attacher_interpolator[i].last_io_vectors.last_psi_time = environment->get_current_time().get_time();

	} else {
#endif /* if !defined(IVP_NO_MD_INTERPOLATION) */	
	    //don't use interpolation
	    use_buoyancy_solver(&b_input, temp_buoyancy, &b_output, &resulting_speed_of_current_ws, i);
#ifdef WITH_DEBUG_OUTPUT
	    printf("nr_not_interpolated = %d\n", ++nr_not_interpolated);
#endif
#if !defined(IVP_NO_MD_INTERPOLATION)
	}
#endif	


	//apply the calculated impulses to the current object
	apply_buoyancy_impulse( attacher_interpolator[i].object,
				temp_buoyancy,
				es->delta_time,
				b_output.volume_under,
				&b_output.volume_center_under );

	apply_dampening( attacher_interpolator[i].object,
			 IVP_Inline_Math::fabsd(b_output.object_visible_surface_content_under),
			 es->delta_time,
			 &b_output.sum_impulse_x_movevector,
			 &b_output.sum_impulse_x_point,
			 &b_output.sum_impulse);
	
	//add 'object_visible_surface_content_under' to 'core_visible_surface_content_under'
	//'fabs' is taken in case the interpolation's result was negative
	core_visible_surface_content_under += IVP_Inline_Math::fabsd(b_output.object_visible_surface_content_under);
    }

#ifdef DAMPENING_WITH_PARTICLE_ACCELERATION
    IVP_FLOAT inverse_viscosity = 1.0f - (temp_buoyancy->viscosity_factor * es->delta_time);
    IVP_U_Float_Point speed_of_object_relative_to_medium_ws;
    speed_of_object_relative_to_medium_ws.set(&core->speed);
    speed_of_object_relative_to_medium_ws.subtract(&resulting_speed_of_current_ws);
    
    relative_speed_of_current_in_objects_vicinity_old.mult(inverse_viscosity);
    relative_speed_of_current_in_objects_vicinity_old.add_multiple(&speed_of_object_relative_to_medium_ws, temp_buoyancy->viscosity_input_factor*es->delta_time);
#endif
}



/*********************
 * Constructor
 *********************/
IVP_Controller_Buoyancy::IVP_Controller_Buoyancy(IVP_Attacher_To_Cores<IVP_Controller_Buoyancy> *attacher_buoyancy_, IVP_Core *core_) {
    attacher_buoyancy = (IVP_Attacher_To_Cores_Buoyancy *)attacher_buoyancy_;
    core = core_;
    core->environment->get_controller_manager()->add_controller_to_core(this,core);

    relative_speed_of_current_in_objects_vicinity_old.set(0.0f, 0.0f, 0.0f);
    core_visible_surface_content_under_old = 0.0f;
    core_visible_surface_content_under = 0.0f;

    //initialize vector which applies a multidimensional interpolator to every object in the core
    int nr_of_objects = core->objects.len();

	attacher_interpolator = new Attacher_Interpolator[nr_of_objects];
	for(int i=0; i<nr_of_objects; i++) {
	    attacher_interpolator[i].object = core->objects.element_at(i);
#if !defined( IVP_NO_MD_INTERPOLATION )
	    attacher_interpolator[i].mi = new IVP_Multidimensional_Interpolator(5, INPUT_VECTOR_LENGTH, SOLUTION_VECTOR_LENGTH);
		attacher_interpolator[i].last_io_vectors.last_input_vector = IVP_MI_Vector::malloc_mi_vector(INPUT_VECTOR_LENGTH);
	    attacher_interpolator[i].last_io_vectors.last_solution_vector = IVP_MI_Vector::malloc_mi_vector(SOLUTION_VECTOR_LENGTH);
#endif
	    attacher_interpolator[i].last_io_vectors.last_psi_time = 0;
	    attacher_interpolator[i].nr_interpolated = 0;
	    attacher_interpolator[i].nr_not_interpolated = 0;
	}
    
    interpolation_counter = 0; //initialize the counter

    //for debugging
    nr_not_interpolated = 0;
}



/*********************
 * Destructor
 *********************/
IVP_Controller_Buoyancy::~IVP_Controller_Buoyancy() {
    int nr_of_objects = core->objects.len();
    if ( attacher_interpolator){
	for(int i=0; i<nr_of_objects; i++) {
#if !defined( IVP_NO_MD_INTERPOLATION )
	    P_DELETE(attacher_interpolator[i].mi);
	    //#if !defined(__MWERKS__) || !defined(__POWERPC__)
#pragma message ("this code should not be called")
	    P_DELETE_ARRAY(attacher_interpolator[i].last_io_vectors.last_input_vector);
	    P_DELETE_ARRAY(attacher_interpolator[i].last_io_vectors.last_solution_vector);
	    //#endif
#endif
	}
	P_DELETE_ARRAY(attacher_interpolator); //@@CB was a P_DELETE, which was bad
    }
    
    attacher_buoyancy->attachment_is_going_to_be_deleted(this,core);
    core->environment->get_controller_manager()->remove_controller_from_core(this,core);
}



IVP_Attacher_To_Cores_Buoyancy::IVP_Attacher_To_Cores_Buoyancy(IVP_Template_Buoyancy &templ, IVP_U_Set_Active<IVP_Core> *set_of_cores_, IVP_Liquid_Surface_Descriptor *liquid_surface_descriptor_)
    : IVP_Attacher_To_Cores<IVP_Controller_Buoyancy>(set_of_cores_)
{
    template_buoyancy = templ;
    set_of_cores = set_of_cores_;
    liquid_surface_descriptor = liquid_surface_descriptor_;
};

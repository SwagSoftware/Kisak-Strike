// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#ifndef WIN32
#	pragma implementation "ivp_calc_next_psi_solver.hxx"
#endif
#include <ivu_float.hxx>
#include <ivp_core_macros.hxx>
#include <ivp_calc_next_psi_solver.hxx>
#include <ivp_hull_manager.hxx>
#include <ivu_min_hash.hxx>
#include <ivp_hull_manager_macros.hxx>
#include <ivp_debug_manager.hxx>
#include <ivp_mindist_intern.hxx>
#include <ivp_controller.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_performancecounter.hxx>
#include <ivp_constraint_car.hxx>

IVP_DOUBLE IVP_Calc_Next_PSI_Solver::get_longest_time_step_dependent_on_rot() {
    IVP_DOUBLE rot_speed_scalar = core->rot_speed.real_length() + P_DOUBLE_EPS;
    return ( MAX_OBJECT_ROT_SPEED / rot_speed_scalar );
}

/********************
 * calculate the rotation quaternion
 *************/
inline void IVP_Calc_Next_PSI_Solver::calc_rotation_matrix(IVP_FLOAT delta_sim_time, IVP_U_Quat *q_core_f_core){
  IVP_Core *pc = core;

  // collision events do not want to calculate gyro effects because
  // they are based on synchronize_with_rot_z
  if ( pc->rot_inertias_are_equal || pc->environment->get_env_state() == IVP_ES_AT){

#ifdef IVP_FAST_WHEELS_ENABLED   // @@@OSP
      if ( pc->car_wheel && (pc->max_surface_deviation == 0.0f) ){
		  int axis = pc->car_wheel->solver_car->x_idx;
		  IVP_DOUBLE        rot_speed_wheel_axis = pc->rot_speed.k[axis];
		  IVP_U_Float_Point p(pc->rot_speed);	p.k[axis] = 0.0f;

		  IVP_U_Quat gyro_rotation;				gyro_rotation.set_fast_multiple_with_clip(&p, delta_sim_time);

		  IVP_U_Quat wheel_rotation; 

		  p.set_to_zero();
		  IVP_DOUBLE sinus = IVP_Inline_Math::sind(rot_speed_wheel_axis*0.5f * delta_sim_time);
		  p.k[axis] = sinus;

		  wheel_rotation.x = p.k[0];
		  wheel_rotation.y = p.k[1];
		  wheel_rotation.z = p.k[2];

		  wheel_rotation.w = IVP_Inline_Math::sqrtd(1.0f - sinus * sinus);
		  
		  q_core_f_core->inline_set_mult_quat(&gyro_rotation, &wheel_rotation);
	  }
	  else
#endif
	  {
		q_core_f_core->set_fast_multiple_with_clip( &pc->rot_speed,delta_sim_time);
	  }

      return;
  }

  const IVP_DOUBLE max_angle = 5.0f * 6.0f / 360.0f;	// ten degrees per simulation step
  const IVP_DOUBLE qmax_angle = max_angle * max_angle;

  IVP_DOUBLE square_rot_speed = pc->rot_speed.quad_length(); //was already calculated in clip rot speed
  IVP_DOUBLE dt = delta_sim_time;
  IVP_DOUBLE qangle = square_rot_speed * dt * dt;

  // calc number of steps needed for dirty simulation
  int steps = 1;
  {
    if ( qangle > 4.0f * qmax_angle){
      steps = 1 + (int)IVP_Inline_Math::ivp_sqrtf( qangle / qmax_angle );
      dt /=  IVP_FLOAT(steps);
    }
  }
  /* divide matrix into small rotational matrizes,
   * so that angle < .1f == 60 Steps per 360
   */

  q_core_f_core->set_very_fast_multiple( &pc->rot_speed,dt);

  const IVP_U_Float_Point *ri = pc->get_rot_inertia();
  const IVP_U_Float_Point *iri = pc->get_inv_rot_inertia();

  IVP_U_Point diff_other_rot_inertia_div_this(
	(ri->k[1] - ri->k[2]) * iri->k[0],
	(ri->k[2] - ri->k[0]) * iri->k[1],
	(ri->k[0] - ri->k[1]) * iri->k[2]);
  for (int i = 1;; i++){
      IVP_U_Float_Point hp;
      hp.set ( pc->rot_speed.k[1] * pc->rot_speed.k[2] * diff_other_rot_inertia_div_this.k[0],
	       pc->rot_speed.k[2] * pc->rot_speed.k[0] * diff_other_rot_inertia_div_this.k[1],
	       pc->rot_speed.k[0] * pc->rot_speed.k[1] * diff_other_rot_inertia_div_this.k[2]);
      
      pc->rot_speed.add_multiple(&hp,dt);  
      if (i >= steps) break;
      IVP_U_Quat rot_matrix;
      rot_matrix.set_very_fast_multiple( &pc->rot_speed,dt);
      q_core_f_core->inline_set_mult_quat(&rot_matrix, q_core_f_core);
  }
}

// returns the next simulation slot and the extra time step for faster simulation ( 0 means no extra time step )
#if 0
void IVP_Core::get_next_simulation_slot(IVP_DOUBLE next_lowest_psi,int *next_used_slot,int *time_step)
{
    IVP_DOUBLE current_rot_speed = this->rot_speed.real_length();
    IVP_DOUBLE max_time_step_to_simulate = ( MAX_OBJECT_ROT_SPEED / (current_rot_speed + DOUBLE_EPS) );
    
    IVP_DOUBLE rotation_estimate = current_rot_speed * this->max_surface_deviation * IVP_SAFETY_FACTOR_HULL; //1.01f for safety (an error in this one can cause hangup)
    IVP_DOUBLE gradient_estimate=  rotation_estimate + this->current_speed; 
	
    for(int c = objects.len()-1;c>=0;c--){
	IVP_Real_Object *r_obj=this->objects.element_at(c);
	IVP_Hull_Manager *h_manager = r_obj->get_hull_manager();

	IVP_DOUBLE obj_time_max = h_manager->get_highest_time_before_event(time_of_last_psi, gradient_estimate);
	if(obj_time_max < max_time_step_to_simulate) {
	    max_time_step_to_simulate = obj_time_max;
	}
    }
	
    IVP_DOUBLE time_step_forward = max_time_step_to_simulate + time_of_last_psi - next_lowest_psi; //offset
    
    int time_step_multiplier=(int)( time_step_forward * environment->get_inv_delta_PSI_time() );
    if(time_step_multiplier<1) {
	goto no_sim_speedup;
    }
    if(time_step_multiplier>= IVP_DEBRIS_DELAY_NUM-1) {
	time_step_multiplier= IVP_DEBRIS_DELAY_NUM-2;    // do not use full possible ahead entry, because debris_sim_index is pointing to it right now 
    }
    {
	int next_sim_man_slot = this->environment->get_core_sim_manager()->next_debris_sim_slot + time_step_multiplier; 
	if(next_sim_man_slot>= IVP_DEBRIS_DELAY_NUM) {
	    next_sim_man_slot-= IVP_DEBRIS_DELAY_NUM;
	}
	next_sim_man_slot++; // debris slots begin with 1
	*next_used_slot=next_sim_man_slot;
	*time_step=time_step_multiplier;
	return;
    }
no_sim_speedup:
    *next_used_slot=0; //simulate in next PSI
    *time_step=0;
}
#endif


/*new version*/
/* only called of moving objects */
/* needs speed rot_speed  next_psi_time */
void IVP_Calc_Next_PSI_Solver::calc_next_PSI_matrix(IVP_Event_Sim *event_sim,IVP_U_Vector<IVP_Hull_Manager_Base> *active_hull_managers_out){
   
   IVP_IF(1) {
        IVP_Debug_Manager *dm=event_sim->environment->get_debug_manager();
	if(dm->file_out_impacts) {
	    fprintf(dm->out_deb_file,"making_calc_next_psi %lx at %f\n",0x0000ffff&(long)this,core->environment->get_current_time().get_time());
	}
    }    

   if (!( core->car_wheel && (core->max_surface_deviation == 0.0f) )) { //@@CB
		core->clip_velocity(&core->speed, &core->rot_speed);
	}
   
    core->current_speed = core->speed.real_length();
    IVP_FLOAT simulation_delta_time = event_sim->delta_time;
    core->i_delta_time = event_sim->i_delta_time;
    
    IVP_U_Quat q_core_f_core;
    this->calc_rotation_matrix(simulation_delta_time,&q_core_f_core);

    {   // update AT positions !!!!
	IVP_DOUBLE d_last_time = core->environment->get_current_time() - core->time_of_last_psi;
	core->time_of_last_psi = core->environment->get_current_time();
	core->pos_world_f_core_last_psi.add_multiple( &core->delta_world_f_core_psis, d_last_time);
	core->delta_world_f_core_psis.set( &core->speed );

	core->q_world_f_core_last_psi = core->q_world_f_core_next_psi;
	core->q_world_f_core_next_psi.inline_set_mult_quat( &core->q_world_f_core_next_psi, &q_core_f_core);
	core->q_world_f_core_next_psi.fast_normize_quat();
    }
    
    IVP_Hull_Manager *h_manager_0;
    IVP_IF_PREFETCH_ENABLED(IVP_TRUE){
	IVP_Real_Object *r_obj=core->objects.element_at(0);
	h_manager_0 = r_obj->get_hull_manager();
	h_manager_0->prefetch0_gradient();
    }
    calc_psi_rotation_axis(&q_core_f_core);
    
    IVP_DOUBLE new_speed = core->max_surface_rot_speed + core->current_speed;
    for(int c = core->objects.len()-1;c>=0;c--){
	IVP_Real_Object *r_obj = core->objects.element_at(c);
	IVP_Hull_Manager *h_manager = r_obj->get_hull_manager();
	h_manager->increase_hull_by_x(event_sim->environment->get_current_time(), event_sim->delta_time, new_speed, core->current_speed);
	if (h_manager->are_events_in_hull()){
	    active_hull_managers_out->add(h_manager);
	}
    }
}




void IVP_Calc_Next_PSI_Solver::calc_psi_rotation_axis(const IVP_U_Quat *q_core_f_core){

    // calc rotation axis in world space and rotation speed. using quaternions
    IVP_DOUBLE qlen = q_core_f_core->x * q_core_f_core->x +
	q_core_f_core->y * q_core_f_core->y +
	q_core_f_core->z * q_core_f_core->z;
    if ( qlen > P_DOUBLE_EPS ){
	IVP_DOUBLE ilen = IVP_Fast_Math::isqrt(qlen,3);
	IVP_FLOAT abs_angle;			// absolute rotation angle interpolated movement
	abs_angle  = IVP_Inline_Math::upper_limit_asin( qlen * ilen );
	IVP_IF(1){
#if defined(DEBUG) && 0
	    IVP_DOUBLE ref_angle = asin( sqrt(qlen)) - 1E9f; //@@@ was P_DOUBLE_RES; difference between old and new (fast) calculation should not be too great;
	    IVP_ASSERT( abs_angle >= ref_angle );
#endif
	}
	core->abs_omega =  2.0f * abs_angle * core->i_delta_time;


	IVP_U_Point rot_axis_ws;
	core->m_world_f_core_last_psi.inline_vmult3( (IVP_U_Point *)q_core_f_core, &rot_axis_ws );
	core->rotation_axis_world_space.set_multiple( &rot_axis_ws, ilen );
    }else{
	core->abs_omega = 0.0f;
	core->rotation_axis_world_space.set(1.0f,0.0f,0.0f);
    }
    core->max_surface_rot_speed = core->max_surface_deviation * core->abs_omega;
}


void IVP_Calc_Next_PSI_Solver::set_transformation( const IVP_U_Quat *rotation, const IVP_U_Point *position, IVP_BOOL optimize_for_repeated_calls)
{
	IVP_Environment *env = core->get_environment();
	IVP_Time current_time = env->get_current_time();
	
	env->set_current_time( current_time );  // invalid all flags, all caches ....
	
	IVP_FLOAT simulation_delta_time = current_time - core->time_of_last_psi;

	// set time flags for core
	if (simulation_delta_time >= env->get_delta_PSI_time())
	{
		core->i_delta_time = 1.0f / simulation_delta_time;
	}
	else
	{
		simulation_delta_time = env->get_delta_PSI_time();
		core->i_delta_time = env->get_inv_delta_PSI_time();	// infinit
	}
	
	IVP_FLOAT moved_distance_val;

	// set speed values for collision detection
	core->time_of_last_psi = current_time;
	
	core->q_world_f_core_next_psi = *rotation;
	
	IVP_U_Quat q_core_f_core;
	q_core_f_core.set_invert_mult( &core->q_world_f_core_last_psi, &core->q_world_f_core_next_psi);
	core->q_world_f_core_last_psi = *rotation;
	
	
	core->time_of_last_psi = env->get_old_time_of_last_PSI();
	
	IVP_U_Point moved_distance;
	moved_distance.subtract( position, &core->pos_world_f_core_last_psi);
	moved_distance_val = moved_distance.real_length();
	core->pos_world_f_core_last_psi.set(position);
	core->delta_world_f_core_psis.set_to_zero();
	core->speed.set_to_zero();
	
	// update redundant m_world_f_core
	core->m_world_f_core_last_psi.get_position()->set(position);
	rotation->set_matrix(&core->m_world_f_core_last_psi);
	
	// set some speed values so there will be some speed values for hull conversion
	if (optimize_for_repeated_calls)
	{
		core->current_speed = moved_distance_val * core->i_delta_time;  // simulate speed
		this->calc_psi_rotation_axis(&q_core_f_core);
	}
	else
	{
		core->current_speed = 0.0f;
		q_core_f_core.init();
		this->calc_psi_rotation_axis(&q_core_f_core);
	}
	
	// do collision checking
	// calc rotation axis in world space and rotation speed. using quaternions
	IVP_DOUBLE diff = core->max_surface_deviation * core->max_surface_rot_speed * simulation_delta_time + moved_distance_val;
	
	for( int c = core->objects.len()-1; c>=0; c-- )
	{
		IVP_Real_Object *r_obj=core->objects.element_at(c);
		
		IVP_Hull_Manager *h_manager = r_obj->get_hull_manager();
		h_manager->jump_add_hull(diff,moved_distance_val);
		
		if ( !IVP_MTIS_SIMULATED(r_obj->get_movement_state()) )
		{  
			// invalid caches
			IVP_Cache_Object_Manager::invalid_cache_object(r_obj);
			r_obj->recalc_exact_mindists_of_object();
			r_obj->recalc_invalid_mindists_of_object();
		}    
		
		if (!optimize_for_repeated_calls)
		{
			env->get_mindist_manager()->recheck_ov_element(r_obj);
		}

		h_manager->check_hull_synapses();
		h_manager->check_for_reset();
	}
	
	// reset speed values
	core->current_speed = 0.0f;
	core->abs_omega = 0.0f;
	core->max_surface_rot_speed = 0.0f;
}


void IVP_Calc_Next_PSI_Solver::prefetch0_calc_next_PSI_matrix(IVP_Core *core)
{
	IVP_USE(core);
	IVP_IF_PREFETCH_ENABLED(IVP_TRUE)
	{
	    IVP_PREFETCH(core, &((IVP_Core *)0)->rot_speed );
	}
}


void IVP_Calc_Next_PSI_Solver::commit_all_calc_next_PSI_matrix(IVP_Environment *env, IVP_U_Vector<IVP_Core> *cores_which_needs_calc_next_psi, IVP_U_Vector<IVP_Hull_Manager_Base> *active_hulls_out){
    IVP_Event_Sim es(env, env->get_delta_PSI_time());
    int i;
    for (i = cores_which_needs_calc_next_psi->len()-1; i>1;i--){
	IVP_IF_PREFETCH_ENABLED(IVP_TRUE){
	    IVP_Core *pcore = cores_which_needs_calc_next_psi->element_at(i-2);
	    IVP_Calc_Next_PSI_Solver::prefetch0_calc_next_PSI_matrix(pcore);
	}
	IVP_Core *core = cores_which_needs_calc_next_psi->element_at(i);
	IVP_Calc_Next_PSI_Solver nps(core);
	nps.calc_next_PSI_matrix(&es,active_hulls_out);
    }
    if (i == 1){
	IVP_Core *core = cores_which_needs_calc_next_psi->element_at(1);
	IVP_Calc_Next_PSI_Solver nps(core);
	nps.calc_next_PSI_matrix(&es,active_hulls_out);
	i--;
    }
    if (i == 0){
	IVP_Core *core = cores_which_needs_calc_next_psi->element_at(0);
	IVP_Calc_Next_PSI_Solver nps(core);
	nps.calc_next_PSI_matrix(&es,active_hulls_out);
    }
}

void IVP_Calc_Next_PSI_Solver::commit_one_hull_manager( IVP_Environment *, IVP_U_Vector<IVP_Hull_Manager_Base> *active_hull_managers){
    if (active_hull_managers->len()){
	IVP_ASSERT( active_hull_managers->len() == 1 );
	IVP_Hull_Manager *hm = (IVP_Hull_Manager*)active_hull_managers->element_at(0);
	hm->check_hull_synapses();
	hm->check_for_reset();
    }
}

void IVP_Calc_Next_PSI_Solver::commit_all_hull_managers( IVP_Environment *, IVP_U_Vector<IVP_Hull_Manager_Base> *active_hull_managers){
	//h_manager->check_hull_synapses();
	//h_manager->check_for_reset();
    int i;
    for (i = active_hull_managers->len()-1; i>1;i--){
	IVP_IF_PREFETCH_ENABLED(IVP_TRUE){
	    IVP_Hull_Manager *hm2 = (IVP_Hull_Manager *)active_hull_managers->element_at(i-2);
	    hm2->prefetch0_hull();
	    IVP_Hull_Manager *hm1 = (IVP_Hull_Manager *)active_hull_managers->element_at(i-1);
	    hm1->prefetch1_hull();
	}
	IVP_Hull_Manager *hm = (IVP_Hull_Manager *)active_hull_managers->element_at(i);
	hm->check_hull_synapses();
	hm->check_for_reset();
    }
    if (i == 1){
	IVP_Hull_Manager *hm = (IVP_Hull_Manager*)active_hull_managers->element_at(1);
	hm->check_hull_synapses();
	hm->check_for_reset();
	i--;
    }
    if (i == 0){
	IVP_Hull_Manager *hm = (IVP_Hull_Manager*)active_hull_managers->element_at(0);
	hm->check_hull_synapses();
	hm->check_for_reset();
    }
}

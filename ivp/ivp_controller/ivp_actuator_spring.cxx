// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#ifndef WIN32
#	pragma implementation "ivp_actuator_spring.hxx"
#endif

#include <ivp_physics.hxx>
#include <ivp_solver_core_reaction.hxx>

#include <ivu_active_value.hxx>
#include <ivp_actuator.hxx>


//////////////////////////

IVP_Template_Spring::IVP_Template_Spring(){
    P_MEM_CLEAR(this);
    this->break_max_len = P_FLOAT_MAX;
}

/////////////////////////////

IVP_Template_Suspension::IVP_Template_Suspension(){
    P_MEM_CLEAR(this);
    this->break_max_len = 10e8f;
}

/////////////////// SPRING /////////////////

void IVP_Actuator_Spring::fire_event_spring_broken(){
    for (int i = listeners_spring.len()-1; i>=0; i--){
	IVP_Listener_Spring *li = listeners_spring.element_at(i);
	li->event_spring_broken(this);
    }
}

IVP_Actuator_Spring::IVP_Actuator_Spring(IVP_Environment *env,
					 IVP_Template_Spring *spring_templ,
					 IVP_ACTUATOR_TYPE act_type)
                                       : IVP_Actuator_Two_Point(env, spring_templ, act_type)
{
    this->spring_len = spring_templ->spring_len;
    this->break_max_len = spring_templ->break_max_len;
    this->max_len_exceed_type = spring_templ->max_len_exceed_type;
    this->l_environment=env;
    
    /*** interpret some values (now that masses are known) ***/

    // get some physical infos
    IVP_Anchor *anchor_0 = get_actuator_anchor(0);
    IVP_Anchor *anchor_1 = get_actuator_anchor(1);
    IVP_Real_Object *obj_0 = anchor_0->anchor_get_real_object();
    IVP_Real_Object *obj_1 = anchor_1->anchor_get_real_object();
    IVP_Core *core_0 = obj_0->get_core();
    IVP_Core *core_1 = obj_1->get_core();

    spring_values_factor = 1.0f;
    if (spring_templ->spring_values_are_relative){	
	IVP_DOUBLE min_mass_0 = core_0->calc_virt_mass(&anchor_0->core_pos, NULL);
	IVP_DOUBLE min_mass_1 = core_1->calc_virt_mass(&anchor_1->core_pos, NULL);
	spring_values_factor = (min_mass_0 * min_mass_1) / (min_mass_0 + min_mass_1);
    }

    // set mass adapted spring constant
    this->spring_constant = spring_templ->spring_constant * spring_values_factor;
    this->spring_damp = spring_templ->spring_damp * spring_values_factor;
    this->rel_pos_damp = spring_templ->rel_pos_damp * spring_values_factor;
	this->spring_force_only_on_stretch = spring_templ->spring_force_only_on_stretch;
}


void IVP_Actuator_Spring::set_len(IVP_DOUBLE len){
    if (spring_len == len) return;
    spring_len = len;
    this->ensure_actuator_in_simulation();
}

void IVP_Actuator_Spring::set_break_max_len(IVP_DOUBLE len){
    if (break_max_len == len) return;
    break_max_len = len;
    this->ensure_actuator_in_simulation();
}


void IVP_Actuator_Spring::set_constant(IVP_DOUBLE c){
    spring_constant = c * spring_values_factor;
    this->ensure_actuator_in_simulation();
}

void IVP_Actuator_Spring::set_damp(IVP_DOUBLE c){
    spring_damp = c * spring_values_factor;
    this->ensure_actuator_in_simulation();
}

void IVP_Actuator_Spring::set_rel_pos_damp(IVP_DOUBLE c){
    rel_pos_damp = c * spring_values_factor;
    this->ensure_actuator_in_simulation();
}


IVP_Actuator_Spring::~IVP_Actuator_Spring()
{
}


void IVP_Actuator_Spring_Active::active_float_changed(IVP_U_Active_Float *af){
    if (af == active_float_spring_len){
	set_len( af->get_float_value());
	return;
    }
    if (af == active_float_spring_constant){
	set_constant( af->get_float_value());
	return;
    }
    if (af == active_float_spring_damp){
	set_damp( af->get_float_value());
	return;
    }
    if (af == active_float_spring_rel_pos_damp){
	set_rel_pos_damp( af->get_float_value());
	return;
    }
    CORE;
}

IVP_Actuator_Spring_Active::IVP_Actuator_Spring_Active(IVP_Environment *env,
						       IVP_Template_Spring *spring_templ)
    : IVP_Actuator_Spring(env,spring_templ, IVP_ACTUATOR_TYPE_SPRING){
    
    active_float_spring_len = spring_templ->active_float_spring_len;
    active_float_spring_constant = spring_templ->active_float_spring_constant;
    active_float_spring_damp = spring_templ->active_float_spring_damp;
    active_float_spring_rel_pos_damp = spring_templ->active_float_spring_rel_pos_damp;

    if (active_float_spring_len){
	active_float_spring_len->add_dependency(this);
	spring_len = active_float_spring_len->get_float_value();
    }
    if (active_float_spring_constant){
	active_float_spring_constant->add_dependency(this);
	spring_constant = active_float_spring_constant->get_float_value();
    }
    if (active_float_spring_damp){
	active_float_spring_damp->add_dependency(this);
	spring_damp = active_float_spring_damp->get_float_value();
    }
    if (active_float_spring_rel_pos_damp){
	active_float_spring_rel_pos_damp->add_dependency(this);
	rel_pos_damp = active_float_spring_rel_pos_damp->get_float_value();
    }
}


IVP_Actuator_Spring_Active::~IVP_Actuator_Spring_Active(){
    if (active_float_spring_len){
	active_float_spring_len->remove_dependency(this);
    }
    if (active_float_spring_constant){
	active_float_spring_constant->remove_dependency(this);
    }
    if (active_float_spring_damp){
	active_float_spring_damp->remove_dependency(this);
    }
    if (active_float_spring_rel_pos_damp){
	active_float_spring_rel_pos_damp->remove_dependency(this);
    }
}



void IVP_Actuator_Spring::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> * /*core_list*/) {

  
	IVP_Anchor *anch0 = this->get_actuator_anchor(0);
	IVP_Anchor *anch1 = this->get_actuator_anchor(1);
	IVP_Core *pc0 = anch0->l_anchor_object->get_core();
	IVP_Core *pc1 = anch1->l_anchor_object->get_core();

   IVP_U_Float_Point dir_ws;
	IVP_U_Point pos0_ws;
	IVP_U_Point pos1_ws;

	pc0->get_m_world_f_core_PSI()->vmult4( &anch0->core_pos, &pos0_ws);
	pc1->get_m_world_f_core_PSI()->vmult4( &anch1->core_pos, &pos1_ws);
	dir_ws.subtract(&pos0_ws,&pos1_ws); //force direction world coords
	
	IVP_FLOAT dlen = dir_ws.real_length_plus_normize();

	if (dlen < P_FLOAT_EPS)
	    return;
	
	if (dlen > break_max_len)
	{
	    switch(max_len_exceed_type)
		{
	    case IVP_SFE_BREAK:
			fire_event_spring_broken();
			P_DELETE_THIS(this);
			return;
	    default:
			;
	    }
	}

	if ( !spring_force_only_on_stretch || dlen > this->spring_len )
	{
		IVP_DOUBLE force = (dlen - this->spring_len) * this->spring_constant;

		// spring damping	
		IVP_U_Float_Point relative_world_speed;

		IVP_Core::get_diff_surface_speed_of_two_cores(pc1, pc0,&anch1->core_pos,&anch0->core_pos,&relative_world_speed);
		
		IVP_DOUBLE damp_speed = dir_ws.dot_product(&relative_world_speed); //positive means closing each other

		IVP_DOUBLE damp_s = this->spring_damp;
		IVP_DOUBLE damp_m = this->rel_pos_damp;    

		force = force - damp_s*damp_speed;

		IVP_U_Float_Point force_vector_ws;	// the force 
		force_vector_ws.set_multiple(&dir_ws, force * es->delta_time);

		{ // manhattan damping
			IVP_DOUBLE manhattan_factor = - es->delta_time * damp_m;
			force_vector_ws.add_multiple(&relative_world_speed,manhattan_factor); //manhattan damping
		}

		if (pc1->movement_state<IVP_MT_NOT_SIM)
		{ 
			pc1->async_push_core_ws(&pos1_ws,&force_vector_ws);
      	}   
		if (pc0->movement_state<IVP_MT_NOT_SIM)
		{ 
			force_vector_ws.mult(-1);
			pc0->async_push_core_ws(&pos0_ws,&force_vector_ws);
      	}	
	}
}

/////////////// Suspension ///////////////////////////////
/////////////// Suspension ///////////////////////////////
/////////////// Suspension ///////////////////////////////
/////////////// Suspension ///////////////////////////////

IVP_Actuator_Suspension::IVP_Actuator_Suspension(IVP_Environment *env,
					 IVP_Template_Suspension *templ)
                                       : IVP_Actuator_Spring(env, templ, IVP_ACTUATOR_TYPE_SUSPENSION)
{
    // set mass adapted spring constant
    this->spring_dampening_compression = templ->spring_dampening_compression * spring_values_factor;
    this->max_body_force = templ->max_body_force;
}

void IVP_Actuator_Suspension::set_spring_damp_compression(IVP_FLOAT comp)
{
    if (spring_dampening_compression == comp) return;
    spring_dampening_compression = comp * spring_values_factor;
    this->ensure_actuator_in_simulation();
}

void IVP_Actuator_Suspension::set_max_body_force(IVP_FLOAT max_force)
{
    if (max_body_force == max_force) return;
    max_body_force = max_force;
    this->ensure_actuator_in_simulation();
}

IVP_Actuator_Suspension::~IVP_Actuator_Suspension()
{
}


void IVP_Actuator_Suspension::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> * /*core_list*/) {

    IVP_Actuator_Suspension *my_spring=this;
  
	IVP_Anchor *anch0,*anch1;
	anch0= my_spring->get_actuator_anchor(0);
	anch1= my_spring->get_actuator_anchor(1);
	IVP_Core *pc0 = anch0->l_anchor_object->get_core();
	IVP_Core *pc1 = anch1->l_anchor_object->get_core();

	IVP_U_Point pos0_ws;
	IVP_U_Point pos1_ws;

        IVP_U_Float_Point dir_ws;
	pc0->get_m_world_f_core_PSI()->vmult4( &anch0->core_pos, &pos0_ws);
	pc1->get_m_world_f_core_PSI()->vmult4( &anch1->core_pos, &pos1_ws);
	dir_ws.subtract(&pos0_ws,&pos1_ws); //force direction world coords
	
	IVP_FLOAT dlen = dir_ws.real_length_plus_normize();
	if (dlen < P_FLOAT_EPS){
	    return;
	}
	
	IVP_DOUBLE delta_len = dlen - my_spring->spring_len;
	// linear (standard) spring
	IVP_DOUBLE force = delta_len * my_spring->spring_constant;

	// spring damping	
	IVP_U_Float_Point relative_world_speed;

	IVP_Core::get_diff_surface_speed_of_two_cores(pc1, pc0,&anch1->core_pos,&anch0->core_pos,&relative_world_speed);
	IVP_DOUBLE damp_speed = relative_world_speed.dot_product(&dir_ws); //positive means closing each other

	if (0){ // Version with new IVP_Solver_Core_Reaction
	  IVP_Solver_Core_Reaction tcb;
	  IVP_U_Float_Point direction(&dir_ws);
	  tcb.init_reaction_solver_translation_ws(pc1,pc0, pos0_ws, &direction,0,0);
	  printf("suspension new %f old %f\n", tcb.delta_velocity_ds.k[0], damp_speed);
	}
	/// SPECIAL Suspension treatment
	IVP_DOUBLE damp_s;
	if(damp_speed < 0.0f){
	    damp_s = my_spring->spring_damp;
	}else{
	    // use another factor when spring is compressing
	    damp_s = my_spring->spring_dampening_compression;
	}

	force = (force - damp_s*damp_speed);
	IVP_DOUBLE force_clipped = -force;

	// maximum force for body
	if(force_clipped < -my_spring->max_body_force){
	    // printf("force clipped a: %g %g\n", force_clipped, -my_spring->max_body_force); 
	    force_clipped = -my_spring->max_body_force;
	} else if(force_clipped > my_spring->max_body_force){
	    // printf("force clipped b: %g %g\n", force_clipped, my_spring->max_body_force); 
	    force_clipped = my_spring->max_body_force;
	}
	//	printf("force: %g\n", force); 
	IVP_DOUBLE impulse = force *  es->delta_time;
	IVP_DOUBLE impulse_clipped = force_clipped *  es->delta_time;

	IVP_U_Float_Point impulse_vector_ws;
	impulse_vector_ws.set_multiple(&dir_ws, impulse );
	pc1->async_push_core_ws(&pos1_ws, &impulse_vector_ws);
	
	impulse_vector_ws.set_multiple(&dir_ws, impulse_clipped );
	pc0->async_push_core_ws(&pos0_ws, &impulse_vector_ws);
}

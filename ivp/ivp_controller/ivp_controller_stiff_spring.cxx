// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#include <ivp_physics.hxx>

#ifndef WIN32
#	pragma implementation "ivp_controller_stiff_spring.hxx"
#endif


#include <ivp_solver_core_reaction.hxx>

#include <ivp_actuator.hxx>
#include <ivp_controller_stiff_spring.hxx>


//////////////////////////

IVP_Template_Stiff_Spring::IVP_Template_Stiff_Spring(){
    P_MEM_CLEAR(this);
    this->break_max_len = P_FLOAT_MAX;
}


IVP_Template_Stiff_Spring_Active::IVP_Template_Stiff_Spring_Active(){
    active_float_spring_len = NULL;
    active_float_spring_constant = NULL;
    active_float_spring_damp = NULL;
}

/////////////////// SPRING /////////////////

IVP_CONTROLLER_PRIORITY IVP_Controller_Stiff_Spring::get_controller_priority(){
    return IVP_CP_STIFF_SPRINGS;
}

void IVP_Controller_Stiff_Spring::fire_event_spring_broken(){
    for (int i = listeners_spring_event.len()-1; i>=0; i--){
	IVP_Listener_Stiff_Spring *li = listeners_spring_event.element_at(i);
	li->event_stiff_spring_broken(this);
    }
}

IVP_Controller_Stiff_Spring::IVP_Controller_Stiff_Spring(IVP_Environment *env, IVP_Template_Stiff_Spring *spring_templ )
                                       : IVP_Actuator_Two_Point(env, spring_templ, IVP_ACTUATOR_TYPE_STIFF_SPRING){
    this->spring_len = spring_templ->spring_len;
    this->break_max_len = spring_templ->break_max_len;
    this->max_len_exceed_type = spring_templ->max_len_exceed_type;
    this->l_environment=env;
    
    // set mass adapted spring constant
    this->spring_constant = spring_templ->spring_constant;
    this->spring_damp = spring_templ->spring_damp;
}


void IVP_Controller_Stiff_Spring::set_len(IVP_DOUBLE len){
    if (spring_len == len) return;
    spring_len = len;
    this->ensure_actuator_in_simulation();
}

void IVP_Controller_Stiff_Spring::set_break_max_len(IVP_DOUBLE len){
    if (break_max_len == len) return;
    break_max_len = len;
    this->ensure_actuator_in_simulation();
}


void IVP_Controller_Stiff_Spring::set_constant(IVP_DOUBLE c){
    spring_constant = c;
    this->ensure_actuator_in_simulation();
}

void IVP_Controller_Stiff_Spring::set_damp(IVP_DOUBLE c){
    spring_damp = c;
    this->ensure_actuator_in_simulation();
}


IVP_Controller_Stiff_Spring::~IVP_Controller_Stiff_Spring()
{
}


void IVP_Controller_Stiff_Spring_Active::active_float_changed(IVP_U_Active_Float *af){
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
    CORE;
}

IVP_Controller_Stiff_Spring_Active::IVP_Controller_Stiff_Spring_Active(IVP_Environment *env,
						       IVP_Template_Stiff_Spring_Active *spring_templ)
    : IVP_Controller_Stiff_Spring(env,spring_templ){
    
    active_float_spring_len = spring_templ->active_float_spring_len;
    active_float_spring_constant = spring_templ->active_float_spring_constant;
    active_float_spring_damp = spring_templ->active_float_spring_damp;

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
}


IVP_Controller_Stiff_Spring_Active::~IVP_Controller_Stiff_Spring_Active(){
    if (active_float_spring_len){
	active_float_spring_len->remove_dependency(this);
    }
    if (active_float_spring_constant){
	active_float_spring_constant->remove_dependency(this);
    }
    if (active_float_spring_damp){
	active_float_spring_damp->remove_dependency(this);
    }
}



void IVP_Controller_Stiff_Spring::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> * /*core_list*/) {

    /* first step: collect all data */
    IVP_U_Float_Point dir_ws;
    IVP_U_Point pos0_ws;
    IVP_DOUBLE dlen;
    IVP_Anchor *anch0 = this->get_actuator_anchor(0);
    IVP_Anchor *anch1 = this->get_actuator_anchor(1);
    IVP_Core *pc0 = anch0->l_anchor_object->get_core();
    IVP_Core *pc1 = anch1->l_anchor_object->get_core();
    {
	IVP_U_Point pos1_ws;

	pc0->get_m_world_f_core_PSI()->vmult4( &anch0->core_pos, &pos0_ws);
	pc1->get_m_world_f_core_PSI()->vmult4( &anch1->core_pos, &pos1_ws);
	dir_ws.subtract(&pos0_ws,&pos1_ws); //force direction world coords

	dlen = dir_ws.real_length_plus_normize();
	if (dlen < P_FLOAT_EPS){
	    return;
	}
	
	if (dlen > break_max_len){
	    switch(max_len_exceed_type){
	    case IVP_SFE_BREAK:
		fire_event_spring_broken();
		P_DELETE_THIS(this);
		return;
	    default:
		;
	    }
	}
    }
    
    if (pc0->movement_state >= IVP_MT_NOT_SIM)	pc0 = NULL;
    if (pc1->movement_state >= IVP_MT_NOT_SIM)	pc1 = NULL;

    
    /* second step: calc and apply forces */
    IVP_Solver_Core_Reaction scr;

    scr.init_reaction_solver_translation_ws( pc0, pc1, pos0_ws, &dir_ws, NULL, NULL);
    IVP_U_Matrix3 &tpm = scr.m_velocity_ds_f_impulse_ds;
    
    IVP_DOUBLE a = (this->spring_len - dlen) * es->i_delta_time * this->spring_constant -
	scr.delta_velocity_ds.k[0] * ( this->spring_constant +  this->spring_damp * ( 1.0f - this->spring_constant));
    IVP_U_Float_Point impulses;
    impulses.k[0] = a / tpm.get_elem(0,0) ;
    
    scr.exert_impulse_dim1(pc0, pc1, impulses);	
}

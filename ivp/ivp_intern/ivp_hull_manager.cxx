// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef WIN32
#	pragma implementation "ivp_hull_manager_macros.hxx"
#	pragma implementation "ivp_listener_hull.hxx"
#	pragma implementation "ivp_hull_manager.hxx"
#endif

#include <ivp_physics.hxx>

#include <ivu_min_hash.hxx>
#include <ivp_hull_manager.hxx>
#include "ivp_hull_manager_macros.hxx"

// IVP_EXPORT_PRIVATE

IVP_Hull_Manager_Base::IVP_Hull_Manager_Base(): sorted_synapses(8){
    gradient = center_gradient = 0.0f;
    hull_value_last_vpsi = 0.0f;
    hull_center_value_last_vpsi = 0.0f;
    hull_value_next_psi = 0.0f;
    time_of_next_reset = 0;
}

IVP_Hull_Manager_Base::~IVP_Hull_Manager_Base(){
    IVP_ASSERT( ! sorted_synapses.has_elements() );
}

void IVP_Hull_Manager::delete_hull_manager(){
    while ( sorted_synapses.has_elements() ){
	IVP_Listener_Hull *syn = (IVP_Listener_Hull *)sorted_synapses.find_min_elem();
	syn->hull_manager_is_going_to_be_deleted_event((IVP_Hull_Manager *)this);
    }
}

void   IVP_Listener_Hull::hull_manager_is_reset(IVP_FLOAT /*dt*/, IVP_FLOAT /*center_dt*/){
    ;
}

void IVP_Hull_Manager::reset_times(){
    IVP_FLOAT mt = - hull_value_last_vpsi;
    IVP_FLOAT mc = - hull_center_value_last_vpsi;
    IVP_U_Min_List_Enumerator mle(&sorted_synapses);
    
    while ( IVP_U_Min_List_Element *el = mle.get_next_element_header()){
	IVP_Listener_Hull *lh = (IVP_Listener_Hull *)el->element;
	el->value += mt;
	lh->hull_manager_is_reset(mt,mc);
	
    }
    sorted_synapses.min_value += mt;
    hull_value_last_vpsi = 0.0f;
    hull_center_value_last_vpsi = 0.0f;
    hull_value_next_psi += mt;
}

void IVP_Hull_Manager::reset_time(IVP_Time offset){
    last_vpsi_time -= offset;
}

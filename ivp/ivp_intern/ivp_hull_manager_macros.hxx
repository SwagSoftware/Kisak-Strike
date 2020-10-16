// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

IVP_BOOL IVP_Hull_Manager::are_events_in_hull(){
    if ( sorted_synapses.find_min_value() - hull_value_next_psi < 0.0f){ // no other synapses need to be checked
	return IVP_TRUE;
    }else{
	return IVP_FALSE;
    }
}

void IVP_Hull_Manager::check_hull_synapses(){
    // check all synapses

    // eventually throw hull limit exceeded event
    IVP_FLOAT syn_val;
    int maxcnt = 100;
    while ( (syn_val = sorted_synapses.find_min_value() - hull_value_next_psi) < 0.0f){ // no other synapses need to be checked
	// syn hull event is to be fired
	IVP_Listener_Hull *syn = (IVP_Listener_Hull *)sorted_synapses.find_min_elem(); // corresponding synapse
	syn->hull_limit_exceeded_event(this,syn_val); // ATT: syn MUST update/remove itself/its peer from min_hash!
	if (maxcnt-- <0) break;
    }
}

void IVP_Hull_Manager::increase_hull_by_x(IVP_Time t_now, IVP_FLOAT delta_time, IVP_FLOAT gradient_, IVP_FLOAT center_gradient_) {
    IVP_FLOAT dt = t_now - last_vpsi_time;
    hull_value_last_vpsi += gradient * dt;
    hull_center_value_last_vpsi += center_gradient * dt;
    last_vpsi_time = t_now;
    gradient = gradient_ * IVP_HULL_MANAGER_GRADIENT_FACTOR;
    center_gradient = center_gradient_;

    IVP_FLOAT delta = delta_time * gradient;
    hull_value_next_psi = hull_value_last_vpsi + delta;
    IVP_ASSERT(hull_value_next_psi >= 0.0f);
}


// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

class IVP_Real_Object;
class IVP_Listener_Hull;


#ifndef IVP_U_MINLIST_INCLUDED
#     include <ivu_min_list.hxx>
#endif
#ifndef _IVP_LISTENER_HULL_INCLUDED
#	include <ivp_listener_hull.hxx>
#endif

#define IVP_MAX_TIME_WITHOUT_RESET 10.0f  /* seconds */
#define IVP_HULL_MANAGER_GRADIENT_FACTOR 1.00001f  /* high enough that till MAX_TIME no bits are lost */


class IVP_Hull_Manager: public IVP_Hull_Manager_Base {

    void reset_times();		// reset all values to zero, needed to avoid float overflow
public:
    IVP_FLOAT get_current_hull_time() const{ return hull_value_last_vpsi; };
    IVP_FLOAT get_current_center_hull_time() const{ return hull_center_value_last_vpsi; };
    
    IVP_U_Min_List *get_sorted_synapses() { return &sorted_synapses; };

    inline void increase_hull_by_x(IVP_Time t_now, IVP_FLOAT delta_time, IVP_FLOAT gradient, IVP_FLOAT center_gradient_);

    // inline functions, include ivp_hull_manager_macros.hxx
    inline void check_hull_synapses();
    inline IVP_BOOL are_events_in_hull(); // quick check for events in hull
    
    inline void check_for_reset(){
	if ( last_vpsi_time.get_seconds() > time_of_next_reset){
	    reset_times();
	    time_of_next_reset = int(last_vpsi_time.get_seconds() + IVP_MAX_TIME_WITHOUT_RESET);
	}
    }
    
    inline void prefetch0_hull();	// prefetches gradient information
    inline void prefetch0_gradient();   // prefetches the sorted minlist header
    inline void prefetch1_hull();	// prefetches some sorted minlist data
    inline IVP_FLOAT insert_synapse(IVP_Listener_Hull *syn, IVP_Time t_now, IVP_DOUBLE delta_valid_hull_time); // assert delta_valid_hull_time ensures no events till next PSI
    inline IVP_FLOAT insert_lazy_synapse(IVP_Listener_Hull *syn, IVP_Time t_now, IVP_DOUBLE delta_valid_hull_time); // same, but adds some extra time to avoid assert on delta_valid_hull_time
    inline void update_synapse(IVP_Listener_Hull *syn, IVP_Time t_now, IVP_DOUBLE delta_valid_hull_time);
    inline void update_lazy_synapse(IVP_Listener_Hull *syn, IVP_Time t_now, IVP_DOUBLE delta_valid_hull_time);
    inline void remove_synapse(IVP_Listener_Hull *syn);

  IVP_FLOAT get_angular_hull_time(IVP_Time current_time){
     return (current_time - last_vpsi_time) * (gradient - center_gradient) + ( hull_value_last_vpsi - hull_center_value_last_vpsi);
  }
  
    void init_hull_for_simulation(){
    }
    void reset_time(IVP_Time offset);
    void exit_hull_for_simulation(IVP_Time t_now){
	hull_value_last_vpsi += gradient * ( t_now - last_vpsi_time);
	hull_center_value_last_vpsi += center_gradient * ( t_now - last_vpsi_time);
	gradient = 0.0f;
	center_gradient = 0.0f;
	reset_times();
    }

  void jump_add_hull( IVP_FLOAT delta_x, IVP_FLOAT delta_center_x){
    hull_value_last_vpsi = hull_value_next_psi = hull_value_next_psi + delta_x;
    IVP_ASSERT(hull_value_next_psi >= 0.0f);
    hull_center_value_last_vpsi += delta_center_x;
    gradient = 0.0f;
    center_gradient = 0.0f;
  }
    
protected:  
    IVP_Hull_Manager();
public:
    void delete_hull_manager();
};


IVP_FLOAT IVP_Hull_Manager::insert_synapse(IVP_Listener_Hull *syn, IVP_Time t_now, IVP_DOUBLE delta_valid_hull_time)
{
    IVP_FLOAT dt = t_now - last_vpsi_time;
    IVP_FLOAT val = hull_value_last_vpsi + gradient * dt;
    syn->minlist_index = sorted_synapses.add((void *)syn, val + delta_valid_hull_time);

    //sorted_synapses.check();
    return (hull_value_last_vpsi - hull_center_value_last_vpsi) + (gradient - center_gradient) * dt;
}

IVP_FLOAT IVP_Hull_Manager::insert_lazy_synapse(IVP_Listener_Hull *syn, IVP_Time /*t_now*/, IVP_DOUBLE delta_valid_hull_time)
{
    IVP_FLOAT val = hull_value_next_psi;
    syn->minlist_index = sorted_synapses.add((void *)syn, val + delta_valid_hull_time);
    return hull_center_value_last_vpsi;
}

inline void IVP_Hull_Manager::prefetch0_gradient(){
    IVP_IF_PREFETCH_ENABLED(IVP_TRUE){	
	IVP_PREFETCH_BLOCK(this, sizeof(IVP_Hull_Manager_Base_Gradient));
    }
}

inline void IVP_Hull_Manager::prefetch0_hull(){
    IVP_IF_PREFETCH_ENABLED(IVP_TRUE){	
	IVP_PREFETCH_BLOCK(this, sizeof(IVP_Hull_Manager_Base));
    }
}

inline void IVP_Hull_Manager::prefetch1_hull(){
    sorted_synapses.prefetch1_minlist();
}

void IVP_Hull_Manager::remove_synapse(IVP_Listener_Hull *syn)
{
    sorted_synapses.remove_minlist_elem(syn->minlist_index);
}

void IVP_Hull_Manager::update_synapse(IVP_Listener_Hull *syn, IVP_Time t_now,  IVP_DOUBLE delta_valid_hull_time)
{
    sorted_synapses.remove_minlist_elem(syn->minlist_index);
    IVP_FLOAT dt = t_now - this->last_vpsi_time;
    IVP_FLOAT val = hull_value_last_vpsi + gradient * dt;
    syn->minlist_index = sorted_synapses.add((void *)syn, val + delta_valid_hull_time);
}

void IVP_Hull_Manager::update_lazy_synapse(IVP_Listener_Hull *syn, IVP_Time /*t_now*/,  IVP_DOUBLE delta_valid_hull_time)
{
    sorted_synapses.remove_minlist_elem(syn->minlist_index);
    IVP_FLOAT val = hull_value_next_psi;
    syn->minlist_index = sorted_synapses.add((void *)syn, val + delta_valid_hull_time);
}

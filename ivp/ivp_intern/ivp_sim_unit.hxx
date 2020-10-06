// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef WIN32
#	pragma interface
#endif

#define IVP_SIM_SLOTS_NUM 100

//#define IVP_DISABLE_FREEZING /* disable freezing of objects */

// #+# put a default in controllers dependent
class IVP_Sim_Unit_Controller_Core_List {
public:
    IVP_Controller *l_controller;
    IVP_Vector_of_Cores_2 cores_controlled_by;
};



//different simulation rates: recalculate rate after fusion (like in try_to_generate_managed_friction)
//                            and in sim_unit_revive_for_simulation
class IVP_Simulation_Unit {
    IVP_Movement_Type sim_unit_movement_type:8;
public:
    IVP_BOOL union_find_needed_for_sim_unit:2;
    IVP_BOOL sim_unit_has_fast_objects:2; //fast moving objects: some optimizations like: no energy controll ...
    IVP_BOOL sim_unit_just_slowed_down:2; //changing from fast moving to slowly moving
    
    IVP_Simulation_Unit *prev_sim_unit;
    IVP_Simulation_Unit *next_sim_unit;
  
    IVP_Vector_of_Cores_2 sim_unit_cores;
    IVP_U_Vector<IVP_Sim_Unit_Controller_Core_List> controller_cores;

    IVP_Simulation_Unit();
    ~IVP_Simulation_Unit();
  
    void do_sim_unit_union_find();
    IVP_BOOL sim_unit_calc_movement_state(IVP_Environment *env); //return TRUE when Simulation_Unit was removed from Simulation
    void sim_unit_clear_movement_check_values();
    void sim_unit_revive_for_simulation(IVP_Environment *env); //can handle mixture of simulated objs and not simulated objs
    inline void sim_unit_ensure_in_simulation();                      //can not ...
    void sim_unit_ensure_cores_movement();    //reset_freeze_check_values
    IVP_Core *sim_unit_union_find_test(); //problem: quadratic search, because every controller gives list of associated cores (but this list depends on core)
    void sim_unit_calc_redundants();
    IVP_BOOL controller_is_known_to_sim_unit( IVP_Controller *my_controller );
    void add_controlled_core_for_controller( IVP_Controller *cntrl, IVP_Core *my_core );
    void add_controller_unit_sim( IVP_Controller *new_cntrl );
    void split_sim_unit(IVP_Core *split_father);
    void perform_test_and_split();
    void clean_sim_unit();
    void throw_cores_into_my_sim_unit(IVP_Simulation_Unit *second_unit);
    void fusion_simulation_unities(IVP_Simulation_Unit *second_unit);
    void rem_sim_unit_controller( IVP_Controller *rem_controller );
    void sim_unit_remove_core( IVP_Core *del_core );
    void sim_unit_sort_controllers();
    void add_controller_of_core( IVP_Core *my_core, IVP_Controller *cntrl );
    void remove_controller_of_core( IVP_Core *my_core, IVP_Controller *cntrl );
    IVP_Movement_Type get_unit_movement_type() { return (IVP_Movement_Type)sim_unit_movement_type; };
    void set_unit_movement_type(IVP_Movement_Type mm_type) { sim_unit_movement_type=mm_type; };
  
    void add_sim_unit_core( IVP_Core *add_core );
    void rem_sim_unit_core( IVP_Core *del_core );
  //IVP_Controller *get_first_sim_unit_controller();
  //IVP_Controller *get_next_sim_unit_controller();

    static inline void prefetch0_init_moving_core_for_psi(IVP_Core *core);
    static inline void init_moving_core_for_psi(IVP_Core *core, const IVP_Time &current_time);

    
    int get_pos_of_controller( IVP_Controller *contr );
    void sim_unit_exchange_controllers(int first, int second);

    void reset_time( IVP_Time offset);
    void simulate_single_sim_unit_psi(class IVP_Event_Sim *es, IVP_U_Vector<IVP_Core> *touched_cores_out);
    
//    inline void prefetch0_simulate_single_sim_unit_psi();   // first level prefetch
//    inline void prefetch1_simulate_single_sim_unit_psi();   // second level prefetch
//    inline void prefetch2_simulate_single_sim_unit_psi();   // second level prefetch
#ifdef DEBUG  
    void sim_unit_debug_consistency();
    void sim_unit_debug_out();
#endif    
    IVP_BOOL sim_unit_core_exists(IVP_Core *core);
};

class IVP_Sim_Units_Manager {
    friend class IVP_Simulation_Unit;
public:
    IVP_Environment *l_environment;
    IVP_Time nb;
    IVP_Time bt;
  
    IVP_Simulation_Unit *sim_units_slots[ IVP_SIM_SLOTS_NUM ];
    IVP_Simulation_Unit *still_slot;
  
    IVP_Sim_Units_Manager(IVP_Environment *env);
    void add_sim_unit_to_manager(IVP_Simulation_Unit *sim_u);
    void rem_sim_unit_from_manager(IVP_Simulation_Unit *sim_u);
    void add_unit_to_slot(IVP_Simulation_Unit *sim_u,IVP_Simulation_Unit **slot);
    void rem_unit_from_slot(IVP_Simulation_Unit *sim_u,IVP_Simulation_Unit **slot);

    void simulate_sim_units_psi(IVP_Environment *env, IVP_U_Vector<IVP_Core> *touched_cores_out);

    void reset_time( IVP_Time offset );
};

void IVP_Simulation_Unit::sim_unit_ensure_in_simulation() {
    if(sim_unit_movement_type >= IVP_MT_NOT_SIM) {
	IVP_ASSERT(sim_unit_cores.len() > 0);
	this->sim_unit_revive_for_simulation(sim_unit_cores.element_at(0)->environment);
    } else {
	sim_unit_ensure_cores_movement();
    }
}

inline void IVP_Simulation_Unit::prefetch0_init_moving_core_for_psi(IVP_Core *core){
    IVP_IF_PREFETCH_ENABLED(IVP_TRUE){
	IVP_PREFETCH_BLOCK( (char *)core + sizeof(IVP_Core_Fast_Static), sizeof(IVP_Core_Fast_PSI)- sizeof(IVP_Core_Fast_Static));
    }
}

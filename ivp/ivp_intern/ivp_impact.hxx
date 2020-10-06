// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


class IVP_Core;
class IVP_Friction_Core_Pair;
class IVP_Friction_System;
class IVP_Mindist;
class IVP_Contact_Point;

class IVP_Impact_System {
    IVP_Environment *l_environment;
    int sum_of_pushes;
    IVP_U_Vector<IVP_Core> i_s_pushed_cores;
    IVP_U_Vector<IVP_Core> i_s_known_cores;    
    IVP_U_Vector<IVP_Friction_Core_Pair> i_s_pairs;
    IVP_Friction_System *associated_fs_system;
    
    void impact_system_check_start_pair(IVP_Friction_Core_Pair *start_pair,IVP_Contact_Point *mdfr);
    void synchronize_core_for_impact_system(IVP_Core *new_core);
    void invalidate_impact_mindists(IVP_Core *my_core);
    inline void add_pushed_core_to_impact_system(IVP_Core *new_core);
    inline void add_known_core_to_impact_system(IVP_Core *new_core);
    void add_pair_to_impact_system(IVP_Friction_Core_Pair *new_pair);
    inline IVP_BOOL core_was_already_pushed_in_system(IVP_Core *test_core);
    inline IVP_BOOL core_is_already_known_to_system(IVP_Core *test_core);
    IVP_BOOL pair_is_already_in_system(IVP_Friction_Core_Pair *test_pair); //place info in pair to avoid linear search
    void add_pushed_core_with_pairs_except(IVP_Core *new_core,IVP_Friction_Core_Pair *start_pair);
    IVP_BOOL test_loop_all_pairs();
    void recalc_all_affected_cores();

    void debug_check_all_dists_at_end();
public:
    static int sum_sys_impacts;
    
    void init_and_solve_impact_system(IVP_Mindist *mindist, IVP_Friction_System *fs_system,IVP_Friction_Core_Pair *start_pair,IVP_Contact_Point *start_mdf);
};


void IVP_Impact_System::add_known_core_to_impact_system(IVP_Core *new_core){
    //IVP_ASSERT(new_core->tmp.old_sync_info==NULL);
    i_s_known_cores.add(new_core);
}
void IVP_Impact_System::add_pushed_core_to_impact_system(IVP_Core *new_core)
{
    i_s_pushed_cores.add(new_core);
    IVP_ASSERT(new_core->tmp_null.old_sync_info->was_pushed_during_i_s==IVP_FALSE);
    new_core->tmp_null.old_sync_info->was_pushed_during_i_s=IVP_TRUE;
}

IVP_BOOL IVP_Impact_System::core_was_already_pushed_in_system(IVP_Core *test_core) {
    return test_core->tmp_null.old_sync_info->was_pushed_during_i_s;
}

IVP_BOOL IVP_Impact_System::core_is_already_known_to_system(IVP_Core *test_core) {
    void *p=test_core->tmp_null.old_sync_info;
    return (IVP_BOOL)(p != 0);
}


class IVP_Impact_Solver {
    friend class IVP_Impact_Solver_Long_Term;
    
    IVP_FLOAT rescue_speed_impact_solver;
    IVP_DOUBLE virt_mass[2];
    IVP_BOOL delaying_is_allowed;
    int impact_reversed; //objects are no longer closing, but flying away
    int turnaround_next_time; //lowest energy point is traversed (but because of calculation faults objects may still be closing!)
    //IVP_DOUBLE percent_deformation; //for each impact step the percent energy that is reversible deformation
    IVP_DOUBLE energy_deformation;  //whole reversible deformation energy stored so far
    IVP_U_Matrix *m_world_f_core[2];
    IVP_U_Float_Point rot_speed[2]; //rotation speed (changes with several impacts)
    IVP_U_Float_Point trans_speed[2]; //center speed (changes with several impacts)
    IVP_U_Float_Point rot_speed_change[2]; //change within one push
    IVP_U_Float_Point trans_speed_change[2]; 
    IVP_U_Float_Point relative_world_speed; //seen from object 0 (that is virtually still)
    IVP_U_Float_Point world_push_direction; //normized relative_world_speed and changed according to friction (which makes vector smaller)
    IVP_U_Float_Point integral_pushes_world; //sum over pushes done  

    void do_rescue_push(IVP_U_Float_Point *push_dir,IVP_BOOL panic_mode);
    void get_relative_speed_vector();
    IVP_DOUBLE get_total_energy();
    IVP_DOUBLE estimate_push_impulse();
    void confirm_impact(int core_nr);  //change values in object cores
    void undo_push();
    void do_push(IVP_DOUBLE impact);
    void get_world_push_direction();
    void get_world_push_direction_two_friction(IVP_DOUBLE angle);
    void calc_virt_masses_impact_solver(const IVP_U_Float_Point *world_direction_normal);
    void do_push_on_core(IVP_U_Float_Point *push_vec_world,int num_core);    
    void clear_change_values_cores();    
    void delay_of_impact(int delayed_core_nr);
    void delay_decision(IVP_Core *pushed_cores[2]);
    void get_cos_sin_for_impact(IVP_FLOAT friction_val,IVP_FLOAT percent_energy_conservation,IVP_FLOAT *cos_val,IVP_FLOAT *sin_val);
public:
//special case two friction values
    void get_world_direction_second_friction(IVP_Contact_Point *cp);
    IVP_BOOL two_friction_values;
    IVP_FLOAT sin_second_friction;
    //IVP_FLOAT first_friction_value;
    //IVP_FLOAT second_friction_value;
    //IVP_U_Point world_direction_first_friction; //normized, orthogonal to surface normal
    IVP_U_Float_Point world_direction_second_friction; //normized, orthogonal to first_friction and surface_normal
    
//from long term    
    IVP_Core *core[2]; // == 0 for immoveables, represents a surface and a point
    IVP_U_Float_Point *obj_point[2]; //impact position in object coordinates
    IVP_FLOAT percent_energy_conservation; // 1.0f subtract energy destroyed
    IVP_FLOAT cos_friction;  // angle how far friction is valid seen form surface normal 90 degree 
    IVP_FLOAT sin_friction; // sin of angle
    IVP_U_Float_Point *surf_normal;
    IVP_U_Float_Point *speed; //return value for IVP_Contact_Situation
    
    void do_impact(IVP_Core *pushed_cores[2],IVP_BOOL allow_delaying,int pushes_while_system,IVP_FLOAT rescue_speed_val); // comunicates what cores where pushed
    void do_single_impact(IVP_Core *pushed_cores[2],IVP_BOOL allow_delaying);
};













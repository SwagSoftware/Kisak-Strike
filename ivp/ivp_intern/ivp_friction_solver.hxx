// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#define IVP_DESCENDING_SPEEDUP 20.0f

class IVP_U_Memory;

class IVP_Vector_of_Contact_Info_512:public  IVP_U_Vector<IVP_Impact_Solver_Long_Term> {
    IVP_Impact_Solver_Long_Term *elem_buffer[512];
public:
    IVP_Vector_of_Contact_Info_512(): IVP_U_Vector<IVP_Impact_Solver_Long_Term>( (void **)&elem_buffer[0],512 ){;};
};

// temporarily used class, no long term values 
class IVP_Friction_Solver {
public:
    //IVP_U_Memory mem_friction;
    IVP_Great_Matrix_Many_Zero dist_change_mat; /* matrix of how change of one dist inflicts others. Matrix line i, column j means:
						 * how is velocity of distance i changed, when j is changed
						 */
    IVP_DOUBLE correct_x_factor;
    IVP_Environment *l_environment;
    const IVP_Event_Sim *es;
    
    IVP_Vector_of_Contact_Info_512 contact_info_vector; 

    int gauss_succed; //for perf test

    
    // some simple functions (super fast and inline)
    static inline IVP_DOUBLE get_closing_speed_core_i(const IVP_Impact_Solver_Long_Term *,int core_index, const IVP_U_Float_Point *rot_sped_cs, IVP_U_Float_Point *speed_ws);  // based on current core speed
    static inline IVP_DOUBLE get_inv_virtual_mass(const IVP_Impact_Solver_Long_Term *i){return i->inv_virtual_mass;};  // based on core
    static inline void apply_impulse( const IVP_Impact_Solver_Long_Term *, IVP_DOUBLE impulse);
    static inline void async_apply_impulse( const IVP_Impact_Solver_Long_Term *, IVP_DOUBLE impulse);
    static inline void test_push_core_i( const IVP_Impact_Solver_Long_Term *, int core_index, IVP_U_Float_Point *rot_change, IVP_U_Float_Point *speed_change, IVP_FLOAT factor);


    // easing 
    static void ease_friction_pair(IVP_Friction_Core_Pair *my_pair,IVP_U_Memory *my_mem);    
    static void ease_test_two_mindists(IVP_Contact_Point *dist0,IVP_Contact_Point *dist1,IVP_U_Float_Point *world_surf_normal);
    static void ease_two_mindists(IVP_Contact_Point *dist0,IVP_Contact_Point *dist1, IVP_U_Float_Point *diff_vec0,IVP_U_Float_Point *diff_vec1,IVP_DOUBLE ease_factor);


    // setup
    static inline IVP_DOUBLE calc_desired_gap_speed(IVP_DOUBLE closing_speed,IVP_DOUBLE gap_diff,IVP_FLOAT speedup_factor);
    void setup_coords_mindists(IVP_Friction_System *fs);
    inline void calc_distance_matrix_column(int current_contact_point_index,IVP_Core *core_to_test,IVP_Friction_Info_For_Core *fr_info,IVP_U_Float_Point *rotation_vec,IVP_U_Float_Point *translation_vec);
    int calc_solver_PSI(IVP_Friction_System *fs,int *original_pos_of_active);

    // gauss
    IVP_RETURN_TYPE test_gauss_solution_suggestion(IVP_DOUBLE *push_results,int *active_is_at_pos,int total_actives,IVP_U_Memory *my_mem);

    // penalty method
    IVP_FLOAT do_penalty_step( IVP_FLOAT *impulses, IVP_FLOAT *pretension, IVP_FLOAT force_factor, IVP_FLOAT damp_factor); // returns quad_sum_desired_speed
    void do_penalty_method(IVP_Friction_System *fs);
    
    // barraf
    void solve_linear_equation_and_push(IVP_Friction_System *fs,int *active_is_at_pos,int total_actives,IVP_U_Memory *my_mem);
    void complex_failed(IVP_Friction_System *fs);
    int do_resulting_pushes(IVP_Friction_System *my_friction_system);
    void normize_constraint_equ();
    void factor_result_vec();

    // debug
    void debug_distance_after_push(int counter);
    void print_dist_velocity(IVP_Friction_System *fs); 

    // etc
    void do_inactives_pushes(IVP_Friction_System *fs);
    
    IVP_Friction_Solver(IVP_Friction_System *fs,const IVP_Event_Sim *);
};



IVP_DOUBLE IVP_Friction_Solver::get_closing_speed_core_i(const IVP_Impact_Solver_Long_Term *info,
							 int core_index_i,
							 const IVP_U_Float_Point *rot_sped_cs, IVP_U_Float_Point *speed_ws){
    IVP_DOUBLE result = speed_ws->dot_product( & info->surf_normal ) +
	rot_sped_cs->dot_product( &info->contact_cross_nomal_cs[core_index_i] );
    return result;
}


void IVP_Friction_Solver::apply_impulse( const IVP_Impact_Solver_Long_Term *info, IVP_DOUBLE impulse){
    IVP_Core *c0;
    if ( (c0 = info->contact_core[0])){
	IVP_U_Float_Point hp; hp.set_pairwise_mult( &info->contact_cross_nomal_cs[0], c0->get_inv_rot_inertia());
	c0->rot_speed.add_multiple( &hp, -impulse);
	c0->speed.add_multiple( &info->surf_normal, -impulse * c0->get_inv_mass());
    }

    IVP_Core *c1;
    if ((c1 = info->contact_core[1])){
	IVP_U_Float_Point hp; hp.set_pairwise_mult( &info->contact_cross_nomal_cs[1], c1->get_inv_rot_inertia());
	c1->rot_speed.add_multiple( &hp, impulse);
	c1->speed.add_multiple( &info->surf_normal, impulse * c1->get_inv_mass());
    }
}

void IVP_Friction_Solver::async_apply_impulse( const IVP_Impact_Solver_Long_Term *info, IVP_DOUBLE impulse){
    IVP_Core *c0;
    if ( (c0 = info->contact_core[0])){
	IVP_U_Float_Point hp; hp.set_pairwise_mult( &info->contact_cross_nomal_cs[0], c0->get_inv_rot_inertia());
	c0->rot_speed_change.add_multiple( &hp, -impulse);
	c0->speed_change.add_multiple( &info->surf_normal, -impulse * c0->get_inv_mass());
    }

    IVP_Core *c1;
    if ((c1 = info->contact_core[1])){
	IVP_U_Float_Point hp; hp.set_pairwise_mult( &info->contact_cross_nomal_cs[1], c1->get_inv_rot_inertia());
	c1->rot_speed_change.add_multiple( &hp, impulse);
	c1->speed_change.add_multiple( &info->surf_normal, impulse * c1->get_inv_mass());
    }
}

void IVP_Friction_Solver::test_push_core_i( const IVP_Impact_Solver_Long_Term *info, int core_index,
					    IVP_U_Float_Point *rot_change, IVP_U_Float_Point *speed_change, IVP_FLOAT factor){
    IVP_Core *c0 = info->contact_core[core_index];
    rot_change->set_pairwise_mult( &info->contact_cross_nomal_cs[core_index], c0->get_inv_rot_inertia());
    if (factor < 0.0f){
	rot_change->set_negative( rot_change );
    }
    speed_change->set_multiple( &info->surf_normal, c0->get_inv_mass() * factor);
}


IVP_DOUBLE IVP_Friction_Solver::calc_desired_gap_speed(IVP_DOUBLE closing_speed,IVP_DOUBLE gap_diff,IVP_FLOAT speedup_factor) {
    IVP_DOUBLE use_delay = speedup_factor;
    if(gap_diff < 0.0f) {
	use_delay *= IVP_DESCENDING_SPEEDUP;
    }
    return gap_diff * use_delay + closing_speed;
};

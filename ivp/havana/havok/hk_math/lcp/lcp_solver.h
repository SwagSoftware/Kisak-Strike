#ifndef HK_LCP_SOLVER
#define HK_LCP_SOLVER

class hk_LCP_Input {
public:
	hk_Dense_Matrix *m_A; //aligned Matrix
	//Warning: at the moment m_A is interpreted as row-wise organized and simply pointer to memory is taken
	//         but actually m_A is column-wise organized. Doesn't matter because matrix is symetric, but ...
	hk_real		*m_b; //aligned Vector
	//m_A and m_b are sorted in a way first the m_n_equations occur and then the m_n_actives_at_start

	//int				m_row_count;
	int				m_n_variables;
	int				m_n_equations;
	int				m_n_actives_at_start; //includes already equations
	// temporary memory pool for solver
};

class hk_LCP_Temp {
public:
	//warning: hk_incrlu_real type has to be greater or equal hk_real and hk_gauss_real, because memory is shared

	hk_incrlu_real *m_L_matrix; //aligned Matrix
	hk_incrlu_real *m_U_matrix; //aligned Matrix	
	hk_incrlu_real *m_inout_vec; //aligned vector
	hk_incrlu_real *m_temp_vec; //aligned vector

	int *m_actives_inactives_ignored; //aligned vector 
	hk_real *m_accel;       //aligned vector
	hk_real *m_delta_accel; //aligned vector
	hk_real *m_delta_f;     //aligned vector
};

class hk_LCP_Output {
public:
	hk_real			*m_result;
	hk_bool			*m_active; 
};


//this algorithm solves unilateral equation m_A * m_result >= m_b
// m_result has >= 0.0 and if m_result[i] > 0.0 than ith equation is '==' and not '>' 
// vector m_active gives information about what components in m_result are > 0.0

// Warning: solver uses three different types of floating points (hk_real, hk_incrlu_real, hk_gauss_real)
//          if alignment (right now 16Bytes for all three) differs, algorithm will crash !!!
class hk_LCP_Solver {
    int m_max_steps_lcp;
    hk_LCP_Input *input_struct;
    hk_real *full_A; //aligned matrix, points to input
    hk_real *full_b; //aligned vector, points to input
    hk_real *full_x; //aligned vector, points to output
    hk_real *temp;   //aligned vector
    hk_real *delta_x; //aligned vector, delta of x per step
    hk_real *accel; //aligned vector, is A*x-b must be greater equal 0.0
    hk_real *delta_accel; //aligned vector

    int *actives_inactives_ignored;
    int n_variables;
    int n_equations;
    int aligned_size;
    int r_actives;
    int aligned_sub_size;
    int ignored_pos;
    int debug_lcs;
    int debug_no_lu_count;
    int first_permute_index;
    int second_permute_index;
    int first_permute_ignored;
    int second_permute_ignored;
    
    int sub_solver_status; //0 is ok, 1 is not usable
    hk_Incr_LU_Matrix lu_sub_solver; //two aligned NxN matrices and 4 aligned vectors

    //IVP_Great_Matrix_Many_Zero sub_solver_mat;
    hk_Gauss_Elm_Solver m_gauss_sub_solver;
    hk_Gauss_Elm_Input  m_gauss_inp; //aligned matrix,NxN + aligned vector
    hk_Gauss_Elm_Output m_gauss_out; //aligned vector
    
    void decrement_sub_solver( int sub_pos );
    void increment_sub_solver();
    void exchange_activ_inactiv( int change_var );
    void exchange_lcs_variables( int nr0, int nr1 );
    void update_step_vars( hk_real step_size );

    void mult_active_x_for_accel();
    void mult_x_with_full_A_minus_b();
    hk_result get_xdirection();
    hk_result solve_lc();
    hk_result setup_l_u_solver();
    hk_result full_setup();
    void get_values_when_setup();
    void startup_setup(int num_actives);
    hk_bool numerical_stability_ok();
    void increase_step_count(int *counter) { (*counter)++; };
    void do_a_little_random_permutation();
    void lcs_bubble_sort_x_vals();
    void move_not_necessary_actives_to_inactives();
    int full_setup_test_ranges(); //returns number of illegal variables
    void move_variable_to_end(int var_nr);
    
    //debug
    void debug_out_lcs();
    void start_debug_lcs();
    void debug_test_all_values();
    void debug_dep_var(int full_var_nr);	


public:
	hk_result solve_lcp( hk_LCP_Input &, hk_LCP_Temp &, hk_LCP_Output &);
};


#endif //HK_LCP_SOLVER

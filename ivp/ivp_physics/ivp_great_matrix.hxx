// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef IVP_GREAT_MATRIX_INCLUDED
#define IVP_GREAT_MATRIX_INCLUDED

#define IVP_MAX_GREAT_MATRIX_SIZE 2048

#if defined( IVP_WILLAMETTE ) || defined( IVP_WMT_ALIGN )
    #define IVP_VECFPU_SIZE 2  
    #define IVP_VECFPU_LD 1
    #define IVP_VECFPU_MASK 0xfffffffe 

    #define IVP_VECFPU_MEM_MASK 0xfffffff0 //16Byte per Block
    #define IVP_VECFPU_MEMSHIFT 3 //8 Bytes per Floating Point Number
#else
    #ifdef IVP_NO_DOUBLE
        #define IVP_VECFPU_SIZE 4
        #define IVP_VECFPU_LD 2
        #define IVP_VECFPU_MASK 0xfffffffc

        #ifdef IVP_NO_DOUBLE
        #	define IVP_VECFPU_MEM_MASK 0xfffffff0 //16Byte
        #	define IVP_VECFPU_MEMSHIFT 2 //4 Bytes per Floating Point Number
        #else
        #	define IVP_VECFPU_MEM_MASK 0xffffffe0 //32Byte
        #	define IVP_VECFPU_MEMSHIFT 3 //8 Bytes per Floating Point Number
        #endif
    #else
//lwss -x64 changes
        #ifdef __x86_64__
            #define IVP_VECFPU_SIZE 4 // taken from vphysics retail
            //#define IVP_VECFPU_LD 0 //unused
            #define IVP_VECFPU_MASK 0xfffffffc // taken from vphysics retail
            #define IVP_VECFPU_MEM_MASK 0xfffffffffffffff0 // taken from vphysics retail
            #define IVP_VECFPU_MEMSHIFT 2 // taken from vphysics retail
        #else
            #define IVP_VECFPU_SIZE 1
            #define IVP_VECFPU_LD 0
            #define IVP_VECFPU_MASK 0xffffffff
            #define IVP_VECFPU_MEM_MASK 0xfffffff8 //8Byte per Block
            #define IVP_VECFPU_MEMSHIFT 3 //8 Bytes per Floating Point Number
        #endif
    #endif
//lwss end
#endif

enum IVP_2P_RET {
    IVP_2P_SIGMA_TRUE,
    IVP_2P_SIGMA_FALSE,
    IVP_2P_ILLEGAL
};

class IVP_U_Memory;

class IVP_Great_Matrix_Many_Zero;

// incremental LU decomposition
// U_matrix and L_matrix are aligned_row_len*n_max big, rowwise organized, but only first n_sub entries of rows are filled
// first matrix A is copied to U_matrix, then following equation holds: L_matrix * A = U_matrix
class IVP_Incr_L_U_Matrix {
public:
    IVP_DOUBLE MATRIX_EPS;
    IVP_DOUBLE *L_matrix;
    IVP_DOUBLE *U_matrix;
    int *index_pos_contains; //index
    int *inv_index_pos_contains;

    IVP_DOUBLE *input_vec;
    IVP_DOUBLE *out_vec;
    IVP_DOUBLE *temp_vec;
    IVP_DOUBLE *mult_vec;
    
    int aligned_row_len; //length of each row
    int n_sub;           //only those are in use

    void solve_lin_equ_index();
    void mult_vec_with_L_index();
    IVP_RETURN_TYPE l_u_decomposition_with_pivoting_index();
    void pivot_search_l_u_index(int col_nr);
    void add_neg_row_to_row_l_u_index(int pivot_row,int dest_row,IVP_DOUBLE factor);
    void exchange_rows_l_u_index(int pivot_col,int exchange);

    IVP_RETURN_TYPE increment_l_u();
    void solve_lin_equ();
    void mult_vec_with_L();
    void solve_vec_with_U();
    IVP_RETURN_TYPE normize_row(int nr);
    IVP_RETURN_TYPE l_u_decomposition_with_pivoting();
    void pivot_search_l_u(int col_nr);
    void add_neg_row_to_row_l_u(int pivot_row,int dest_row,IVP_DOUBLE factor);
    void exchange_rows_l_u(int pivot_col,int exchange);
    void exchange_columns_l_u(int col0,int col1);
    IVP_RETURN_TYPE normize_row_L(int row_nr);
    void add_neg_row_upwards_l_u(int row_down,int dest_row,IVP_DOUBLE factor);
    void subtract_row_L(int from,int dest,IVP_DOUBLE factor);
    IVP_RETURN_TYPE decrement_l_u(int del_nr);
    void delete_row_and_col_l_u(int var_nr);
    void exchange_columns_L(int first,int second);
    void exchange_columns_U(int first,int second);
    void add_neg_row_L(int source_row,int dest_row,IVP_DOUBLE factor);
    void add_neg_col_L(int col0,int col_dest,IVP_DOUBLE factor);
    
    void debug_print_a();
    void debug_print_l_u();
};

// NxN Matrix with N = columns
// many elements are zero, diagonal elements are not zero
class IVP_Great_Matrix_Many_Zero {
public:   
    IVP_DOUBLE MATRIX_EPS;

    int columns;
    int aligned_row_len;
    
    IVP_DOUBLE *matrix_values; //N*N flat array
    IVP_DOUBLE *desired_vector; //N array
    IVP_DOUBLE *result_vector;  //N array
    
    IVP_Great_Matrix_Many_Zero();
    IVP_Great_Matrix_Many_Zero(int n);
    inline void calc_aligned_row_len(); //when using vector FPUs rows have aligned size
    void align_matrix_values();  //shift 'matrix_values' to aligned memory when malloc gives not aligned memory; Warning when using 'delete' afterwards
    IVP_RETURN_TYPE solve_great_matrix_many_zero(); //returns 0 if ok, 1 if not solvable
    IVP_RETURN_TYPE solve_lower_null_matrix();
    void transform_to_lower_null_triangle(); //with gauss algo.
    void add_multiple_line(int first,int second,IVP_DOUBLE factor);
    void exchange_rows(int a,int b);
    void find_pivot_in_column(int j);
    int test_result(IVP_DOUBLE *old_matrix,IVP_DOUBLE *old_desired);
    void matrix_out_before_gauss();
    void copy_matrix(IVP_DOUBLE *values,IVP_DOUBLE *desired);
    void copy_matrix(IVP_Great_Matrix_Many_Zero *orig_mat);
    void copy_to_sub_matrix(IVP_DOUBLE *values_big_matrix,IVP_Great_Matrix_Many_Zero *sub_matrix,int *original_pos);
    int get_number_null_lines();
    IVP_RETURN_TYPE solve_lower_null_matrix_unsymmetrical(int *col_is_original,int *columns_went_inactive,int *number_columns_went_inactive);

    void matrix_multiplication(IVP_DOUBLE *values_0,IVP_DOUBLE *values_1);
    void fill_from_bigger_matrix(IVP_Great_Matrix_Many_Zero *big_matrix,int *original_pos, int n_column);    
    IVP_RETURN_TYPE solve_lp_with_complex();
    IVP_RETURN_TYPE solve_lp_with_own_complex(IVP_U_Memory *mem);
    void matrix_test_unequation(); //text out for visualizing
    IVP_RETURN_TYPE matrix_check_unequation_line(int linenum);

    void set_value(IVP_DOUBLE val, int col, int row);
    IVP_DOUBLE get_value(int col, int row) const;

    IVP_RETURN_TYPE lu_crout(int *index,IVP_DOUBLE *sign);
    IVP_RETURN_TYPE lu_solve(int *index_vec);
    IVP_RETURN_TYPE lu_inverse(IVP_Great_Matrix_Many_Zero *matrix_out,int *index_vec);
    
    IVP_RETURN_TYPE invert(IVP_Great_Matrix_Many_Zero *out_matrix);
    void mult(); // input: desired_vector, output: result_vector
	void mult_aligned(); //same but matrix is aligned
    int print_great_matrix(const char *comment) const;

    void debug_fill_zero();
#ifdef DEBUG
    void plausible_input_check();
#endif
};

void IVP_Great_Matrix_Many_Zero::calc_aligned_row_len() {
    aligned_row_len=(columns+IVP_VECFPU_SIZE-1)&IVP_VECFPU_MASK;
}

class IVP_Complex_Simple {
public:
  int memory_column; // aligned to 4
  int n_columns;
  int m_rows;
  IVP_INT32 *inactives;
  IVP_INT32 *inactives_copy;
  IVP_INT32 *actives;
  IVP_INT32 *actives_copy;
  IVP_DOUBLE *matrix;
  IVP_DOUBLE *matrix_copy;
  //IVP_DOUBLE *x_vals;
  IVP_INT32 *index_is_at;
  IVP_INT32 *index_is_at_copy;

  IVP_DOUBLE *help_column;
  IVP_DOUBLE *help_row;

  int search_for_save_pivots(int *pivot_row,int *pivot_column); //returns 0 if fail
  void complex_step(int pivot_row,int pivot_column);
  void complex_step_fast(int pivot_row,int pivot_column);
  IVP_RETURN_TYPE complex_solving();
  void debug_print(void *fp);
  void make_copy();
  void undo_copy();
};

// assumption: the memory is always allocated aligned to the block size
//             otherwise -> failure
class IVP_VecFPU {
public:
    static inline void fpu_add_multiple_row(IVP_DOUBLE *target_adress,IVP_DOUBLE *source_adress,IVP_DOUBLE factor,int size,IVP_BOOL adress_aligned);
    static inline void fpu_multiply_row(IVP_DOUBLE *target_adress,IVP_DOUBLE factor,int size,IVP_BOOL adress_aligned);
    static inline void fpu_exchange_rows(IVP_DOUBLE *target_adress1,IVP_DOUBLE *target_adress2,int size,IVP_BOOL adress_aligned);
    static inline void fpu_copy_rows(IVP_DOUBLE *target_adress,IVP_DOUBLE *source_adress,int size,IVP_BOOL adress_aligned);
    static inline void fpu_set_row_to_zero(IVP_DOUBLE *target_adress,int size,IVP_BOOL adress_aligned);
    static inline IVP_DOUBLE fpu_large_dot_product(IVP_DOUBLE *base_a,IVP_DOUBLE *x,int size,IVP_BOOL adress_aligned);
};

class IVP_Linear_Constraint_Solver {
    IVP_DOUBLE SOLVER_EPS;
    IVP_DOUBLE GAUSS_EPS;
    IVP_DOUBLE TEST_EPS;
    IVP_DOUBLE MAX_STEP_LEN;
    IVP_DOUBLE *full_A;
    IVP_DOUBLE *full_b;
    IVP_DOUBLE *temp;
    IVP_DOUBLE *full_x; 	   //must be greater equal 0.0
    IVP_DOUBLE *delta_f; //delta of above per step
    IVP_DOUBLE *accel; //is A*x-b must be greater equal 0.0
    IVP_DOUBLE *delta_accel;
    IVP_DOUBLE *reset_x;
    IVP_DOUBLE *reset_accel;
    int *actives_inactives_ignored;
    int *variable_is_found_at;
    int n_variables;
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
    IVP_Incr_L_U_Matrix lu_sub_solver;
    IVP_Great_Matrix_Many_Zero sub_solver_mat;
    IVP_Great_Matrix_Many_Zero full_solver_mat;
    IVP_Great_Matrix_Many_Zero debug_mat;
    IVP_Great_Matrix_Many_Zero inv_mat;
    
    void decrement_sub_solver( int sub_pos );
    void increment_sub_solver();
    void exchange_activ_inactiv( int change_var );
    void exchange_lcs_variables( int nr0, int nr1 );
    void update_step_vars( IVP_DOUBLE step_size );

    void alloc_memory(IVP_U_Memory *my_mem);
    void mult_active_x_for_accel();
    void mult_x_with_full_A_minus_b();
    IVP_RETURN_TYPE get_fdirection();
    IVP_RETURN_TYPE solve_lc();
    IVP_RETURN_TYPE setup_l_u_solver();
    IVP_RETURN_TYPE full_setup();
    void get_values_when_setup();
    void startup_setup(int num_actives);
    IVP_BOOL numerical_stability_ok();
    void increase_step_count(int *counter);
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
    
    IVP_RETURN_TYPE init_and_solve_lc(IVP_DOUBLE *A_in,IVP_DOUBLE *b_in,IVP_DOUBLE *result_vec_out,int var_num,int actives_at_begin,IVP_U_Memory *my_mem);   
};

#endif


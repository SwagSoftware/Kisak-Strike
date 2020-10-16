#ifndef HK_GAUSS_ELEMINATION
#define HK_GAUSS_ELEMINATION


typedef float hk_gauss_real;

#define hk_GAUSS_SOLUTION_TOLERANCE (1000.0f) //factor to hk_GAUSS_ELM_EPS

class hk_Gauss_Elm_Input {
public:
	hk_gauss_real m_gauss_eps;
	hk_gauss_real *m_A;      //N*N Matrix, organized by rows - rows have to be memory aligned 
	hk_gauss_real *m_b;      //vector, memory aligned

	int           m_n_columns;
	int           m_aligned_row_len; //number of elements (not Bytes!) aligned per row
};

class hk_Gauss_Elm_Output {
public:
	hk_gauss_real *m_result; //vector, memory aligned
};

class hk_Gauss_Elm_Solver {
    hk_gauss_real m_gauss_eps;
    hk_gauss_real *m_A;      //N*N Matrix, organized by rows - rows have to be memory aligned 
    hk_gauss_real *m_b;      //vector, memory aligned
    int           m_n_columns;
    int           m_aligned_row_len; //number of elements (not Bytes!) aligned per row
    hk_gauss_real *m_result; //vector, memory aligned
    
    void transform_to_lower_null_triangle(); //with gauss elemination algo.
    hk_result solve_lower_null_matrix();
    void add_multiple_line(int first,int second,hk_gauss_real factor);
    void exchange_rows(int a,int b);
    void find_pivot_in_column(int j);	

public:
	hk_result solve_gauss_elemination( hk_Gauss_Elm_Input &, hk_Gauss_Elm_Output &);
};

#endif //HK_GAUSS_ELEMINATION

#if 0
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
#endif

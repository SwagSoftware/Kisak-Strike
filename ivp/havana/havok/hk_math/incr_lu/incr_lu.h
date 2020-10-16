#ifndef HK_INCR_LU_MATRIX
#define HK_INCR_LU_MATRIX

#define hk_Incr_LU_EPS 1E-6f

// incremental LU decomposition
// U_matrix and L_matrix are aligned_row_len*n_max big, rowwise organized, but only first n_sub entries of rows are filled
// first matrix A is copied to U_matrix, then following equation holds: L_matrix * A = U_matrix
class hk_Incr_LU_Matrix {
    void pivot_search_l_u(int col_nr);
    void add_neg_row_to_row_l_u(int pivot_row,int dest_row,hk_incrlu_real factor);
    void exchange_rows_l_u(int pivot_col,int exchange);
    void exchange_columns_l_u(int col0,int col1);
    hk_result normize_row_L(int row_nr);
    hk_result normize_row(int nr);

    void mult_vec_with_L();
    void solve_vec_with_U();
    void add_neg_row_upwards_l_u(int row_down,int dest_row,hk_incrlu_real factor);
    void subtract_row_L(int from,int dest,hk_incrlu_real factor);
    void delete_row_and_col_l_u(int var_nr);
    void exchange_columns_L(int first,int second);
    void exchange_columns_U(int first,int second);
    void add_neg_row_L(int source_row,int dest_row,hk_incrlu_real factor);
    void add_neg_col_L(int col0,int col_dest,hk_incrlu_real factor);
    
    void debug_print_a();

public:
    hk_incrlu_real *m_L_matrix;
    hk_incrlu_real *m_U_matrix;

    hk_incrlu_real *m_inout_vec;
    hk_incrlu_real *m_temp_vec;

    int m_aligned_row_len; //length of each row
    int m_n_sub;           //only those are in use

    hk_result l_u_decomposition_with_pivoting();
    hk_result decrement_l_u(int del_nr);
    hk_result increment_l_u();
    void solve_lin_equ(); //input and output is stored/found in vector m_inout_vec

    void debug_print_l_u();
};


#endif //HK_INCR_LU_MATRIX

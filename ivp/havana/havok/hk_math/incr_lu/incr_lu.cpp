#include <hk_math/vecmath.h>

#include <hk_math/incr_lu/incr_lu.h>
#include <hk_math/vector_fpu/vector_fpu.h>

//only for debugging:
#include <hk_math/densematrix.h>
#include <hk_math/gauss_elimination/gauss_elimination.h>
#include <hk_math/lcp/lcp_solver.h>

hk_result hk_Incr_LU_Matrix::normize_row_L(int row_nr) {
    hk_incrlu_real *base=&m_L_matrix[row_nr*m_aligned_row_len];
    hk_incrlu_real val=base[row_nr];
    if(hk_Math::fabsd(val)<hk_Incr_LU_EPS) {
	return HK_FAULT;
    }
    hk_incrlu_real inv_val=1.0f/val;
    
    hk_VecFPU::fpu_multiply_row(&base[0],inv_val,m_n_sub,HK_FALSE); //should be TRUE
    
    base[row_nr]=1.0f;
    return HK_OK;
}

hk_result hk_Incr_LU_Matrix::normize_row(int row_nr) {
    int row_base = row_nr * m_aligned_row_len;
    hk_incrlu_real val=m_U_matrix[row_base + row_nr];
    if(hk_Math::fabsd(val)<hk_Incr_LU_EPS) {
	return HK_FAULT;
    }
    hk_incrlu_real inv_val = 1.0f / val;
    hk_VecFPU::fpu_multiply_row(&m_L_matrix[row_base],inv_val,m_n_sub,HK_FALSE);
    hk_VecFPU::fpu_multiply_row(&m_U_matrix[row_base+row_nr+1],inv_val,m_n_sub-row_nr-1,HK_FALSE);
    
    m_U_matrix[row_base + row_nr] = 1.0f;
    return HK_OK;
}

void hk_Incr_LU_Matrix::exchange_rows_l_u(int pivot_col,int exchange) {
    int base_a = pivot_col*m_aligned_row_len;
    int base_b = exchange *m_aligned_row_len;

#if 0 /* single FPU */  
    hk_incrlu_real temp;
    int i;
    for(i=m_n_sub-1;i>=pivot_col;i--) {
	temp=m_U_matrix[base_a + i];
	m_U_matrix[base_a + i]=m_U_matrix[base_b + i];
	m_U_matrix[base_b + i]=temp;
    }
    for(i=m_n_sub-1;i>=0;i--) {
	temp=m_L_matrix[base_a + i];
	m_L_matrix[base_a + i]=m_L_matrix[base_b + i];
	m_L_matrix[base_b + i]=temp;	
    }
#else
    hk_VecFPU::fpu_exchange_rows(&m_U_matrix[base_a+pivot_col],&m_U_matrix[base_b+pivot_col],m_n_sub-pivot_col,HK_FALSE);
    hk_VecFPU::fpu_exchange_rows(&m_L_matrix[base_a],&m_L_matrix[base_b],m_n_sub,HK_FALSE);
#endif    
}

// search from col_nr|colnr down to m_n_sub|colnr
void hk_Incr_LU_Matrix::pivot_search_l_u(int col_nr) {
    hk_incrlu_real biggest = hk_Math::fabsd( m_U_matrix[col_nr * m_aligned_row_len + col_nr] );
    int pivot_row = col_nr;

    for(int i=m_n_sub-1;i>col_nr;i--) {
	hk_incrlu_real new_val = hk_Math::fabsd( m_U_matrix[i * m_aligned_row_len + col_nr] );
	if( new_val > biggest ) {
	    pivot_row=i;
	    biggest = new_val;
	}
    }

    if(pivot_row!=col_nr) {
	exchange_rows_l_u(col_nr,pivot_row);
    }
}

// add row to a row more upwards
void hk_Incr_LU_Matrix::add_neg_row_upwards_l_u(int row_down,int dest_row,hk_incrlu_real factor) {
    hk_incrlu_real *base_a=&m_U_matrix[row_down * m_aligned_row_len];
    hk_incrlu_real *base_b=&m_U_matrix[dest_row * m_aligned_row_len];
#if 0 /* single FPU */
    int i;
    for(i=row_down+1;i<m_n_sub;i++) {
	base_b[i] -= factor * base_a[i];
    }
#else
    hk_VecFPU::fpu_add_multiple_row(&base_b[row_down+1],&base_a[row_down+1],-factor,m_n_sub-row_down-1,HK_FALSE);
#endif    
    
    
    base_a=&m_L_matrix[row_down * m_aligned_row_len];
    base_b=&m_L_matrix[dest_row * m_aligned_row_len];
#if 0    
    for(i=m_n_sub-1;i>=0;i--) {
	base_b[i] -=factor * base_a[i];
    }
#else
    hk_VecFPU::fpu_add_multiple_row(&base_b[0],&base_a[0],-factor,m_n_sub,HK_TRUE); //should be TRUE
#endif    
    base_b[row_down] = 0.0f;
}

// add row to a row more downwards
void hk_Incr_LU_Matrix::add_neg_row_to_row_l_u(int pivot_row,int dest_row,hk_incrlu_real factor) {
    int base_a = pivot_row * m_aligned_row_len;
    int base_b = dest_row * m_aligned_row_len;

#if 0 /* single FPU */    
    int i;
    for(i=m_n_sub-1;i>pivot_row;i--) {
	m_U_matrix[ base_b + i ] -= factor*m_U_matrix[ base_a + i];
    }
    for(i=m_n_sub-1;i>=0;i--) {
	m_L_matrix[ base_b + i ] -= factor*m_L_matrix[ base_a + i]; 
    }
#else
    hk_VecFPU::fpu_add_multiple_row(&m_U_matrix[ base_b + pivot_row +1], &m_U_matrix[ base_a + pivot_row +1], -factor, m_n_sub-pivot_row-1,HK_FALSE);
    hk_VecFPU::fpu_add_multiple_row(&m_L_matrix[ base_b ], &m_L_matrix[ base_a ], -factor, m_n_sub, HK_FALSE); //set to TRUE
#endif
    m_U_matrix[base_b + pivot_row] = 0.0f; 
}

void hk_Incr_LU_Matrix::subtract_row_L(int from,int dest,hk_incrlu_real factor) {
    hk_incrlu_real *base_a=&m_L_matrix[from*m_aligned_row_len];
    hk_incrlu_real *base_b=&m_L_matrix[dest*m_aligned_row_len];
#if 0 /* single FPU */
    int i;
    for(i=m_n_sub-1;i>=0;i--) {
	base_b[i] -= base_a[i]*factor;
    }
#else
    hk_VecFPU::fpu_add_multiple_row(&base_b[0],&base_a[0],-factor,m_n_sub,HK_FALSE); //should be TRUE
#endif
    
    base_b[from]=0.0f;
}

hk_result hk_Incr_LU_Matrix::l_u_decomposition_with_pivoting() {
    if( m_n_sub == 0 ) {
	return HK_OK;
    }
    
    int i;
    for(i=m_n_sub-1;i>=0;i--) {
	hk_incrlu_real *base=&m_L_matrix[ i*m_aligned_row_len ];
	hk_VecFPU::fpu_set_row_to_zero(&base[0],m_n_sub,HK_FALSE); //should be TRUE
	base[i]=1.0f;
    }
    
    int rows_to_do;
    for(rows_to_do=1;rows_to_do<m_n_sub;rows_to_do++) {
	//debug_print_l_u();
	int pivot_elem=rows_to_do-1;
	pivot_search_l_u(pivot_elem);
	//m_L_matrix[pivot_elem * n_max + pivot_elem] = 1.0f;
	if(normize_row(pivot_elem)==HK_FAULT) {
	    return HK_FAULT;
	}
	
	for(int rows_become_zero=m_n_sub-1;rows_become_zero>=rows_to_do;rows_become_zero--) {
	    hk_incrlu_real factor = m_U_matrix[ rows_become_zero * m_aligned_row_len + pivot_elem ];
	    if( factor != 0.0f ) { //original matrix may be filled with many zeros
		add_neg_row_to_row_l_u(pivot_elem,rows_become_zero,factor);
	    }
	}
    }
    //m_L_matrix[ (m_n_sub-1)*n_max + m_n_sub-1 ] = 1.0f;
    if(normize_row(m_n_sub-1)==HK_FAULT) {
	return HK_FAULT;
    }
    //debug_print_l_u();
    return HK_OK;
}

// incremental l_u_decomposition: matrix is correctly decomposed to m_n_sub*m_n_sub
// last row of new U was already added, row has length m_n_sub+1
// last column of new U is stored in m_m_input_vec, column has length m_n_sub
hk_result hk_Incr_LU_Matrix::increment_l_u() {
    int i;
#if 0
    for(i=m_n_sub-1;i>=0;i--) {
	int original_index = i; //m_index_pos_contains[ i ];
	m_mult_vec[i] = m_input_vec[ original_index ];
    }
#endif    

    mult_vec_with_L();

    for(i=m_n_sub-1;i>=0;i--) {
	m_U_matrix[ i * m_aligned_row_len + m_n_sub ] = m_temp_vec[ i ];
    }
    
    //clear row and column of L
    for(i=m_n_sub-1;i>=0;i--) {
	    m_L_matrix[ i * m_aligned_row_len + m_n_sub ] = 0.0f;
    }
	hk_VecFPU::fpu_set_row_to_zero(&m_L_matrix[m_n_sub*m_aligned_row_len],m_n_sub,HK_FALSE); //should be TRUE
    
    m_L_matrix[ m_n_sub * m_aligned_row_len + m_n_sub ] = 1.0f;

    //inv_m_index_pos_contains[ m_n_sub ] = m_n_sub;
    //m_index_pos_contains[ m_n_sub ] = m_n_sub;

    int base_row = m_n_sub * m_aligned_row_len;

    m_n_sub++;
    for(i=0;i<m_n_sub-1;i++) {
	hk_incrlu_real factor = m_U_matrix[ base_row + i ];
	add_neg_row_to_row_l_u(i,m_n_sub-1,factor);
    }

    hk_result ret=normize_row( m_n_sub-1 );
    //debug_print_l_u();
    return ret;
}

// variable with nr del_nr is removed from linear equation systems
// first U is transformed in a way only 0 0 1 0 0 is found in the row
// then L is transformed in a way only 0 0 1 0 0 is found in the column
// then column and row is deleted from L and U
hk_result hk_Incr_LU_Matrix::decrement_l_u(int del_nr) {
    int i;
    //exchange_rows_l_u(del_nr,m_n_sub-1);
    exchange_columns_L(del_nr,m_n_sub-1);
    exchange_columns_U(del_nr,m_n_sub-1);

    for(i=m_n_sub-2;i>del_nr;i--) {
	hk_incrlu_real factor=m_U_matrix[ i * m_aligned_row_len + del_nr ];
	if(factor != 0.0f) {
	    add_neg_row_L(m_n_sub-1,i,factor);
	}
	m_U_matrix[ i * m_aligned_row_len + del_nr ] = 0.0f;
    }
    if( normize_row( del_nr ) != HK_OK ) {
	//hk_Console::get_instance()->printf("special_case_decrement\n");
	add_neg_row_L(m_n_sub-1,del_nr,-1.0f);
	m_U_matrix[ del_nr * m_aligned_row_len + del_nr ] = 1.0f;
    }
    
    for(i=del_nr;i<m_n_sub-1;i++) {
	hk_incrlu_real factor=m_U_matrix[ (m_n_sub-1) * m_aligned_row_len + i];
	if(factor != 0.0f) {
	    add_neg_row_to_row_l_u(i,m_n_sub-1,factor);
	}
    }
    //normize_row(m_n_sub-1);
    
    if( normize_row_L(m_n_sub-1) != HK_OK ) {
	m_n_sub--;
	return HK_FAULT;
    }
    
    del_nr = m_n_sub-1;
    for(i=del_nr-1;i>=0;i--) {
	hk_incrlu_real factor=m_L_matrix[i*m_aligned_row_len + del_nr];
	if(factor != 0.0f) {
	    subtract_row_L(del_nr,i,factor);
	}
    }
    

    m_n_sub--;
    return HK_OK;
}

void hk_Incr_LU_Matrix::add_neg_col_L(int col0,int col_dest,hk_incrlu_real factor) {
    //Vector FPU not possible
    int i;
    for(i=m_n_sub-1;i>=0;i--) {
	m_L_matrix[ i * m_aligned_row_len + col_dest ] -= factor * m_L_matrix[ i * m_aligned_row_len + col0 ];
    }
}

void hk_Incr_LU_Matrix::exchange_columns_L(int col0,int col1) {
    //Vector FPU not possible
    int i;
    for(i=0;i<m_n_sub;i++) {
	hk_incrlu_real temp_val;
	//temp_val=m_L_matrix[i * n_max + col0];
	//m_L_matrix[i * n_max + col0]=m_L_matrix[i * n_max + col1];
	//m_L_matrix[i * n_max + col1]=temp_val;
	
	temp_val=m_L_matrix[i * m_aligned_row_len + col0];
	m_L_matrix[i * m_aligned_row_len + col0]=m_L_matrix[i * m_aligned_row_len + col1];
	m_L_matrix[i * m_aligned_row_len + col1]=temp_val;
    }
}

void hk_Incr_LU_Matrix::exchange_columns_U(int col0,int col1) {
    //Vector FPU not possible
    int i;
    for(i=0;i<m_n_sub;i++) {
	hk_incrlu_real temp_val;
	//temp_val=m_L_matrix[i * n_max + col0];
	//m_L_matrix[i * n_max + col0]=m_L_matrix[i * n_max + col1];
	//m_L_matrix[i * n_max + col1]=temp_val;
	
	temp_val=m_U_matrix[i * m_aligned_row_len + col0];
	m_U_matrix[i * m_aligned_row_len + col0]=m_U_matrix[i * m_aligned_row_len + col1];
	m_U_matrix[i * m_aligned_row_len + col1]=temp_val;
    }
}

void hk_Incr_LU_Matrix::mult_vec_with_L() {
    int i;
    for(i=m_n_sub-1;i>=0;i--) {
	hk_incrlu_real sum=hk_VecFPU::fpu_large_dot_product(&m_L_matrix[i*m_aligned_row_len],&m_inout_vec[0],m_n_sub,HK_TRUE);
	m_temp_vec[i]=sum;
    }
}

void hk_Incr_LU_Matrix::solve_vec_with_U() {
    int i;
    for(i=m_n_sub-1;i>=0;i--) {
//#if !defined(IVP_WILLAMETTE)
	    hk_incrlu_real leftsum=0.0f;
	    int j;
	    for(j=m_n_sub-1;j>i;j--) {
	        leftsum += m_U_matrix[ i * m_aligned_row_len + j ] * m_temp_vec[ j ];
		}
#if 0 /*doesnt work, but why?*/
		//WARNING !!!!!
		//this is dirty: vector multiplication starts in middle of datastructure
		//               put one zero in front -> zero mult something is zero
		hk_incrlu_real remember=m_U_matrix[i*m_aligned_row_len+i];
		m_U_matrix[i*m_aligned_row_len+i]=0.0f;
		hk_incrlu_real leftsum = hk_VecFPU::fpu_large_dot_product(&m_U_matrix[i*m_aligned_row_len+i+1],&m_temp_vec[i+1],m_n_sub-i-1,HK_FALSE);
		m_U_matrix[i*m_aligned_row_len+i]=remember;
		if( hkMath::fabs(leftsum-oldleftsum) > 0.0001f ) {
			leftsum=oldleftsum;
		}
#endif 
	    hk_incrlu_real res = m_temp_vec[ i ] - leftsum;
	    m_temp_vec[ i ] = res;
	    m_inout_vec[ i ] = res;
    }
} 

void hk_Incr_LU_Matrix::solve_lin_equ() {
    mult_vec_with_L();
    solve_vec_with_U(); 
}

void hk_Incr_LU_Matrix::add_neg_row_L(int source_row,int dest_row,hk_incrlu_real factor) {
    int base_a = source_row * m_aligned_row_len;
    int base_b = dest_row * m_aligned_row_len;

#if 0 /* single FPU */    
    int i;
    for(i=m_n_sub-1;i>=0;i--) {
	m_L_matrix[ base_b + i ] -= factor*m_L_matrix[ base_a + i]; 
    }
#else
    hk_VecFPU::fpu_add_multiple_row(&m_L_matrix[base_b],&m_L_matrix[base_a],-factor,m_n_sub,HK_FALSE); //should be TRUE
#endif    
}

void hk_Incr_LU_Matrix::debug_print_l_u() {
    int i;
#if 0
    hk_Console::get_instance()->printf("\nindex  ");
    for(i=0;i<m_n_sub;i++) {
	hk_Console::get_instance()->printf("%d  ",m_index_pos_contains[i]);
    }
    hk_Console::get_instance()->printf("\n");
    hk_Console::get_instance()->printf("\ninvindex  ");
    for(i=0;i<m_n_sub;i++) {
	hk_Console::get_instance()->printf("%d  ",inv_m_index_pos_contains[i]);
    }
    hk_Console::get_instance()->printf("\n");
#endif    
    hk_Console::get_instance()->printf("  L                                      U\n");
    for(i=0;i<m_n_sub;i++) {
	int j;
	for(j=0;j<m_n_sub;j++) {
	    hk_Console::get_instance()->printf("%.5f  ",m_L_matrix[i*m_aligned_row_len+j]);
	}
	hk_Console::get_instance()->printf("          ");
	for(j=0;j<m_n_sub;j++) {
	    hk_Console::get_instance()->printf("%.5f  ",m_U_matrix[i*m_aligned_row_len+j]);
	}
	hk_Console::get_instance()->printf("\n");
    }
}

void hk_Incr_LU_Matrix::debug_print_a() {
#if 0
	IVP_Great_Matrix_Many_Zero *was_l=new IVP_Great_Matrix_Many_Zero(m_n_sub);
	IVP_Great_Matrix_Many_Zero *was_u=new IVP_Great_Matrix_Many_Zero(m_n_sub);

	int i,j;
	for(i=0;i<m_n_sub;i++) {
	  for(j=0;j<m_n_sub;j++) {
	    was_l->matrix_values[i*m_n_sub+j]=m_L_matrix[i*m_aligned_row_len+j];
	    was_u->matrix_values[i*m_n_sub+j]=m_U_matrix[i*m_aligned_row_len+j];	    
	  }
	}

	IVP_Great_Matrix_Many_Zero *inv_l=new IVP_Great_Matrix_Many_Zero(m_n_sub);
	was_l->invert(inv_l);

	IVP_Great_Matrix_Many_Zero *new_a=new IVP_Great_Matrix_Many_Zero(m_n_sub);
	new_a->matrix_multiplication(inv_l->matrix_values,was_u->matrix_values);

	new_a->print_great_matrix("orig_A");
#endif
}

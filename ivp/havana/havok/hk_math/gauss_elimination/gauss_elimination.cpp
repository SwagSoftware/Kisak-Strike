#include <hk_math/vecmath.h>

#include <hk_math/gauss_elimination/gauss_elimination.h>
#include <hk_math/vector_fpu/vector_fpu.h>

// add factor * line number j to line number i (j=first i=second)
// the matrix-entries left of j in both lines are zero
void hk_Gauss_Elm_Solver::add_multiple_line(int first,int second,hk_gauss_real factor)
{    
    hk_gauss_real *first_line=&m_A[first * this->m_aligned_row_len + first];
    hk_gauss_real *second_line=&m_A[second * this->m_aligned_row_len + first];

    hk_VecFPU::fpu_add_multiple_row(second_line,first_line,factor,m_n_columns-first,HK_FALSE);
    m_b[second]+=m_b[first] * factor;
}

hk_result hk_Gauss_Elm_Solver::solve_gauss_elemination( hk_Gauss_Elm_Input &inp, hk_Gauss_Elm_Output &outp) {
    m_A=inp.m_A;
    m_b=inp.m_b;
    m_n_columns=inp.m_n_columns;
    m_aligned_row_len=inp.m_aligned_row_len;
    m_result=outp.m_result;

    transform_to_lower_null_triangle();
    hk_result res=solve_lower_null_matrix();
    return res;
}

void hk_Gauss_Elm_Solver::find_pivot_in_column(int col)
{
    hk_gauss_real best_val = hk_Math::fabsd(m_A[col*m_aligned_row_len+col]);
    int pos=-1;
    for(int i=m_n_columns-1;i > col;i--)    {
	hk_gauss_real h=m_A[i*m_aligned_row_len+col];

	if( hk_Math::fabsd(h) > best_val )	{
	    best_val = hk_Math::fabsd(h);
	    pos=i;
	}
    }
    if(pos>=0) {
        exchange_rows(col,pos);
    }
}

//optimization: when called from transform_to_lower_null_triangle, begin by 'a' (first values are 0.0f)
void hk_Gauss_Elm_Solver::exchange_rows(int a,int b)
{
    hk_gauss_real h;
#if 0 /* without Vector FPU */   
    for(int i=m_n_columns-1;i>=0;i--) {
	h=m_A[a*m_aligned_row_len+i];
	m_A[a*m_aligned_row_len+i]=m_A[b*m_aligned_row_len+i];
	m_A[b*m_aligned_row_len+i]=h;
    }
#endif
    hk_VecFPU::fpu_exchange_rows(&m_A[b*m_aligned_row_len],&m_A[a*m_aligned_row_len],m_n_columns,HK_TRUE);
    
    h=m_b[a];
    m_b[a]=m_b[b];
    m_b[b]=h;
}

// solve with Gauss-Algo. : upper triangle matrix with nulls in lower part
void hk_Gauss_Elm_Solver::transform_to_lower_null_triangle()
{
    //matrix_out_before_gauss();
    int i,j;
    
    //j is counting m_n_columns, i is counting lines

    //walk through m_n_columns
    for(j=0;j<m_n_columns;j++)
    {
	hk_gauss_real diagonal_elem,negativ_inv_diagonal;
#if 0
	//not always pivot search
	diagonal_elem=m_A[j*m_aligned_row_len+j];
	if(hk_Math::fabs(diagonal_elem)<hk_GAUSS_ELM_EPS) {
	    find_pivot_in_column(j);
	    diagonal_elem=m_A[j*m_aligned_row_len+j];
	    if(hk_Math::fabs(diagonal_elem)<hk_GAUSS_ELM_EPS) {
		goto column_done;
	    }
	}
#endif
	//always pivot search
	find_pivot_in_column(j);
	diagonal_elem=m_A[j*m_aligned_row_len+j];
	if(hk_Math::fabsd(diagonal_elem)<m_gauss_eps) {
	    goto column_done;
	}	
	
	negativ_inv_diagonal=-1.0f/diagonal_elem;
	//walk through lines
	for(i=j+1;i<m_n_columns;i++)
	{
	    hk_gauss_real col_down_elem=m_A[i*m_aligned_row_len+j];
	    if(hk_Math::fabsd(col_down_elem)>m_gauss_eps)
	    {
		add_multiple_line(j,i,col_down_elem*negativ_inv_diagonal);
	    }
	}
column_done: ;	
    }

}

// matrix values are in upper triangle form, solve from bottom to top
hk_result hk_Gauss_Elm_Solver::solve_lower_null_matrix()
{
    int i;
    for(i=m_n_columns-1;i>=0;i--)
    {
	hk_gauss_real *pointer_in_line=&m_A[i*m_aligned_row_len+m_n_columns-1];
     
	hk_gauss_real right_side=m_b[i];
	for(int j=m_n_columns-1;j>i;j--)
	{
	    right_side -= m_b[j] * *pointer_in_line;
	    pointer_in_line--;
	}
	hk_gauss_real left_side=*pointer_in_line;
	if(hk_Math::fabsd(left_side) < m_gauss_eps){
	    if(hk_Math::fabsd(right_side) < m_gauss_eps*hk_GAUSS_SOLUTION_TOLERANCE)	    {
		// rang(matrix) is not maximal and result space is greater 1
		// choose an arbitrary result element (0.0f)
		//printf("got_gauss_null %f %f\n",left_side,right_side);
		m_b[i]=0.0f;
	    } else {
		// no result possible
		if(1) {
		    i=m_n_columns-1;
		    while(i>=0) {
		        m_result[i]=0.0; //prevent SUN from detecting rui error
		        i--;
		    }
		}
		return HK_FAULT;
	    }
	} else {
	    m_b[i]=right_side/left_side; // division left_side is not needed, it was done already above (make a vector of inverses)
	}
    }

    for(i=0;i<m_n_columns;i++) {
	m_result[i]=m_b[i];
    }
    
    return HK_OK;
}

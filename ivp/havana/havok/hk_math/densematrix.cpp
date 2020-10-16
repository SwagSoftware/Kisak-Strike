#include <hk_math/vecmath.h>
#include <hk_math/densematrix.h>
#include <hk_math/vector_fpu/vector_fpu.h>

void hk_Dense_Matrix::mult_vector( hk_real *x_vector, hk_real *result_vector ) const
{
    int i;
    for(i=getNumRows()-1;i>=0;i--)
	{
		result_vector[i]=0.0f;
    }

    hk_real *start_column=&m_elt[m_lda*getNumCols()];
    for(i=getNumRows()-1;i>=0;i--)
	{
		start_column-=m_lda;
		hk_VecFPU::fpu_add_multiple_row(result_vector,start_column,x_vector[i],getNumCols(),HK_TRUE);
    }
}


hk_Dynamic_Dense_Matrix::hk_Dynamic_Dense_Matrix(int r, int c)
	: hk_Dense_Matrix( hk_allocate(hk_real, HK_NEXT_MULTIPLE_OF(4,c)*r, HK_MEMORY_CLASS_DENSE_VECTOR),
						r,
						c,
						HK_NEXT_MULTIPLE_OF(4,c) )
{
}

hk_Dynamic_Dense_Matrix::~hk_Dynamic_Dense_Matrix()
{
	hk_deallocate(hk_real, m_elt, HK_NEXT_MULTIPLE_OF(4,m_cols)*m_rows, HK_MEMORY_CLASS_DENSE_VECTOR);
}


hk_Dense_Matrix::hk_Dense_Matrix(hk_real *mem, int r, int c, int lda)
	:	m_elt(mem),
		m_rows(r),
		m_cols(c),
		m_lda(lda)
{
}

void hk_Dense_Matrix::makeZero()
{
	for( int i = m_lda * m_rows - 1; i >= 0; --i)
	{
		m_elt[i] = 0;
	}
}


void hk_Dense_Matrix_1x1::makeZero()
{
	getRealPointer()[0] = 0;
}


void hk_Dense_Matrix_3x3::makeZero()
{
	m_elt_buffer.set_zero();
}


hk_Matrix3 &hk_Dense_Matrix_3x3::get_matrix3()
{
	return m_elt_buffer;
}


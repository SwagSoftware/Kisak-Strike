
template <int N>
void hk_Fixed_Dense_Vector<N>::set_add_mul( const hk_Fixed_Dense_Vector<N> &a, hk_real factor, const hk_real b[])
{
	int i;
	if ( N & 1)
	{
		m_elt_buffer[N-1] = a.m_elt_buffer[N-1] + factor * b[N-1];
		i = N-2;
	}
	else
	{
		i = N-1;
	}

	for (; i>0; i-=2)
	{
		m_elt_buffer[i] = a.m_elt_buffer[i] + factor * b[i];
		m_elt_buffer[i-1] = a.m_elt_buffer[i-1] + factor * b[i-1];
	}
}

template <int N>
void hk_Fixed_Dense_Vector<N>::set_mul_add_mul( hk_real factor_a, const hk_Fixed_Dense_Vector<N> &a, hk_real factor_b, const hk_real b[])
{
	int i;
	if ( N & 1)
	{
		m_elt_buffer[N-1] = a.m_elt_buffer[N-1] * factor_a + b[N-1] * factor_b;
		i = N-2;
	}
	else
	{
		i = N-1;
	}

	for (; i>0; i-=2)
	{
		m_elt_buffer[i] = a.m_elt_buffer[i] * factor_a + factor_b * b[i];
		m_elt_buffer[i-1] = a.m_elt_buffer[i-1]  * factor_a + factor_b * b[i-1];
	}
}


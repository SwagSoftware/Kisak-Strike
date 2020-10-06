#ifdef HK_PS2
#	include <hk_math/matrix3ps2.inl>
#else	//HK_PS2

hk_Matrix3::hk_Matrix3()
{
}

inline void hk_Matrix3::set_elem(int r, int c, hk_real v)
{
	// works for transform too if elements are contiguous
	get_column(c)(r) = v;
}

inline hk_real hk_Matrix3::operator() (int r, int c) const
{
	// works for transform too if elements are contiguous
	return get_column(c)(r);
}



inline hk_real *hk_Matrix3::get_elem_address(int r, int c)
{
	// works for transform too if elements are contiguous
	return (&get_column(c).x) + r;
}


inline void hk_Matrix3::_set_rows( const hk_Vector3& r0,
							const hk_Vector3& r1,
							const hk_Vector3& r2)
{
	get_column(0).set( r0(0), r1(0), r2(0) );
	get_column(1).set( r0(1), r1(1), r2(1) );
	get_column(2).set( r0(2), r1(2), r2(2) );
}

inline void hk_Matrix3::_get_rows( hk_Vector3& r0,
							hk_Vector3& r1,
							hk_Vector3& r2)
{
	r0.set( get_column(0)(0), get_column(1)(0), get_column(2)(0) );
	r1.set( get_column(0)(1), get_column(1)(1), get_column(2)(1) );
	r2.set( get_column(0)(2), get_column(1)(2), get_column(2)(2) );
}

inline void hk_Matrix3::_get_row( int row, hk_Vector3& r)
{
	r.set( get_column(0)(row), get_column(1)(row), get_column(2)(row) );
}

inline hk_Vector3& hk_Matrix3::get_column(int x)
{
	return *(hk_Vector3 *)&m_elems[ 4 * x ];
}

inline const hk_Vector3& hk_Matrix3::get_column(int x) const
{
	return *(hk_Vector3 *)&m_elems[ 4 * x ];
}

#endif //HK_PS2


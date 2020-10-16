
void hk_Dense_Matrix_Util::mult_3_symmetric( const hk_Fixed_Dense_Matrix<3>& m, const hk_Vector3& in, hk_Vector3& out )
{
	const hk_real *co0 = m.get_const_real_pointer();
	const hk_real *co1 = co0 + m.getLda();
	const hk_real *co2 = co1 + m.getLda();

	out.set_mul( in.x, *(hk_Vector3 *)co0 );
	out.add_mul( in.y, *(hk_Vector3 *)co1 );
	out.add_mul( in.z, *(hk_Vector3 *)co2 );
}


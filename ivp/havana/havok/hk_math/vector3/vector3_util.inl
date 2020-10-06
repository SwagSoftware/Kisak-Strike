hk_Vector3 hk_Vector3_Util::random_unit_vec()
{
	hk_Vector3 r( random_vec() );
	r.normalize();
	return r;
}


void hk_Vector3_Util::mult_and_add( hk_Vector3 &v, const hk_Vector3 &factor, const hk_Vector3 &add ){
	//: for (i=0..2)   v(i) = v(i) * factor(i) + add(i)
	hk_real x = v.x * factor.x + add.x;
	hk_real y = v.y * factor.y + add.y;
	hk_real z = v.z * factor.z + add.z;
	v.x = x;
	v.y = y;
	v.z = z;
}

#include <hk_math/vecmath.h>
#ifdef HK_PS2
#	include <hk_math/rotationps2.cpp>
#else

void hk_Rotation::set(const hk_Quaternion& q)
{
	// note the resulting assembler code of this function is just awfull, 
	// rearrangement needed to get a better result
	hk_real x2 = q(0) + q(0);
	hk_real y2 = q(1) + q(1);
	hk_real z2 = q(2) + q(2);

	hk_real xx = q(0)*x2;
	hk_real xy = q(0)*y2;
	hk_real xz = q(0)*z2;
	hk_real yy = q(1)*y2;
	hk_real yz = q(1)*z2;
	hk_real zz = q(2)*z2;

	hk_real wx = q(3)*x2;
	hk_real wy = q(3)*y2;
	hk_real wz = q(3)*z2;

	get_column(0).set(1.0f-(yy+zz),	    xy+wz,			xz-wy);
	get_column(1).set(xy-wz,			1.0f-(xx+zz),	yz+wx);
	get_column(2).set(xz+wy,			yz-wx,			1.0f-(xx+yy));
}

void hk_Rotation::set_axis_angle(const hk_Vector3& axis, hk_real angle)
{
	hk_Quaternion q;
	q.set_axis_angle( axis, angle );
	this->set(q);
}

#endif //HK_PS2


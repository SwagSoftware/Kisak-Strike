#include <hk_math/vecmath.h>
#include <hk_math/quaternion/quaternion_util.h>

void hk_Quaternion_Util::set_mul_unit_dir(const hk_Quaternion& q0, int axis, hk_Quaternion& q_out)
{
	hk_Quaternion q1;
	q1.set_zero4();
	q1(axis) = 1.0f;

	q_out.hk_Vector3::set_cross(q0, q1);

	q_out(axis) += q0.w;	//	hk_Vector3::add_mul(q0.w, q1);
							//	hk_Vector3::add_mul(q1.w, q0);
	q_out.w = -q0(axis);	//	q0.get_real() * q1.get_real() - q0.dot( q1 );
}

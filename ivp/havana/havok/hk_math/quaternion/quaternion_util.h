#ifndef HK_MATH_QUATERNION_UTIL
#define HK_MATH_QUATERNION_UTIL

class hk_Quaternion_Util
{
public:
	static void set_mul_unit_dir(const hk_Quaternion& q0, int axis, hk_Quaternion& q_out);
	//: does: hk_Quaternion h(0,0,0,0); h(axis) = 1;  q_out = q0 * h
};

#endif /* HK_MATH_QUATERNION_UTIL */

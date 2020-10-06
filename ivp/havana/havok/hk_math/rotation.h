#ifndef HK_ROTATION_H
#define HK_ROTATION_H

#ifndef HK_MATH_VECMATH_H
#error Include <hk_math/vecmath.h> Do not include this file directly.
#endif // HK_MATH_VECMATH_H

//: An orthonormal 3x3 matrix
class hk_Rotation : public hk_Matrix3
{
	public:		
		inline hk_Rotation() { }
			//: Empty constructor.
		inline ~hk_Rotation() { }
			//: Empty deconstructor.

		inline hk_Rotation(const hk_Quaternion& q);
			//: Initialize from a normalized quaternion.

		void set(const hk_Quaternion& q);
			//: Set from a normalized quaternion.

		void set_axis_angle(const hk_Vector3& axis, hk_real angle);
			//: Set rotation to be 'angle' radians about the vector 'axis'
};

#endif // HK_ROTATION_H

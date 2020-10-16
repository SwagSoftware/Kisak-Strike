#ifndef HK_MATH_QUATERNION_H
#define HK_MATH_QUATERNION_H

#ifndef HK_MATH_VECMATH_H
#error Include <hk_math/vecmath.h> Do not include this file directly.
#endif // HK_MATH_VECMATH_H

//: A generic 3x3 matrix
class hk_Quaternion : public hk_Vector4
{
	public:

		inline hk_Quaternion();
			//: Empty constructor

		inline hk_Quaternion(hk_real x, hk_real y, hk_real z, hk_real w);
			//: set x y z w

		explicit inline hk_Quaternion(const hk_Rotation& r);
			//: XXX

		inline hk_Quaternion(const hk_Vector3& axis, hk_real angle);
			//: XXX

		inline void set_identity();

		void set(const hk_Rotation& r);
			//: XXX

		inline void set_mul(hk_real r, const hk_Quaternion& q);
		inline void add_mul(hk_real r, const hk_Quaternion& q);

		inline void set_mul(const hk_Quaternion& q0, const hk_Quaternion& q1);

		//inline void set_mul_inv(const hk_Quaternion& q0, const hk_Quaternion& q1);
			//: this = (q0-1) * q1

		void set_axis_angle(const hk_Vector3& axis, hk_real angle);
			//: XXX

		void set_slerp(const hk_Quaternion& q0, const hk_Quaternion& q1, hk_real t);
			//: Set this to be the spherical linear interpolation of q0, q1 at parameter t
		
		inline void normalize_quaternion();
			//: 

		/* element access */

	    inline hk_real& operator() (int a);
			//: XXX
		inline const hk_real& operator() (int a) const;
			//: XXX

		/* access access */

		inline void set_real(hk_real r);
		inline hk_real get_real() const;
			//: XXX

		inline void set_imag(const hk_Vector3& );
		inline const hk_Vector3& get_imag() const;
			//: XXX
};

#endif /* HK_MATH_QUATERNION_H */


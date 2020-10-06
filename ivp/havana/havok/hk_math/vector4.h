#ifndef HK_VECTOR4_H
#define HK_VECTOR4_H

#ifndef HK_MATH_VECMATH_H
#error Include <hk_math/vecmath.h> Do not include this file directly.
#endif // HK_MATH_VECMATH_H

class hk_Vector4 : public hk_Vector3
{
	public:

		inline hk_Vector4();
		inline hk_Vector4(hk_real a, hk_real b, hk_real c, hk_real d);

		inline void operator=  (const hk_Vector4& v);
		inline void operator+= (const hk_Vector4& a);
		inline void operator-= (const hk_Vector4& a);
		inline void operator*= (hk_real a);

		inline void set4(hk_real a, hk_real b, hk_real c, hk_real d);
		inline void set_zero4();

		inline void set_add4(const hk_Vector4& a, const hk_Vector4& b);
		inline void set_sub4(const hk_Vector4& a, const hk_Vector4& b);
		//inline void set_mul(const hk_Vector4& a, const hk_Vector4& b);
		//inline void set_cross(const hk_Vector4& a, const hk_Vector4& b);

		inline void set_mul4(hk_real r, const hk_Vector4& a);
		inline void add_mul4(hk_real r, const hk_Vector4& a);

		/* length and distance */
		inline hk_real dot4(const hk_Vector4& a) const;
		inline hk_real length4() const;
		inline hk_real length_inv4() const;
		inline hk_real length_squared4() const;

		inline void normalize4();
};

#endif // HK_VECTOR4_H


#ifndef HK_MATH_VECTOR3_UTIL
#define HK_MATH_VECTOR3_UTIL

class hk_Vector3_Util
{
public:
	// can highly be optimized for PS2
	static inline void mult_and_add2( hk_Vector3 &v, hk_real factor, const hk_Vector3 &add_a, const hk_Vector3 &add_b ){
		//: v = v * factor + add_a + add_b
		v *= factor;
		v += add_a;
		v += add_b;
	}

	static inline void mult_and_add( hk_Vector3 &v, const hk_Vector3 &factor, const hk_Vector3 &add );
	//: for (i=0..2)   v(i) = v(i) * factor(i) + add(i)

	static hk_Vector3 random_vec();
	//: returns a random vec; hkMath::fabs(v(i)) < 1.0f

	static inline	hk_Vector3 random_unit_vec();
	//: returns a random vector with a length of 1.0f
	
	static	hk_Vector3 perp_vec( const hk_Vector3 &v );
	//: returns a vector orthonormal to v
};

#include <hk_math/vector3/vector3_util.inl>

#endif /*HK_MATH_VECTOR3_UTIL */



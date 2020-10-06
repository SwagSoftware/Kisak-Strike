#ifndef HAK_MATH_VECTOR3_H
#define HAK_MATH_VECTOR3_H

#ifndef HK_MATH_VECMATH_H
#error Include <hk_math/vecmath.h> Do not include this file directly.
#endif // HK_MATH_VECMATH_H

namespace Havok { class Vector3; }

struct hkVector3ExpressionPlus;
struct hkVector3ExpressionMinus;

#define HK_REF &
#define Const //const
#define ConsT const

class hk_Vector3
{
	public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Vector3)

		inline hk_Vector3();
		inline hk_Vector3(hk_real a, hk_real b, hk_real c);
		inline hk_Vector3(const double*);
		inline hk_Vector3(const float*);

		inline hk_Vector3( const hk_Vector3& v);
		inline void operator=  (const hk_Vector3& v);

		/* expression operators */
		inline hk_Vector3(ConsT hkVector3ExpressionPlus HK_REF v);
		inline void operator= (ConsT hkVector3ExpressionPlus HK_REF v);
		inline hk_Vector3(ConsT hkVector3ExpressionMinus HK_REF v);
		inline void operator= (ConsT hkVector3ExpressionMinus HK_REF v);

		/* accumulation operators */
		inline void operator=  (const Havok::Vector3&); //<todo> remove me
		inline void operator+= (const hk_Vector3& a);
		inline void operator-= (const hk_Vector3& a);
		inline void operator*= (hk_real a);

		inline void set(hk_real a, hk_real b, hk_real c);
		inline void set_zero();

		/* arithmetic operations - fast! */
		inline void set_add(const hk_Vector3& a, const hk_Vector3& b);
		inline void set_sub(const hk_Vector3& a, const hk_Vector3& b);
		inline void set_cross(const hk_Vector3& a, const hk_Vector3& b);

		inline void set_mul(hk_real r, const hk_Vector3& a);
		inline void set_mul3(const hk_Diagonal_Matrix& a, const hk_Vector3& b);
		inline void set_mul3( const hk_Matrix3& a, const hk_Vector3& b ); 
		inline void add_mul(hk_real r, const hk_Vector3& a);
		inline void set_interpolate( const hk_Vector3& a, const hk_Vector3& b, hk_real t);

		/* transform */
		void set_rotated_dir    (const hk_Rotation& a, const hk_Vector3& b);
		void set_rotated_inv_dir(const hk_Rotation& a, const hk_Vector3& b);

		void set_transformed_pos    (const hk_Transform& a, const hk_Vector3& b);
		void set_transformed_inv_pos(const hk_Transform& a, const hk_Vector3& b);
    
		/* inline versions of above */
		inline void _set_rotated_dir    (const hk_Rotation& a, const hk_Vector3& b);
		inline void _set_rotated_inv_dir(const hk_Rotation& a, const hk_Vector3& b);

		inline void _set_transformed_pos    (const hk_Transform& a, const hk_Vector3& b);
		inline void _set_transformed_inv_pos(const hk_Transform& a, const hk_Vector3& b);

		/* length and distance */
		inline hk_real dot(const hk_Vector3& a) const;
		inline hk_real length() const;
		inline hk_real length_inv() const;
		inline hk_real length_squared() const;

		inline hk_real distance_squared_to( const hk_Vector3 &) const;
		inline hk_real distance_to( const hk_Vector3 &) const;

		inline void normalize();
		inline hk_real normalize_with_length();
		//void fast_normalize_e10(); // 10 bits valid

        /* element access */
	    inline hk_real& operator() (int a);
		inline const hk_real& operator() (int a) const;

		inline       hk_real* get_real_pointer() { return &x; }
		inline const hk_real* get_real_pointer() const { return &x; }

	public:

		hk_real HK_ALIGNED_VARIABLE(x,16);
		hk_real y;
		hk_real z;
		hk_real w;
};

inline Const hkVector3ExpressionPlus operator+ (const hk_Vector3& a, const hk_Vector3& b);
inline Const hkVector3ExpressionMinus operator- (const hk_Vector3& a, const hk_Vector3& b);

#endif // HAK_MATH_VECTOR3_H

#ifndef HK_SPATIAL_VECTOR_H
#define HK_SPATIAL_VECTOR_H

#ifndef HK_MATH_VECMATH_H
#include <hk_math/vecmath.h>
#endif // HK_MATH_VECMATH_H

class hk_Spatial_Matrix;

class hk_Spatial_Vector
{
	public:

		inline hk_Spatial_Vector();		
		inline hk_Spatial_Vector( const hk_Spatial_Vector& v);
		inline hk_Spatial_Vector(hk_real a, hk_real b, hk_real c, hk_real d, hk_real e, hk_real f);
		inline hk_Spatial_Vector(hk_Vector3 &a, hk_Vector3 &b ); 

		/* accumulation operators - fast! */
		inline hk_Spatial_Vector& operator=  (const hk_Spatial_Vector& v);
		inline void operator+= (const hk_Spatial_Vector& a);
		inline void operator-= (const hk_Spatial_Vector& a);
		inline void operator*= (hk_real a);

		inline void set(hk_real a, hk_real b, hk_real c, hk_real d, hk_real e, hk_real f);
		inline void set_top( const hk_Vector3 &a );
		inline void set_bottom( const hk_Vector3 &b );
		inline void set_zero();

		/* arithmetic operations - fast! */
		inline void set_add(const hk_Spatial_Vector& a, const hk_Spatial_Vector& b);
		inline void set_sub(const hk_Spatial_Vector& a, const hk_Spatial_Vector& b);

		inline void set_mul(hk_real r, const hk_Spatial_Vector& a);
		inline void add_mul(hk_real r, const hk_Spatial_Vector& a);
		void set_mul(const hk_Spatial_Matrix& a, const hk_Spatial_Vector& b);

		/* arithmetic operators - slow! */
		inline hk_Spatial_Vector operator+ (const hk_Spatial_Vector& v);
		inline hk_Spatial_Vector operator- (const hk_Spatial_Vector& a);

		/* spatial operations */
		inline hk_real dot(const hk_Spatial_Vector& a) const;
		void set_transformed_pos(const hk_Spatial_Matrix& a, const hk_Spatial_Vector& b);
		inline void _set_transformed_pos(const hk_Spatial_Matrix& a, const hk_Spatial_Vector& b);

        /* element access */
	    inline hk_real& operator() (int a);
		inline const hk_real& operator() (int a) const;

	public:

		hk_Vector3 top;  // angular accel, linear force
		hk_Vector3 bottom;  // linear accel, torque
};

#include <hk_math/spatial_vector.inl>

#endif //HK_SPATIAL_VECTOR_H

#ifndef HK_TRANSFORM_H
#define HK_TRANSFORM_H

#ifndef HK_MATH_VECMATH_H
#error Include <hk_math/vecmath.h> Do not include this file directly.
#endif // HK_MATH_VECMATH_H

//: An orthonormal rotation with translation.
// Note: some methods depend on the rotation component being orthonormal.
// If the user changes the values directly (ie either through set_cols
// or set_elem) some of these methods will silently fail or produce
// bad results.
class hk_Transform : public hk_Rotation
{
	public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Transform)

		inline hk_Transform();
			//: Empty default constructor.

		hk_Transform(const hk_Quaternion &q, const hk_Vector3 &translation);

		void set_transform(const hk_Quaternion &q, const hk_Vector3 &translation);
			//: XXX
 
		inline void set_qtransform( const hk_QTransform& q );
		inline void set_rotation( const hk_Quaternion& q );

		void set_identity_transform();
			//: XXX

		inline hk_Vector3& get_translation();
		inline const hk_Vector3& get_translation() const;
			//: XXX

		inline void set_translation(const hk_Vector3& t0);
			//: XXX

		//hk_Transform_Utility& utility(){ return (hk_Transform_Utility&)(*this); }

		void set_cols4( const hk_Vector3& c0,
					   const hk_Vector3& c1,
					   const hk_Vector3& c2,
					   const hk_Vector3& c3);
			//:
		inline void _set_interpolate( hk_QTransform &a, hk_QTransform &b , hk_real t);
		void set_interpolate( hk_QTransform &a, hk_QTransform &b , hk_real t);

		void get_4x4_column_major(hk_real* p) const;
			//: write a 4x4 column major matrix into p
		

	protected:

		hk_Vector3 m_translation;
};


// example of the utility class for hk_Transform:
//class hk_Transform_Utility : public hk_Transform
//{
//	public:
//		void calc_eigen_vectors();
//};

#endif // HK_TRANSFORM_H

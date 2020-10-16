#ifndef HK_MATH_QTRANSFORM_H
#define HK_MATH_QTRANSFORM_H

//:
class hk_QTransform
{
	public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_QTransform)

		inline hk_QTransform();
			//:

		inline hk_QTransform(const hk_Vector3& axis, hk_real angle,
							const hk_Vector3& translate);
			//:

		inline void set_identity();
			//:


		inline void add_mul( hk_real r, const hk_QTransform& t);

		inline const hk_Vector3& get_translation() const;
		inline const hk_Quaternion& get_rotation() const;
			//: XXX

		inline void set_translation(const hk_Vector3& t0);
			//: XXX

		inline void set_interpolate(	const hk_QTransform& a,
										const hk_QTransform& b,
										hk_real t);
			//: XXX

	//private:
		hk_Quaternion	m_rotation;
		hk_Vector3		m_translation;
};

#endif /* HK_MATH_QTRANSFORM_H */

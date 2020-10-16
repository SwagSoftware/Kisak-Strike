#ifndef HK_MATH_RAY_H
#define HK_MATH_RAY_H

#include <hk_math/vecmath.h>

class hk_Ray
{
    public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Ray)

		inline hk_Ray() { }
		inline hk_Ray(const hk_Vector3& orig, const hk_Vector3 dir);

		inline void set_zero();
		inline void set(const hk_Vector3& o, const hk_Vector3& d);

		inline const hk_Vector3& get_origin() const { return m_origin; }
		inline const hk_Vector3& get_direction() const { return m_direction; }

    public:
		hk_Vector3 m_origin;
		hk_Vector3 m_direction;
};

#include <hk_math/ray.inl>

#endif /*HK_MATH_RAY_H*/

#ifndef HK_MATH_PLANE_H
#define HK_MATH_PLANE_H

//: XXX
class hk_Plane
{
	public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Plane)

		inline hk_Plane();
			//: Empty default constructor.

		inline hk_Plane(const hk_Vector3& normal, hk_real distance);
			//: Initialize with plane normal and perpendicular distance to the origin.

		hk_Plane(const hk_Vector3& p0, const hk_Vector3& p1, const hk_Vector3& p2);
			//: Set to be plane defined by three non-colinear points in the plane.

		void set(const hk_Vector3& normal, hk_real distance);
			//: Initialize with plane normal and perpendicular distance to the origin.
			
		void set(const hk_Vector3& p0, const hk_Vector3& p1, const hk_Vector3& p2);		
			//: Set to be plane defined by three non-colinear points in the plane.

	private:
		
		hk_Vector4 m_plane;
};

#endif /* HK_MATH_PLANE_H */


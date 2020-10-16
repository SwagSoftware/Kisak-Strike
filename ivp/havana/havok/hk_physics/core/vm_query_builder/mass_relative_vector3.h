#ifndef HK_PHYSICS_MASS_RELATIVE_VECTOR3
#define HK_PHYSICS_MASS_RELATIVE_VECTOR3

// IVP_EXPORT_PUBLIC

class hk_Mass_Relative_Vector3 
{
	public:

	HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Mass_Relative_Vector3)

	hk_Vector3 m_vector;
	inline hk_Mass_Relative_Vector3(){;}
	inline hk_Mass_Relative_Vector3( hk_Rigid_Body *rb, const hk_Vector3 &position_ws )
	{
		m_vector.set_sub( position_ws, rb->get_center_of_mass() );
	}

	inline void set( hk_Rigid_Body* rb, const hk_Vector3& position_ws )
	{
		m_vector.set_sub( position_ws, rb->get_center_of_mass() );
	}
};

#endif /*HK_PHYSICS_MASS_RELATIVE_VECTOR3*/


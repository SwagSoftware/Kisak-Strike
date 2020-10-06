#ifndef HK_PHYSICS_BREAKABLE_BP_H
#define HK_PHYSICS_BREAKABLE_BP_H


// IVP_EXPORT_PUBLIC

class hk_Breakable_Constraint_BP  //: public hk_Effector_BP 
{
	public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Breakable_Constraint_BP)

		inline hk_Breakable_Constraint_BP()
			:	m_real_constraint(HK_NULL),
				m_linear_strength(0),
				m_angular_strength(0)
		{			
		}

	public: 

		hk_Constraint* m_real_constraint;
		hk_real m_linear_strength;
		hk_real m_angular_strength;

	protected:

		hk_id		m_bodies[2];
};

#endif /* HK_PHYSICS_LOCAL_BREAKABLE_BP_H*/

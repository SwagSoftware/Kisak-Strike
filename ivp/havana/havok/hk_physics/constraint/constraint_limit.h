#ifndef HK_PHYSICS_CONSTRAINT_LIMIT
#define HK_PHYSICS_CONSTRAINT_LIMIT

// IVP_EXPORT_PUBLIC

class hk_Constraint_Limit_BP
{
	public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Constraint_Limit_BP)

		hk_Constraint_Limit_BP()
		{
			m_limit_is_enabled = HK_FALSE;
			m_friction_is_enabled = HK_FALSE;
			m_limit_min = 0.0f;
			m_limit_max = 0.0f;
			m_limit_tau = 1.0f;
		}

		void set_limits(hk_real lower, hk_real upper)
		{
			m_limit_is_enabled = HK_TRUE;
			m_limit_min = lower;
			m_limit_max = upper;
		}

		void set_friction( hk_real friction)
		{
			m_friction_is_enabled = (friction!=0.0f)?HK_TRUE: HK_FALSE;
			m_joint_friction = friction;
			m_desired_velocity = 0.0f;
		}

		void set_motor( hk_real desired_vel, hk_real max_force)
		{
			m_friction_is_enabled = (max_force!=0.0f)?HK_TRUE: HK_FALSE;
			m_joint_friction = max_force;
			m_desired_velocity = desired_vel;
		}

	public:

		hk_bool m_limit_is_enabled;
		hk_bool	m_friction_is_enabled;

		hk_real m_limit_min;
		hk_real m_limit_max;
		hk_real m_limit_tau;		// the stiffness of the constraint
		hk_real	m_joint_friction;	// [Nm] == max force for motor type
		hk_real	m_desired_velocity; // allows to build motors
};


class hk_Constraint_Limit: public hk_Constraint_Limit_BP
{
	public:

		inline hk_Constraint_Limit()
			: m_ref_position(0.0f)
		{			
		}

		void init_limit(const hk_Constraint_Limit_BP &bp, hk_real eps)
		{
			m_limit_is_enabled = bp.m_limit_is_enabled;		
			m_friction_is_enabled = bp.m_friction_is_enabled;		
			m_limit_min = bp.m_limit_min;		
			m_limit_max = bp.m_limit_max;		
			m_limit_tau = bp.m_limit_tau;		
			m_joint_friction = bp.m_joint_friction * eps;		
			m_desired_velocity = bp.m_desired_velocity * eps;		
		}

		void init_angular_limit(const hk_Constraint_Limit_BP &bp, hk_real eps)
			//: limit for angular limits
			// also checks, if limits make sense
		{
			m_limit_is_enabled = bp.m_limit_is_enabled;		
			m_friction_is_enabled = bp.m_friction_is_enabled;		
			m_limit_min = bp.m_limit_min;		
			m_limit_max = bp.m_limit_max;		
			m_limit_tau = bp.m_limit_tau;		
			m_joint_friction = bp.m_joint_friction * eps;		
			m_desired_velocity = bp.m_desired_velocity * eps;		
			if ( m_limit_max - m_limit_min > HK_PI * 2.0f){
				m_limit_is_enabled = HK_FALSE;
			}
		}

		hk_real m_ref_position;
};


#endif /*HK_PHYSICS_CONSTRAINT_LIMIT*/


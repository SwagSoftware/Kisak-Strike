#ifndef HK_PHYSICS_HINGE_BP_H
#define HK_PHYSICS_HINGE_BP_H

// IVP_EXPORT_PUBLIC

class hk_Hinge_BP  //: public hk_Effector_BP 
{
	public:

		inline hk_Hinge_BP()
		{
			m_strength = 1.0f;
			m_tau = 1.0f;
			m_axis_os[0].set_zero();
			m_axis_os[1].set_zero();
			m_axis_perp_os[0].set_zero();
			m_axis_perp_os[1].set_zero();
		}

		void set_axis_os( int obj_idx, const hk_Ray& axis_os);

	public: // hack for now

		hk_real     m_tau;	
		hk_real		m_strength;

		hk_Constraint_Limit_BP m_limit;

		hk_Ray		m_axis_os[2];
		hk_Vector3	m_axis_perp_os[2]; // normalised perpendicular axes for limits

	protected:

		hk_id		m_bodies[2];
};

#endif /* HK_PHYSICS_HINGE_BP_H*/


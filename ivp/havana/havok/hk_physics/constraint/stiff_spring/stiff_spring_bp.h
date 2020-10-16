#ifndef HK_PHYSICS_STIFF_SPRING_BP_H
#define HK_PHYSICS_STIFF_SPRING_BP_H

class hk_Stiff_Spring_BP  //: public hk_Constraint_BP 
{
	public:
		inline hk_Stiff_Spring_BP()
		{
			m_tau = 0.3f;
			m_strength = 0.5f;
			m_length = 1.0f;
			m_translation_os_ks[0].set(0,0,0);
			m_translation_os_ks[1].set(0,0,0);
		}

		void set_position_os ( int, hk_Vector3 &position_os);

	public: // hack for now

		hk_real		m_length;
		hk_real     m_tau;
		hk_real		m_strength;
		hk_Vector3	m_translation_os_ks[2];
		hk_bool		m_is_rigid;

	protected:

		hk_id		m_bodies[2];
};

#endif /* HK_PHYSICS_SPRING_BP_H */


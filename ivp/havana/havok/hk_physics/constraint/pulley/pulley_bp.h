#ifndef HK_PULLEY_BP_H
#define HK_PULLEY_BP_H

class hk_Pulley_BP  //: public hk_Constraint_BP 
{
	public:
		inline hk_Pulley_BP()
		{
			m_tau = 1.0f;
			m_strength = 1.0f;
			m_length = 1.0f;
			m_gearing = 1.0f;
			m_is_rigid = false;
			m_translation_os_ks[0].set(0,0,0);
			m_translation_os_ks[1].set(0,0,0);
			m_worldspace_point[0].set(0,0,0);
			m_worldspace_point[1].set(0,0,0);
		}

		//void set_position_os ( int, hk_Vector3& position_os);

	public: // hack for now

		hk_real		m_length;
		hk_real    	m_tau;
		hk_real		m_strength;
		hk_real		m_gearing;
		hk_Vector3	m_translation_os_ks[2];
		hk_Vector3	m_worldspace_point[2];
		hk_bool		m_is_rigid;

	protected:

		hk_id		m_bodies[2];
};

#endif // HK_PULLEY_BP_H


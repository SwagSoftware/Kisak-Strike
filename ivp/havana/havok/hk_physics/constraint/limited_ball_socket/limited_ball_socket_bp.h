#ifndef HK_PHYSICS_LIMITED_BALL_SOCKET_BP_H
#define HK_PHYSICS_LIMITED_BALL_SOCKET_BP_H

// IVP_EXPORT_PUBLIC
class hk_Limited_Ball_Socket_BP  //: public hk_Effector_BP 
{
	public:

		inline hk_Limited_Ball_Socket_BP()
		{
			m_strength	= 1.0f;
			m_tau		= 1.0f;
			m_transform_os_ks[0].set_identity_transform();
			m_transform_os_ks[1].set_identity_transform();
			for(int i=0; i<3; i++)
			{
				m_angular_limits[i].m_min = 0;
				m_angular_limits[i].m_max = 0;
			}
			m_constrainTranslation = true;
		}

		void set_position_os ( int, hk_Vector3 &position_os);

	
	public: // hack for now //private: 

		hk_Transform m_transform_os_ks[2];
		//hk_Vector3	m_position_os[2];
		//hk_Matrix3	m_joint_axes[2];
		hk_real		m_strength;
		hk_real     m_tau;
		hk_Interval<hk_real> m_angular_limits[3];

	protected:

		hk_id		m_bodies[2];
	public:
		bool		m_constrainTranslation;
};

#endif /* HK_PHYSICS_LOCAL_LIMITED_BALL_SOCKET_BP_H */

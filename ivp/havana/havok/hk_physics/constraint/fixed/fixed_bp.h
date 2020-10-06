// TKBMS v1.0 -----------------------------------------------------
//
// PLATFORM		: ALL
// PRODUCT		: VPHYSICS
// VISIBILITY	: INTERNAL
//
// ------------------------------------------------------TKBMS v1.0

#ifndef HK_FIXED_BP_H
#define HK_FIXED_BP_H

// IVP_EXPORT_PUBLIC
class hk_Fixed_BP  //: public hk_Effector_BP 
{
	public:

		inline hk_Fixed_BP()
		{
			m_strength	= 1.0f;
			m_tau		= 1.0f;
			m_transform_os_ks.set_identity_transform();
		}
	
	public: // hack for now //private: 

		hk_Transform m_transform_os_ks;
		hk_real		m_strength;
		hk_real     m_tau;

	protected:

		hk_id		m_bodies[2];
};

#endif /* HK_PHYSICS_LOCAL_LIMITED_BALL_SOCKET_BP_H */

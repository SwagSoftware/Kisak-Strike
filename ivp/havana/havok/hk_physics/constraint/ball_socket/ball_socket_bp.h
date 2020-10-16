#ifndef HK_PHYSICS_BALL_SOCKET_BP_H
#define HK_PHYSICS_BALL_SOCKET_BP_H


// IVP_EXPORT_PUBLIC

class hk_Ball_Socket_BP  //: public hk_Effector_BP 
{
	public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Ball_Socket_BP)

		inline hk_Ball_Socket_BP()
		{
			m_strength = 1.0f;
			m_tau = 1.0f;
			m_translation_os_ks[0].set(0,0,0);
			m_translation_os_ks[1].set(0,0,0);
		}

		void set_position_os ( int bodyIndex, hk_Vector3 &position_os )
		{
			m_translation_os_ks[bodyIndex] = position_os;
		}

	public: // hack for now

		hk_real     m_tau;
		hk_real		m_strength;
		hk_Vector3	m_translation_os_ks[2];

	protected:

		hk_id		m_bodies[2];
};

#endif /* HK_PHYSICS_LOCAL_BALL_SOCKET_BP_H*/

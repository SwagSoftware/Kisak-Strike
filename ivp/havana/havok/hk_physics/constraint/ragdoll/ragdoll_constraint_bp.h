#ifndef HK_PHYSICS_RAGDOLL_CONSTRAINT_BP_H
#define HK_PHYSICS_RAGDOLL_CONSTRAINT_BP_H

// IVP_EXPORT_PUBLIC

enum {
		HK_LIMIT_TWIST,
		HK_LIMIT_CONE,
		HK_LIMIT_PLANES,
		HK_NLIMIT
};

class hk_Ragdoll_Constraint_BP  //: public hk_Effector_BP 
{
	public:

		inline hk_Ragdoll_Constraint_BP()
		{
			m_transform_os_ks[0].set_identity_transform();
			m_transform_os_ks[1].set_identity_transform();

			m_strength			= 1.0f;
			m_tau				= 1.0f;
			m_constrainTranslation = true;
		}

		void set_position_os ( int, hk_Vector3 &position_os);

	//private: 
	public: // hack for now
		hk_Transform m_transform_os_ks[2];


		hk_Constraint_Limit_BP	m_limits[HK_NLIMIT];  // first is twist, then cone, than planes

		hk_real		m_strength;
		hk_real     m_tau;
		bool		m_constrainTranslation;
};

#endif /* HK_PHYSICS_RAGDOLL_CONSTRAINT_BP_H */

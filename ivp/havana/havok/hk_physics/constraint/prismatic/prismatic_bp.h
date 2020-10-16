#ifndef HK_PHYSICS_PRISMATIC_BP_H
#define HK_PHYSICS_PRISMATIC_BP_H

class hk_Prismatic_BP  //: public hk_Effector_BP 
{
	public:

		inline hk_Prismatic_BP()
		{
			m_strength	= 1.0f;
			m_tau		= 1.0f;
			m_transform_Ros_Aos.set_identity();
			m_axis_Ros.set(1.0f,0,0);
		}

		void set_position_os ( int, hk_Vector3 &position_os);

	protected:

		hk_id		m_bodies[2];

	//private: 
	public: // hack for now

		hk_QTransform m_transform_Ros_Aos;

		hk_real m_strength;
		hk_real m_tau;

		hk_Vector3			m_axis_Ros;
		hk_Constraint_Limit m_limit;
};

#endif /* HK_PHYSICS_LOCAL_PRISMATIC_BP_H */

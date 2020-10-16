#ifndef HK_PHYSICS_CAR_WHEEL_BP_H
#define HK_PHYSICS_CAR_WHEEL_BP_H

class hk_Car_Wheel_BP  //: public hk_Effector_BP 
{
	public:

		inline hk_Car_Wheel_BP()
		{
			m_damp = 1.0f;
			m_tau = 1.0f;
			m_hinge_axis_os[0].set(1,0,0);
			m_hinge_axis_os[1].set(1,0,0);
			m_translation_os_ks[0].set_zero();
			m_translation_os_ks[1].set_zero();
			m_steering_axis_Ros.set(0,1,0);
		}

		void set_axis_os( int obj_idx, const hk_Ray& axis_os);

	public: // hack for now (as we do not have builders)

		hk_real     m_tau;	
		hk_real		m_damp;	

		hk_Constraint_Limit_BP m_suspension_limit;
		hk_Constraint_Limit_BP m_wheel_limit;

		hk_Vector3		m_translation_os_ks[2];
		hk_Vector3		m_hinge_axis_os[2];
		hk_Vector3		m_steering_axis_Ros;	
};

#endif /* HK_PHYSICS_CAR_WHEEL_BP_H */

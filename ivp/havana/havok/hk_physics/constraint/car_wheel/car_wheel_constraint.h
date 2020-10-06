#ifndef HK_PHYSICS_CAR_WHEEL_CONTRAINT_H
#define HK_PHYSICS_CAR_WHEEL_CONTRAINT_H


class hk_Car_Wheel_BP;

class hk_Car_Wheel_Constraint : public hk_Constraint
{
	public:

		hk_Car_Wheel_Constraint( hk_Environment*, const hk_Car_Wheel_BP* ,
							hk_Rigid_Body *a ,hk_Rigid_Body *b );	
		hk_Car_Wheel_Constraint( hk_Local_Constraint_System *, const hk_Car_Wheel_BP *, hk_Rigid_Body *a ,hk_Rigid_Body *b );	

		virtual int get_vmq_storage_size();

		virtual int	setup_and_step_constraint(
				hk_PSI_Info& pi,
				void *mem,
				hk_real tau_factor,
				hk_real damp_factor );

		virtual void step_constraint(
				hk_PSI_Info& pi,
				void *mem,
				hk_real tau_factor,
				hk_real damp_factor );

		void apply_effector_PSI( hk_PSI_Info&, hk_Array<hk_Entity*>* );

		void set_steering_angle( hk_real angle);

		inline void set_wheel_motor( hk_real angular_velocity, hk_real max_force);
	
		virtual void init_constraint(const void*);

	protected:

		void init_car_wheel_constraint(const hk_Car_Wheel_BP*);

		hk_real     m_tau;	
		hk_real		m_damp;

		hk_Constraint_Limit m_suspension_limit;
		hk_Constraint_Limit m_wheel_limit;

		hk_Vector3		m_translation_os_ks[2];
		hk_Vector3		m_hinge_axis_os[2];
		hk_Vector3		m_steering_axis_Ros;	
		hk_Vector3		m_perp_hinge_axis_Aos;		// used for wheel motor
		hk_Vector3		m_initial_hinge_axis_Ros;   // used to recalculate the m_steering_axis_Ros using the set_steering_angle() function
};

#include <hk_physics/constraint/car_wheel/car_wheel_constraint.inl>

#endif /* HK_PHYSICS_CAR_WHEEL_CONTRAINT_H */


#ifndef HK_PHYSICS_HINGE_BP_BUILDER_H
#define HK_PHYSICS_HINGE_BP_BUILDER_H

#ifndef HK_PHYSICS_HINGE_BP_H
#	include <hk_physics/constraint/hinge/hinge_bp.h>
#endif

// IVP_EXPORT_PUBLIC

class hk_Hinge_BP_Builder {
protected:
	hk_Hinge_BP m_hinge_bp;
public:
	void set_position_os( int body_index, const hk_Vector3 &position );
	void set_axis_ws( hk_Rigid_Body *a, hk_Rigid_Body *b, const hk_Vector3& axis_ws );
	//lwss add
    void set_axis_perp_os( int obj_index, const hk_Vector3& perp_os );
    //lwss end
	void set_tau( hk_real );
	void set_joint_friction(hk_real Nm);
		// sets a joint friction === set_angular_motor( 0.0f, Nm )

	void set_angular_motor(hk_real angular_velocity, hk_real max_torque);
		// sets the joint friction attached to an angular motor

	void set_angular_limits( hk_real lower, hk_real upper );
	const hk_Hinge_BP *get_blueprint(){ return &m_hinge_bp; };
};

#endif /*HK_PHYSICS_HINGE_BP_BUILDER_H*/


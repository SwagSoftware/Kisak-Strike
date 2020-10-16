#ifndef HK_PHYSICS_RAGDOLL_CONSTRAINT_BP_BUILDER_H
#define HK_PHYSICS_RAGDOLL_CONSTRAINT_BP_BUILDER_H

#ifndef HK_PHYSICS_RAGDOLL_CONSTRAINT_BP_H
#	include <hk_physics/constraint/ragdoll/ragdoll_constraint_bp.h>
#endif

// IVP_EXPORT_PUBLIC

class hk_Ragdoll_Constraint_BP_Builder {
protected:
	hk_Ragdoll_Constraint_BP m_ragdoll_constraint_bp;
public:
	hk_result initialize_from_limited_ball_socket_bp( const class hk_Limited_Ball_Socket_BP *, hk_Rigid_Body *a, hk_Rigid_Body *b);

	void set_ragdoll_constraint( 
		hk_Rigid_Body *a, hk_Rigid_Body *b,
		const hk_Vector3 &pivot_point_ws,
		const hk_Vector3 &primary_axis_ws, const hk_Vector3 &planes_axis_ws,
		const hk_Interval<hk_real> &twist_limits,	// -pi, pi
		const hk_Interval<hk_real> &cone_limits,	// difference >20 , <340
		const hk_Interval<hk_real> &plane_limits,	// values <80
		bool limitTranslation = true
	);

	const hk_Ragdoll_Constraint_BP *get_blueprint(){ return &m_ragdoll_constraint_bp; };
};

#endif /*HK_PHYSICS_RAGDOLL_CONSTRAINT_BP_BUILDER_H*/


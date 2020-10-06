
#ifndef HK_PHYSICS_CONSTRAINT_LIMIT_UTIL_H
#define HK_PHYSICS_CONSTRAINT_LIMIT_UTIL_H

// IVP_EXPORT_PUBLIC
class hk_Constraint_Limit_Util {
public:

	static inline void do_angular_limit(
		hk_PSI_Info& pi,
		hk_Rigid_Body *b0,
		hk_Vector3 &rot_axis_Ref_ws,		// the axis the objects are rotating around
		hk_real    alpha,				// the current angle
		hk_Rigid_Body *b1,
		hk_Constraint_Limit &limit,
		hk_real		tau_factor,
		hk_real		damp_factor
		)
	{

		hk_VM_Query_Builder< hk_VMQ_Storage<1> > query;
		query.begin(1);
		{
			query.begin_entries(1);
			// angular
			query.add_angular( 0, HK_BODY_A, b0, rot_axis_Ref_ws,  1.0f );
			query.add_angular( 0, HK_BODY_B, b1, rot_axis_Ref_ws, -1.0f );
			query.commit_entries(1);
		}
		query.commit( HK_BODY_A, b0 );
		query.commit( HK_BODY_B, b1 );

		hk_real impulse = 0.0f;
		hk_real virtual_mass = 1.0f / query.get_vmq_storage().get_dense_matrix()(0,0);

		if (limit.m_friction_is_enabled){

			hk_real d_alpha = limit.m_ref_position - alpha;
			limit.m_ref_position += limit.m_desired_velocity * pi.get_delta_time();

			if ( d_alpha > HK_PI ) {
				d_alpha -= HK_PI * 2.0f;
				// don't reset the reference position, and adjust alpha to be in the correct range if it has flipped over
//				limit.m_ref_position -= HK_PI * 2.0f;
				alpha += HK_PI * 2.0f;
			}

			if ( d_alpha < -HK_PI ) {
				d_alpha += HK_PI * 2.0f;
				// don't reset the reference position, and adjust alpha to be in the correct range if it has flipped over
//				limit.m_ref_position += HK_PI * 2.0f;
				alpha -= HK_PI * 2.0f;
			}

			const hk_real friction_tau = 0.8f * tau_factor;
			const hk_real friction_damp = damp_factor;
			hk_real angular_correction_per_psi = d_alpha * friction_tau * pi.get_inv_delta_time() -
													friction_damp * query.get_vmq_storage().get_velocities()[0];

			impulse = angular_correction_per_psi * virtual_mass;

			if ( hk_Math::fabs( impulse ) * pi.get_inv_delta_time() > limit.m_joint_friction ){
				hk_real factor = limit.m_joint_friction/ hk_Math::fabs( impulse ) * pi.get_delta_time();

				impulse *= factor;
				limit.m_ref_position -= (1.0f - factor) * d_alpha;
			}
		}

		if ( limit.m_limit_is_enabled ){ // do the limits

			hk_real angular_vel = query.get_vmq_storage().get_velocities()[0] + impulse * query.get_vmq_storage().get_dense_matrix()(0,0);

			hk_real next_alpha = alpha + angular_vel * pi.get_delta_time();
			if ( next_alpha > limit.m_limit_max ){
				hk_real delta = (next_alpha - limit.m_limit_max) * limit.m_limit_tau * tau_factor * pi.get_inv_delta_time();
				impulse -= virtual_mass * delta;
			}else if ( next_alpha < limit.m_limit_min) {
				hk_real delta = (next_alpha - limit.m_limit_min) * limit.m_limit_tau * tau_factor * pi.get_inv_delta_time();
				impulse -= virtual_mass * delta;
			}
		}
		if (impulse){
			hk_real impulses[1];
			impulses[0] = impulse;
			query.apply_impulses( HK_BODY_A, b0, impulses );
			query.apply_impulses( HK_BODY_B, b1, impulses );
		}
	}

	static inline void init_angular_limit(
		hk_Vector3 &cos_axis_Ref_ws,	// the axis of Ref object which is used to sinus measure the angle
		hk_Vector3 &sin_axis_Ref_ws,	// the axis of Ref object which is used to cosin measure the angle
		hk_Constraint_Limit &limit,
		hk_Vector3 &axis_Att_ws
		)
	{
		hk_real sin_alpha = axis_Att_ws.dot( sin_axis_Ref_ws );
		hk_real cos_alpha = axis_Att_ws.dot( cos_axis_Ref_ws );
			
		// don't reset the reference position
		hk_real proposed_ref_position = hk_Math::atan2( sin_alpha, cos_alpha );
		
		if(proposed_ref_position < 0.0f && limit.m_ref_position > 0.0f)
		{
			if((limit.m_ref_position - proposed_ref_position) > HK_PI)
			{
				limit.m_ref_position = proposed_ref_position + 2.0f * HK_PI;
			}
		}

		if(proposed_ref_position > 0.0f && limit.m_ref_position < 0.0f)
		{
			if((proposed_ref_position - limit.m_ref_position) > HK_PI)
			{
				limit.m_ref_position = proposed_ref_position - 2.0f * HK_PI;
			}
		}
	}

	static inline void init_angular_limit(
		hk_Constraint_Limit &limit,
		hk_real alpha
		)
	{
		limit.m_ref_position = alpha;
	}



	static inline void do_linear_limit(
		hk_PSI_Info& pi,
		hk_Rigid_Body *b0,
		hk_Rigid_Body *b1,
		hk_Vector3 &pos_ws,
		hk_Vector3 &axis_ws,		// the axis the objects are rotating around
		hk_Vector3 &delta_ws,		// the axis of Att object which is used to measure the angle (should be equal to the sin_axis to get zero angles
		hk_Constraint_Limit &limit,
		hk_real		tau_factor,
		hk_real		damp_factor
		)
	{
		hk_real alpha = axis_ws.dot( delta_ws );

		hk_VM_Query_Builder< hk_VMQ_Storage<1> > query;
		query.begin(1);
		{
			query.begin_entries(1);
			query.add_linear( 0, HK_BODY_A, b0, pos_ws, axis_ws,  1.0f );
			query.add_linear( 0, HK_BODY_B, b1, pos_ws, axis_ws, -1.0f );
			query.commit_entries(1);
		}

		query.commit( HK_BODY_A, b0 );
		query.commit( HK_BODY_B, b1 );

		hk_real impulse = 0.0f;
		hk_real virtual_mass = 1.0f / query.get_vmq_storage().get_dense_matrix()(0,0);

		if (limit.m_friction_is_enabled){
			hk_real d_alpha = alpha - limit.m_ref_position;

			limit.m_ref_position += limit.m_desired_velocity * pi.get_delta_time();

			const hk_real friction_tau = 0.8f * tau_factor;
			const hk_real friction_damp = 1.0f * damp_factor;
			hk_real correction_per_psi = d_alpha * friction_tau * pi.get_inv_delta_time() -
													friction_damp * query.get_vmq_storage().get_velocities()[0];


			impulse = correction_per_psi * virtual_mass;

			if ( hk_Math::fabs( impulse ) * pi.get_inv_delta_time() > limit.m_joint_friction ){
				hk_real factor = limit.m_joint_friction/ hk_Math::fabs( impulse ) * pi.get_delta_time();

				impulse *= factor;
				limit.m_ref_position += (1.0f - factor) * d_alpha;
			}
		}

		if ( limit.m_limit_is_enabled ){ // do the limits
			
			hk_real vel = query.get_vmq_storage().get_velocities()[0] + impulse * query.get_vmq_storage().get_dense_matrix()(0,0);

			hk_real next_pos = alpha - vel * pi.get_delta_time();
			if ( next_pos > limit.m_limit_max ){
				hk_real delta = (next_pos - limit.m_limit_max) * tau_factor * limit.m_limit_tau * pi.get_inv_delta_time();
				impulse += virtual_mass * delta;
			}else if ( next_pos < limit.m_limit_min) {
				hk_real delta = (next_pos - limit.m_limit_min) * tau_factor * limit.m_limit_tau * pi.get_inv_delta_time();
				impulse += virtual_mass * delta;	
			}
		}
		if (impulse){
			hk_real impulses[1];
			impulses[0] = impulse;
			query.apply_impulses( HK_BODY_A, b0, impulses );
			query.apply_impulses( HK_BODY_B, b1, impulses );
		}
	}

	static inline void init_linear_limit(
		hk_Vector3 &axis_ws,
		hk_Vector3 &delta,
		hk_Constraint_Limit &limit
		)
	{
		limit.m_ref_position = delta.dot( axis_ws );
	}


	static inline void do_angular_plane_limit(
		// a fake angular constraint based on a linear direction clipping against a plane
		hk_PSI_Info& pi,
		hk_Rigid_Body *b0,
		hk_Rigid_Body *b1,
		hk_Vector3  &axis_ws,		// the axis the objects are rotating around
		hk_Vector3	&plane_normal_ws,		// the plane we are clipping 
		hk_real	    alpha,			// the current distance (e.g. initialized with axis_ws.dot( plane_normal_ws )
		hk_Constraint_Limit &limit,
		hk_real		tau_factor,
		hk_real		damp_factor
		)
	{
		hk_VM_Query_Builder< hk_VMQ_Storage<1> > query;
		query.begin(1);
		{
			hk_Vector3 cross; cross.set_cross( axis_ws, plane_normal_ws );
			hk_real len2 = cross.length_squared();
			if (len2 < 0.01f){			// happens because force direction might get parallel
				if (len2 <= HK_REAL_EPS){
					return;
				}
				cross.normalize();		// clip length to 0.1f
				cross *= 0.1f;
			}

			query.begin_entries(1);
			query.add_angular( 0, HK_BODY_A, b0, cross,  1.0f );
			query.add_angular( 0, HK_BODY_B, b1, cross,  -1.0f );
			query.commit_entries(1);
		}

		query.commit( HK_BODY_A, b0 );
		query.commit( HK_BODY_B, b1 );

		hk_real impulse = 0.0f;
		hk_real inv_virt_mass = query.get_vmq_storage().get_dense_matrix()(0,0);

		hk_real virtual_mass = 1.0f / inv_virt_mass;

		if (limit.m_friction_is_enabled){
			hk_real d_alpha = alpha - limit.m_ref_position;

			limit.m_ref_position += limit.m_desired_velocity * pi.get_delta_time();

			const hk_real friction_tau = 0.8f * tau_factor;
			const hk_real friction_damp = 1.0f * damp_factor;
			hk_real correction_per_psi = d_alpha * friction_tau * pi.get_inv_delta_time() -
													friction_damp * query.get_vmq_storage().get_velocities()[0];


			impulse = correction_per_psi * virtual_mass;

			if ( hk_Math::fabs( impulse ) * pi.get_inv_delta_time() > limit.m_joint_friction ){
				hk_real factor = limit.m_joint_friction/ hk_Math::fabs( impulse ) * pi.get_delta_time();

				impulse *= factor;
				limit.m_ref_position += (1.0f - factor) * d_alpha;
			}
		}

		if ( limit.m_limit_is_enabled ){ // do the limits
			
			hk_real vel = query.get_vmq_storage().get_velocities()[0] + impulse * query.get_vmq_storage().get_dense_matrix()(0,0);

			hk_real next_pos = alpha - vel * pi.get_delta_time();
			if ( next_pos > limit.m_limit_max ){
				hk_real delta = (next_pos - limit.m_limit_max) * tau_factor * limit.m_limit_tau * pi.get_inv_delta_time();
				impulse += virtual_mass * delta;
			}else if ( next_pos < limit.m_limit_min) {
				hk_real delta = (next_pos - limit.m_limit_min) * tau_factor * limit.m_limit_tau * pi.get_inv_delta_time();
				impulse += virtual_mass * delta;	
			}
		}
		if (impulse){
			hk_real impulses[1];
			impulses[0] = impulse;
			query.apply_impulses( HK_BODY_A, b0, impulses );
			query.apply_impulses( HK_BODY_B, b1, impulses );
		}
	}

	private:

		hk_Constraint_Limit_Util(); //not implemented
};


#endif /*HK_PHYSICS_CONSTRAINT_LIMIT_UTIL_H */


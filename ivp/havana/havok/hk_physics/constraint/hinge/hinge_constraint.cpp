#include <hk_physics/physics.h>
#include <hk_physics/simunit/psi_info.h>
#include <hk_physics/constraint/hinge/hinge_bp.h>
#include <hk_physics/constraint/hinge/hinge_constraint.h>


#include <hk_physics/core/vm_query_builder/vm_query_builder.h>
#include <hk_math/vector3/vector3_util.h>
#include <hk_math/densematrix_util.h>
#include <hk_math/dense_vector.h>

#include <hk_physics/constraint/util/constraint_limit_util.h>

// IVP_EXPORT_PUBLIC

#ifdef HK_ARCH_PPC
#include <stddef.h> // for size_t
#endif

/* LOCAL CLASS */
struct hk_Hinge_Constraint_Work
{
	hk_Fixed_Dense_Vector<5> m_correction;
	hk_VM_Query_Builder< hk_VMQ_Storage<5> > query_engine;
	
#ifdef HK_ARCH_PPC

	static inline void *operator new (size_t size, void *addr){
		return addr;
	}
#else
	static inline void *operator new (unsigned long int size, void *addr){
		return addr;
	}
	static inline void operator delete (void *, void *){ }
	
#endif

};


void hk_Hinge_Constraint::init_constraint(const void* vbp)
{
	const hk_Hinge_BP *bp = static_cast<const hk_Hinge_BP *>(vbp);
	this->init_hinge_constraint(bp);
}

void hk_Hinge_Constraint::init_hinge_constraint(const hk_Hinge_BP* bp)
{
	m_tau				= bp->m_tau;
	m_strength			= bp->m_strength;
	m_axis_os[0]		= bp->m_axis_os[0];
	m_axis_os[1]		= bp->m_axis_os[1];
	(hk_Constraint_Limit_BP&)m_limit = bp->m_limit;

	/* m_axis_perp_os[0,1] is a vector perpendicular to the hinge axis
	 * of body[0,1] in object space - its stored to avoid a normalize
	 * and an 'if' which is needed to compute it on the fly */

	for(int i=0; i<2; ++i)
	{
		if( bp->m_axis_perp_os[i].length() != 0.0f)
		{
			m_axis_perp_os[i] = bp->m_axis_perp_os[i];
			m_axis_perp_os[i].normalize();
		}
		else
		{
			m_axis_perp_os[i] = hk_Vector3_Util::perp_vec( m_axis_os[i].get_direction() );
		}
	}

	hk_Rigid_Body* b0 = get_rigid_body(0);
	hk_Rigid_Body* b1 = get_rigid_body(1);

	const hk_Transform &t0 = b0->get_cached_transform();
	const hk_Transform &t1 = b1->get_cached_transform();

	hk_Vector3 line_ws[2];
	line_ws[0].set_rotated_dir( t0, m_axis_os[0].get_direction());
	line_ws[1].set_rotated_dir( t1, m_axis_os[1].get_direction());

	hk_Vector3 perp_x_ws;
	hk_Vector3 perp_y_ws;

	perp_x_ws.set_rotated_dir( t0, m_axis_perp_os[0]);
	perp_y_ws.set_cross( perp_x_ws, line_ws[0] );

	hk_Vector3 perp_x2_ws;
	perp_x2_ws.set_rotated_dir( t1, m_axis_perp_os[1]);

	hk_Constraint_Limit_Util::init_angular_limit( perp_x_ws, perp_y_ws, this->m_limit, perp_x2_ws );
}

void hk_Hinge_Constraint::write_to_blueprint( hk_Hinge_BP *pOutBP )
{
	pOutBP->m_tau = m_tau;
	pOutBP->m_strength = m_strength;
	for ( int i = 0; i < 2; i++ )
	{
		pOutBP->m_axis_os[i] = m_axis_os[i];
		pOutBP->m_axis_perp_os[i] = m_axis_perp_os[i];
	}

	pOutBP->m_limit = m_limit;
}

hk_Hinge_Constraint::hk_Hinge_Constraint(hk_Environment* env,
		const hk_Hinge_BP* bp, hk_Rigid_Body* a ,hk_Rigid_Body* b )
	: hk_Constraint( env, a, b, HK_PRIORITY_LOCAL_CONSTRAINT)
{
	init_constraint(bp);
}


hk_Hinge_Constraint::hk_Hinge_Constraint( 
	hk_Local_Constraint_System* constraint_system,
	const hk_Hinge_BP* bp,
	hk_Rigid_Body* a ,
	hk_Rigid_Body* b )
	: hk_Constraint( constraint_system, a, b, HK_PRIORITY_LOCAL_CONSTRAINT, HK_NEXT_MULTIPLE_OF(16,sizeof(hk_Hinge_Constraint_Work)))
{
	init_constraint(bp);
}




int hk_Hinge_Constraint::get_vmq_storage_size()
{
	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Hinge_Constraint_Work) );
}

int	hk_Hinge_Constraint::setup_and_step_constraint(
		hk_PSI_Info& pi,
		void *mem,
		hk_real tau_factor,
		hk_real strength_factor )
{

	hk_Rigid_Body* b0 = get_rigid_body(0);
	hk_Rigid_Body* b1 = get_rigid_body(1);


	const hk_Transform &t0 = b0->get_cached_transform();
	const hk_Transform &t1 = b1->get_cached_transform();

	hk_Vector3 orig_ws[2];
	orig_ws[0].set_transformed_pos( t0, m_axis_os[0].get_origin());
	orig_ws[1].set_transformed_pos( t1, m_axis_os[1].get_origin());

	hk_Vector3 line_ws[2];
	line_ws[0].set_rotated_dir( t0, m_axis_os[0].get_direction());
	line_ws[1].set_rotated_dir( t1, m_axis_os[1].get_direction());

	hk_Vector3 perp_x_ws;
	hk_Vector3 perp_y_ws;

	perp_x_ws.set_rotated_dir( t0, m_axis_perp_os[0]);
	perp_y_ws.set_cross( perp_x_ws, line_ws[0] );

	if (  this->m_limit.m_limit_is_enabled | m_limit.m_friction_is_enabled ) // joint friction
	{
		hk_Vector3 perp_x2_ws; perp_x2_ws.set_rotated_dir( t1, m_axis_perp_os[1]);

		hk_real sin_alpha = perp_x2_ws.dot( perp_y_ws );
		hk_real cos_alpha = perp_x2_ws.dot( perp_x_ws );
		hk_real alpha = hk_Math::atan2( sin_alpha, cos_alpha );
		
		hk_Constraint_Limit_Util::do_angular_limit( pi, b0, line_ws[0], alpha, b1, m_limit, tau_factor, strength_factor );
	}

	//HK_DISPLAY_POINT(orig_ws[0], 0xff00ff);
	//HK_DISPLAY_POINT(orig_ws[1], 0x00ffff);
	//HK_DISPLAY_RAY(orig_ws[0], line_ws[0], 0xff00ff);
	//HK_DISPLAY_RAY(orig_ws[1], line_ws[1], 0x00ffff);

	hk_Hinge_Constraint_Work &work = *new (mem) hk_Hinge_Constraint_Work;
	hk_VM_Query_Builder< hk_VMQ_Storage<5> >& query_engine = work.query_engine;
	query_engine.begin(5);

	{
		query_engine.begin_entries(5);

		hk_Mass_Relative_Vector3 mcr_pt_0( b0, orig_ws[0]);
		hk_Mass_Relative_Vector3 mcr_pt_1( b1, orig_ws[1]);

		// point-to-point
		query_engine.add_linear_xyz( 0, HK_X_DIRECTION, HK_BODY_A, b0, mcr_pt_0,  1.0f );
		query_engine.add_linear_xyz( 0, HK_X_DIRECTION, HK_BODY_B, b1, mcr_pt_1, -1.0f );
		
		query_engine.add_linear_xyz( 1, HK_Y_DIRECTION, HK_BODY_A, b0, mcr_pt_0,  1.0f );
		query_engine.add_linear_xyz( 1, HK_Y_DIRECTION, HK_BODY_B, b1, mcr_pt_1, -1.0f );

		query_engine.add_linear_xyz( 2, HK_Z_DIRECTION, HK_BODY_A, b0, mcr_pt_0,  1.0f );
		query_engine.add_linear_xyz( 2, HK_Z_DIRECTION, HK_BODY_B, b1, mcr_pt_1, -1.0f );

		// angular
		query_engine.add_angular( 3, HK_BODY_A, b0, perp_x_ws,  1.0f );
		query_engine.add_angular( 3, HK_BODY_B, b1, perp_x_ws, -1.0f );

		query_engine.add_angular( 4, HK_BODY_A, b0, perp_y_ws,  1.0f );
		query_engine.add_angular( 4, HK_BODY_B, b1, perp_y_ws, -1.0f );
		
		query_engine.commit_entries(5);
	}
	query_engine.commit(HK_BODY_A, b0);
	query_engine.commit(HK_BODY_B, b1);

	const hk_real* vmq_velocity = query_engine.get_vmq_storage().get_velocities();

	hk_Fixed_Dense_Vector<5> delta;
	{
		hk_Vector3 &delta_dist = *(hk_Vector3 *)&work.m_correction(0);
		delta_dist.set_sub( orig_ws[1], orig_ws[0] );
		delta_dist.set_mul( pi.get_inv_delta_time(), delta_dist);

		work.m_correction(3) =  pi.get_inv_delta_time() * perp_y_ws.dot(line_ws[1]);
		work.m_correction(4) =  pi.get_inv_delta_time() * -perp_x_ws.dot(line_ws[1]);

		delta.set_mul_add_mul( m_tau * tau_factor, work.m_correction, -1.0f * m_strength * strength_factor, vmq_velocity );
	}

	hk_Fixed_Dense_Matrix<5>& mass_matrix = query_engine.get_vmq_storage().get_fixed_dense_matrix();

	hk_Dense_Matrix_Util::invert_5x5( mass_matrix, 0.0f);

	hk_Fixed_Dense_Vector<5> impulses;
	hk_Dense_Matrix_Util::mult( mass_matrix, delta, impulses);

		
	query_engine.apply_impulses( HK_BODY_A, b0, impulses.get_const_real_pointer() );
	query_engine.apply_impulses( HK_BODY_B, b1, impulses.get_const_real_pointer() );

	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Hinge_Constraint_Work));
}

void hk_Hinge_Constraint::step_constraint(
		hk_PSI_Info& pi,
		void *mem,
		hk_real tau_factor,
		hk_real strength_factor )
{
	hk_Hinge_Constraint_Work& work = *(hk_Hinge_Constraint_Work*)mem;
	hk_VM_Query_Builder< hk_VMQ_Storage<5> >& query_engine = work.query_engine;

	hk_Rigid_Body *b0 = get_rigid_body(0);
	hk_Rigid_Body *b1 = get_rigid_body(1);

	query_engine.get_vmq_storage().clear_velocities();
	query_engine.update_velocities(HK_BODY_A, b0);
	query_engine.update_velocities(HK_BODY_B, b1);

	hk_real* vmq_velocity = query_engine.get_vmq_storage().get_velocities();

	hk_Fixed_Dense_Vector<5> delta;
	delta.set_mul_add_mul( m_tau * tau_factor, work.m_correction, -1.0f * m_strength * strength_factor, vmq_velocity );
	
	hk_Dense_Matrix& mass_matrix = query_engine.get_vmq_storage().get_dense_matrix();

	hk_Fixed_Dense_Vector<5> impulses;
	hk_Dense_Matrix_Util::mult( mass_matrix, delta, impulses);

	query_engine.apply_impulses( HK_BODY_A, b0, impulses.get_const_real_pointer() );
	query_engine.apply_impulses( HK_BODY_B, b1, impulses.get_const_real_pointer() );
}


/* Hinge constraint - angular version */
void hk_Hinge_Constraint::apply_effector_PSI(	hk_PSI_Info& pinfo,
												hk_Array<hk_Entity*>* )
{
	hk_Hinge_Constraint_Work tmp_mem;
	hk_Hinge_Constraint::setup_and_step_constraint( pinfo, (void*)&tmp_mem, 1.0f, 1.0f );
}

void hk_Hinge_Constraint::set_limits( hk_real min, hk_real max )
{
	m_limit.set_limits(min,max);
}

void hk_Hinge_Constraint::set_friction( hk_real friction )
{
	m_limit.set_friction( friction );
}

void hk_Hinge_Constraint::set_motor( hk_real desired_vel, hk_real max_force )
{
	m_limit.set_motor( desired_vel, max_force );
}

/* Ballsocket + point-line version
//
void hk_Hinge_Constraint::apply_effector_PSI(	hk_PSI_Info& pi,
												hk_Array<hk_Entity*>* )
{
	hk_Rigid_Body* b0 = get_rigid_body(0);
	hk_Rigid_Body* b1 = get_rigid_body(1);

	hk_Vector3 orig_ws[2];
	orig_ws[0].set_transformed_pos( b0->get_cached_transform(), m_axis_os[0].get_origin());
	orig_ws[1].set_transformed_pos( b1->get_cached_transform(), m_axis_os[1].get_origin());

	hk_Vector3 line_ws[2];
	line_ws[0].set_transformed_pos( b0->get_cached_transform(),
			m_axis_os[0].get_origin() + m_axis_os[0].get_direction());
	line_ws[1].set_transformed_pos( b1->get_cached_transform(),
			m_axis_os[1].get_origin() + m_axis_os[1].get_direction());

	extern void HK_DISPLAY_POINT(const hk_Vector3& p, int c);
	extern void HK_DISPLAY_RAY(const hk_Vector3& p, const hk_Vector3& d, int c);

	HK_DISPLAY_POINT(orig_ws[0], 0xff00ff);
	HK_DISPLAY_POINT(orig_ws[1], 0x00ffff);
	HK_DISPLAY_POINT(line_ws[0], 0xff00ff);
	HK_DISPLAY_POINT(line_ws[1], 0x00ffff);

	hk_VM_Query_Builder< hk_VMQ_Storage<5> > query_engine;

	query_engine.begin(5);

	{
		hk_Mass_Relative_Vector3 mcr_pt_0( b0, orig_ws[0]);
		hk_Mass_Relative_Vector3 mcr_pt_1( b1, orig_ws[1]);

		query_engine.begin_entries(5);

		// point-to-point
		query_engine.add_linear_xyz( 0, HK_X_DIRECTION, HK_BODY_A, b0, mcr_pt_0,  1.0f );
		query_engine.add_linear_xyz( 0, HK_X_DIRECTION, HK_BODY_B, b1, mcr_pt_1, -1.0f );
		
		query_engine.add_linear_xyz( 1, HK_Y_DIRECTION, HK_BODY_A, b0, mcr_pt_0,  1.0f );
		query_engine.add_linear_xyz( 1, HK_Y_DIRECTION, HK_BODY_B, b1, mcr_pt_1, -1.0f );

		query_engine.add_linear_xyz( 2, HK_Z_DIRECTION, HK_BODY_A, b0, mcr_pt_0,  1.0f );
		query_engine.add_linear_xyz( 2, HK_Z_DIRECTION, HK_BODY_B, b1, mcr_pt_1, -1.0f );

		// point-to-line
		hk_Mass_Relative_Vector3 mcr_ln_0( b0, line_ws[0]);
		hk_Mass_Relative_Vector3 mcr_ln_1( b1, line_ws[1]);

		query_engine.add_linear_xyz( 3, HK_X_DIRECTION, HK_BODY_A, b0, mcr_ln_0,  1.0f );
		query_engine.add_linear_xyz( 3, HK_X_DIRECTION, HK_BODY_B, b1, mcr_ln_1, -1.0f );
		
		query_engine.add_linear_xyz( 4, HK_Y_DIRECTION, HK_BODY_A, b0, mcr_ln_0,  1.0f );
		query_engine.add_linear_xyz( 4, HK_Y_DIRECTION, HK_BODY_B, b1, mcr_ln_1, -1.0f );

		query_engine.commit_entries(5);
	}
	query_engine.commit(HK_BODY_A, b0);
	query_engine.commit(HK_BODY_B, b1);

	const hk_real* vmq_velocity = query_engine.get_vmq_storage().get_velocities();

	hk_Vector3 point_velocity( vmq_velocity[0], vmq_velocity[1], vmq_velocity[2] );
	hk_Vector3 line_velocity( vmq_velocity[3], vmq_velocity[4], 0 );

	hk_Vector3 delta_dist;
	delta_dist.set_sub( orig_ws[1], orig_ws[0] );
	delta_dist.set_mul( m_stiffness * pi.m_inv_delta_time, delta_dist);
	delta_dist.add_mul( -1.0f * m_strength, point_velocity );

	hk_Vector3 delta_line;
	delta_line.set_sub( line_ws[1], line_ws[0] );
	delta_line.set_mul( m_stiffness * pi.m_inv_delta_time, delta_line);
	delta_line.add_mul( -1.0f * m_strength, line_velocity );

	hk_Fixed_Dense_Vector<5> impulse;
	impulse(0) = delta_dist(0);
	impulse(1) = delta_dist(1);
	impulse(2) = delta_dist(2);
	impulse(3) = delta_line(0);
	impulse(4) = delta_line(1);

	hk_Dense_Matrix& mass_matrix = query_engine.get_vmq_storage().get_dense_matrix();
	hk_result solve_ok = hk_Dense_Matrix_Util::solve( mass_matrix, impulse, 0.0f);

	HK_DISPLAY_RAY(orig_ws[0], hk_Vector3(-impulse(0),-impulse(1),-impulse(2)), 0x00ffff);
	HK_DISPLAY_RAY(line_ws[0], hk_Vector3(-impulse(3),-impulse(4),0), 0x00ffff);

	HK_ASSERT(solve_ok == HK_OK);

	query_engine.apply_impulses( HK_BODY_A, b0, impulse.get_const_real_pointer() );
	query_engine.apply_impulses( HK_BODY_B, b1, impulse.get_const_real_pointer() );
}
*/


#include <hk_physics/physics.h>
#include <hk_physics/simunit/psi_info.h>
#include <hk_physics/constraint/car_wheel/car_wheel_bp.h>
#include <hk_physics/constraint/car_wheel/car_wheel_constraint.h>


#include <hk_physics/core/vm_query_builder/vm_query_builder.h>
#include <hk_math/vector3/vector3_util.h>
#include <hk_math/densematrix_util.h>
#include <hk_math/dense_vector.h>

#include <hk_physics/constraint/util/constraint_limit_util.h>

#ifdef HK_ARCH_PPC
#include <stddef.h> // for size_t
#endif

/* LOCAL CLASS */
struct hk_Car_Wheel_Constraint_Work
{
	hk_Fixed_Dense_Vector<4> m_correction;
	hk_VM_Query_Builder< hk_VMQ_Storage<4> > query_engine;
	
#ifdef HK_ARCH_PPC
	static inline void *operator new (size_t size, void *addr){
		return addr;
	}
#else
	static inline void *operator new (unsigned int size, void *addr){
		return addr;
	}

	static inline void operator delete (void *, void *){
	}
#endif

};


void hk_Car_Wheel_Constraint::init_constraint(const void* vbp)
{
	const hk_Car_Wheel_BP* bp = static_cast<const hk_Car_Wheel_BP*>(vbp);
	this->init_car_wheel_constraint(bp);
}

void hk_Car_Wheel_Constraint::init_car_wheel_constraint(const hk_Car_Wheel_BP* bp)
{	
	m_tau						= bp->m_tau;
	m_damp						= bp->m_damp;
	m_translation_os_ks[0]		= bp->m_translation_os_ks[0];
	m_translation_os_ks[1]		= bp->m_translation_os_ks[1];

	m_hinge_axis_os[0]			= bp->m_hinge_axis_os[0];
	m_hinge_axis_os[1]			= bp->m_hinge_axis_os[1];

	m_hinge_axis_os[0].normalize();
	m_hinge_axis_os[1].normalize();

	m_initial_hinge_axis_Ros	= m_hinge_axis_os[0];

	m_steering_axis_Ros			= bp->m_steering_axis_Ros;
	m_steering_axis_Ros.normalize();

	m_perp_hinge_axis_Aos = hk_Vector3_Util::perp_vec(m_hinge_axis_os[1]);

	m_suspension_limit.init_limit( bp->m_suspension_limit, 1.0f);
	m_wheel_limit.init_limit( bp->m_wheel_limit, 1.0f );

	{
		hk_Rigid_Body* b0 = get_rigid_body(0);
		hk_Rigid_Body* b1 = get_rigid_body(1);

		const hk_Transform &t0 = b0->get_cached_transform();
		const hk_Transform &t1 = b1->get_cached_transform();

		hk_Vector3 orig_ws[2];
		orig_ws[0].set_transformed_pos( t0, m_translation_os_ks[0] );
		orig_ws[1].set_transformed_pos( t1, m_translation_os_ks[1] );

		hk_Vector3 delta_pos_ws;
		delta_pos_ws.set_sub ( orig_ws[1], orig_ws[0] );

		hk_Vector3 steering_axis_ws;
		steering_axis_ws.set_rotated_dir( t0, m_steering_axis_Ros );

		hk_Constraint_Limit_Util::init_linear_limit( steering_axis_ws, delta_pos_ws, m_suspension_limit);

		{
			hk_Vector3 hinge_axis_ws;
			hinge_axis_ws.set_rotated_dir( t0, m_hinge_axis_os[0] );

			hk_Vector3 perp_steering_axis_ws;
			perp_steering_axis_ws.set_cross( steering_axis_ws, hinge_axis_ws);

			hk_Vector3 perp_x2_ws; perp_x2_ws.set_rotated_dir( t1, m_perp_hinge_axis_Aos);

			hk_real sin_alpha = perp_x2_ws.dot( steering_axis_ws );
			hk_real cos_alpha = perp_x2_ws.dot( perp_steering_axis_ws );
			hk_real alpha = -hk_Math::atan2( sin_alpha, cos_alpha );
			

			hk_Constraint_Limit_Util::init_angular_limit( m_wheel_limit, alpha);

		}
	}
}


void hk_Car_Wheel_Constraint::set_steering_angle( hk_real angle)
{
	hk_real sin_alpha = hk_Math::sin(angle);
	hk_real cos_alpha = hk_Math::cos(angle);

	hk_Vector3 sin_axis; sin_axis.set_cross( m_initial_hinge_axis_Ros, m_steering_axis_Ros);

	m_hinge_axis_os[0].set_mul( cos_alpha, m_initial_hinge_axis_Ros );
	m_hinge_axis_os[0].add_mul( sin_alpha, sin_axis );
}

hk_Car_Wheel_Constraint::hk_Car_Wheel_Constraint(hk_Environment* env,
		const hk_Car_Wheel_BP* bp, hk_Rigid_Body* a ,hk_Rigid_Body* b )
	: hk_Constraint( env, a, b, HK_PRIORITY_LOCAL_CONSTRAINT)
{
	init_car_wheel_constraint(bp);
}


hk_Car_Wheel_Constraint::hk_Car_Wheel_Constraint( 
	hk_Local_Constraint_System* constraint_system,
	const hk_Car_Wheel_BP* bp,
	hk_Rigid_Body* a ,
	hk_Rigid_Body* b )
	: hk_Constraint( constraint_system, a, b, HK_PRIORITY_LOCAL_CONSTRAINT, HK_NEXT_MULTIPLE_OF(16,sizeof(hk_Car_Wheel_Constraint_Work)))
{
	init_car_wheel_constraint(bp);
}


int hk_Car_Wheel_Constraint::get_vmq_storage_size()
{
	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Car_Wheel_Constraint_Work) );
}

int	hk_Car_Wheel_Constraint::setup_and_step_constraint(
		hk_PSI_Info& pi,
		void *mem,
		hk_real tau_factor,
		hk_real damp_factor )
{

	hk_Rigid_Body* b0 = get_rigid_body(0);
	hk_Rigid_Body* b1 = get_rigid_body(1);

	const hk_Transform &t0 = b0->get_cached_transform();
	const hk_Transform &t1 = b1->get_cached_transform();

	hk_Vector3 orig_ws[2];
	orig_ws[0].set_transformed_pos( t0, m_translation_os_ks[0] );
	orig_ws[1].set_transformed_pos( t1, m_translation_os_ks[1] );

	hk_Vector3 hinge_axis_ws[2];
	hinge_axis_ws[0].set_rotated_dir( t0, m_hinge_axis_os[0] );
	hinge_axis_ws[1].set_rotated_dir( t1, m_hinge_axis_os[1] );

	hk_Vector3 steering_axis_ws;
	steering_axis_ws.set_rotated_dir( t0, m_steering_axis_Ros );

	hk_Vector3 perp_steering_axis_ws;
	perp_steering_axis_ws.set_cross( steering_axis_ws, hinge_axis_ws[0] );

	hk_Vector3 delta_pos_ws;
	delta_pos_ws.set_sub ( orig_ws[1], orig_ws[0] );

	if (  this->m_suspension_limit.m_limit_is_enabled | m_suspension_limit.m_friction_is_enabled ){   // joint friction
		hk_Constraint_Limit_Util::do_linear_limit( pi, b0, b1, orig_ws[0], steering_axis_ws, delta_pos_ws, m_suspension_limit, tau_factor, damp_factor );
	}

	if (  this->m_wheel_limit.m_limit_is_enabled | m_wheel_limit.m_friction_is_enabled ){   // joint friction

		hk_Vector3 perp_x2_ws; perp_x2_ws.set_rotated_dir( t1, m_perp_hinge_axis_Aos);

		hk_real sin_alpha = perp_x2_ws.dot( steering_axis_ws );
		hk_real cos_alpha = perp_x2_ws.dot( perp_steering_axis_ws );
		hk_real alpha = -hk_Math::atan2( sin_alpha, cos_alpha );
		
		hk_Constraint_Limit_Util::do_angular_limit( pi, b0, hinge_axis_ws[0], alpha, b1, m_wheel_limit, tau_factor, damp_factor );
	}


	hk_Car_Wheel_Constraint_Work &work = *new (mem) hk_Car_Wheel_Constraint_Work;
	hk_VM_Query_Builder< hk_VMQ_Storage<4> >& query_engine = work.query_engine;
	query_engine.begin(4);

	{
		query_engine.begin_entries(4);

		hk_Mass_Relative_Vector3 mcr_pt_0( b0, orig_ws[0]);
		hk_Mass_Relative_Vector3 mcr_pt_1( b1, orig_ws[1]);

		// point-to-point
		query_engine.add_linear( 0, HK_BODY_A, b0, mcr_pt_0, hinge_axis_ws[0],  1.0f );
		query_engine.add_linear( 0, HK_BODY_B, b1, mcr_pt_1, hinge_axis_ws[0], -1.0f );
		
		query_engine.add_linear( 1, HK_BODY_A, b0, mcr_pt_0, perp_steering_axis_ws, 1.0f );
		query_engine.add_linear( 1, HK_BODY_B, b1, mcr_pt_1, perp_steering_axis_ws, -1.0f );

		// angular
		query_engine.add_angular( 2, HK_BODY_A, b0, steering_axis_ws,  1.0f );
		query_engine.add_angular( 2, HK_BODY_B, b1, steering_axis_ws, -1.0f );

		query_engine.add_angular( 3, HK_BODY_A, b0, perp_steering_axis_ws,  1.0f );
		query_engine.add_angular( 3, HK_BODY_B, b1, perp_steering_axis_ws, -1.0f );
		
		query_engine.commit_entries(4);
	}

	query_engine.commit(HK_BODY_A, b0);
	query_engine.commit(HK_BODY_B, b1);

	const hk_real* vmq_velocity = query_engine.get_vmq_storage().get_velocities();

	hk_Fixed_Dense_Vector<4> delta;
	{
		hk_real tau = pi.get_inv_delta_time();

		work.m_correction(0) = delta_pos_ws.dot( hinge_axis_ws[0] )         * tau;
		work.m_correction(1) = delta_pos_ws.dot( perp_steering_axis_ws )    * tau;

		work.m_correction(2) =  hinge_axis_ws[1].dot(perp_steering_axis_ws) * tau;
		work.m_correction(3) = -hinge_axis_ws[1].dot(steering_axis_ws)      * tau;

		delta.set_mul_add_mul( m_tau * tau_factor, work.m_correction, -1.0f * m_damp * damp_factor, vmq_velocity );
	}

	hk_Fixed_Dense_Matrix<4>& mass_matrix = query_engine.get_vmq_storage().get_fixed_dense_matrix();

	hk_Dense_Matrix_Util::invert_4x4( mass_matrix, 0.0f);

	hk_Fixed_Dense_Vector<4> impulses;
	hk_Dense_Matrix_Util::mult( mass_matrix, delta, impulses);
		
	query_engine.apply_impulses( HK_BODY_A, b0, impulses.get_const_real_pointer() );
	query_engine.apply_impulses( HK_BODY_B, b1, impulses.get_const_real_pointer() );

	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Car_Wheel_Constraint_Work));
}

void hk_Car_Wheel_Constraint::step_constraint(
		hk_PSI_Info& pi,
		void *mem,
		hk_real tau_factor,
		hk_real damp_factor )
{
	hk_Car_Wheel_Constraint_Work& work = *(hk_Car_Wheel_Constraint_Work*)mem;
	hk_VM_Query_Builder< hk_VMQ_Storage<4> >& query_engine = work.query_engine;

	hk_Rigid_Body *b0 = get_rigid_body(0);
	hk_Rigid_Body *b1 = get_rigid_body(1);

	query_engine.get_vmq_storage().clear_velocities();
	query_engine.update_velocities(HK_BODY_A, b0);
	query_engine.update_velocities(HK_BODY_B, b1);

	hk_real* vmq_velocity = query_engine.get_vmq_storage().get_velocities();

	hk_Fixed_Dense_Vector<4> delta;
	delta.set_mul_add_mul( m_tau * tau_factor, work.m_correction, -1.0f * m_damp * damp_factor, vmq_velocity );
	
	hk_Dense_Matrix& mass_matrix = query_engine.get_vmq_storage().get_dense_matrix();

	hk_Fixed_Dense_Vector<4> impulses;
	hk_Dense_Matrix_Util::mult( mass_matrix, delta, impulses);

	query_engine.apply_impulses( HK_BODY_A, b0, impulses.get_const_real_pointer() );
	query_engine.apply_impulses( HK_BODY_B, b1, impulses.get_const_real_pointer() );
}


/* Car_Wheel constraint */
void hk_Car_Wheel_Constraint::apply_effector_PSI(	hk_PSI_Info& pinfo,
												hk_Array<hk_Entity*>* )
{
	hk_Car_Wheel_Constraint_Work tmp_mem;
	hk_Car_Wheel_Constraint::setup_and_step_constraint( pinfo, (void*)&tmp_mem, 1.0f, 1.0f );
}




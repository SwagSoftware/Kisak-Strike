
#include <hk_physics/physics.h>
#include <hk_math/vector3/vector3_util.h>
#include <hk_physics/simunit/psi_info.h>
#include <hk_physics/constraint/prismatic/prismatic_constraint.h>
#include <hk_physics/constraint/prismatic/prismatic_bp.h>
#include <hk_physics/core/vm_query_builder/vm_query_builder.h>

#include <hk_math/densematrix_util.h>
#include <hk_math/dense_vector.h>
//#include <hk_math/eulerangles.h>
#include <hk_physics/constraint/local_constraint_system/local_constraint_system.h>

#include <hk_physics/constraint/util/constraint_limit_util.h>

/* Local class */

#ifdef HK_ARCH_PPC
#include <stddef.h> // for size_t
#endif

class hk_Prismatic_Work
{
	public:

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

		hk_Fixed_Dense_Vector<5> m_correction;
		hk_VM_Query_Builder< hk_VMQ_Storage<5> > query_engine;
};


void hk_Prismatic_Constraint::init_constraint(const void* vbp)
{
	const hk_Prismatic_BP *bp = static_cast<const hk_Prismatic_BP *>(vbp);
	this->init_prismatic_constraint(bp, HK_NULL);
}

void hk_Prismatic_Constraint::init_prismatic_constraint(const hk_Prismatic_BP* bp, hk_Local_Constraint_System *sys)
{
	m_tau				= bp->m_tau;
	m_strength			= bp->m_strength;
	m_transform_Ros_Aos	= bp->m_transform_Ros_Aos;
	m_axis_Ros			= bp->m_axis_Ros;
	m_axis_Ros.normalize();

	m_perp_axis_Ros = hk_Vector3_Util::perp_vec(	m_axis_Ros );

	const hk_real factor = (sys) ? sys->get_epsilon() : 1.0f;
	m_limits[0].init_limit( bp->m_limit , factor );

	hk_real mass_0 = get_rigid_body(0)->get_mass();
	hk_real mass_1 = get_rigid_body(1)->get_mass();
	if ( mass_0 == 0.0f){
		m_pos_interpolation_value = 1.0f;
	}else if (mass_1 == 0.0f){
		m_pos_interpolation_value = 0.0f;

	}else{
		m_pos_interpolation_value =  mass_1 / ( mass_0 + mass_1 );
	}


	const hk_Transform &t_ws_f_Ros = get_rigid_body(0)->get_cached_transform();
	const hk_Transform &t_ws_f_Aos = get_rigid_body(1)->get_cached_transform();

	hk_Vector3 axis_ws;     axis_ws.set_rotated_dir(t_ws_f_Ros, m_axis_Ros);
	hk_Vector3 pos_ws;		pos_ws.set_interpolate( t_ws_f_Ros.get_translation(), t_ws_f_Aos.get_translation(), m_pos_interpolation_value );

	hk_Vector3 delta_ws;
	{
		delta_ws.set_sub( t_ws_f_Aos.get_translation(), t_ws_f_Ros.get_translation());
		delta_ws.set_sub( delta_ws, m_transform_Ros_Aos.get_translation());
	}

	hk_Constraint_Limit_Util::init_linear_limit( axis_ws, delta_ws, m_limits[0]);
}

void hk_Prismatic_Constraint::write_to_blueprint( hk_Prismatic_BP *bp )
{
	bp->m_tau = m_tau;
	bp->m_strength = m_strength;
	bp->m_transform_Ros_Aos = m_transform_Ros_Aos;
	bp->m_axis_Ros = m_axis_Ros;
	bp->m_limit = m_limits[0];
}

hk_Prismatic_Constraint::hk_Prismatic_Constraint(
		hk_Environment *env,
		const hk_Prismatic_BP *bp,
		hk_Rigid_Body* a,
		hk_Rigid_Body* b )
	: hk_Constraint( env, a, b, HK_PRIORITY_LOCAL_CONSTRAINT)
{
	init_prismatic_constraint( bp, HK_NULL );
}


hk_Prismatic_Constraint::hk_Prismatic_Constraint(
		hk_Local_Constraint_System* constraint_system,
		const hk_Prismatic_BP* bp,
		hk_Rigid_Body* a ,
		hk_Rigid_Body* b )
	: hk_Constraint( constraint_system, a, b,
			HK_PRIORITY_LOCAL_CONSTRAINT,
			HK_NEXT_MULTIPLE_OF(16,sizeof(hk_Prismatic_Work)))
{
	init_prismatic_constraint( bp, constraint_system );
}


int hk_Prismatic_Constraint::get_vmq_storage_size()
{
	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Prismatic_Work));
}

/*
 * SETUP AND STEP
 */

int	hk_Prismatic_Constraint::setup_and_step_constraint(
		hk_PSI_Info& pi, void *mem,
		hk_real tau_factor, hk_real strength_factor )
{
	hk_Prismatic_Work &work = *new (mem) hk_Prismatic_Work;
	hk_Rigid_Body* b0 = get_rigid_body(0);
	hk_Rigid_Body* b1 = get_rigid_body(1);

	const hk_Transform &t_ws_f_Ros = b0->get_cached_transform();
	const hk_Transform &t_ws_f_Aos = b1->get_cached_transform();

	/* get the body joint axes, if specified */
	{ // calc angular parts
		hk_Rotation m_Aos_Ros;
		m_Aos_Ros.set_mul3_inv( t_ws_f_Aos, t_ws_f_Ros );

		hk_Quaternion q_Aos_Ros( m_Aos_Ros );
		/* get the relative joint axes and angles */

		hk_Quaternion &q_Desired_Ros_Aos = m_transform_Ros_Aos.m_rotation;

		hk_Quaternion q_Desired_Ros_Ros;

		q_Desired_Ros_Ros.set_mul( q_Desired_Ros_Aos, q_Aos_Ros );

		hk_real factor = 2.0f * tau_factor * m_tau * pi.get_inv_delta_time();

		if ( q_Aos_Ros.w > 0.0f){
			factor = -factor;
		}

		work.m_correction(0) = factor * q_Desired_Ros_Ros.x;
		work.m_correction(1) = factor * q_Desired_Ros_Ros.y;
		work.m_correction(2) = factor * q_Desired_Ros_Ros.z;
	}

	hk_Vector3 axis_ws;       axis_ws.set_rotated_dir(      t_ws_f_Ros, m_axis_Ros);
	hk_Vector3 perp_axis_ws;  perp_axis_ws.set_rotated_dir( t_ws_f_Ros, m_perp_axis_Ros);
	hk_Vector3 perp_axis2_ws; perp_axis2_ws.set_cross(      axis_ws,    perp_axis_ws);

	hk_Vector3 delta_ws;
	{
		hk_Vector3 shift_ws_Ros; shift_ws_Ros.set_rotated_dir( t_ws_f_Ros, m_transform_Ros_Aos.get_translation());
		delta_ws.set_sub( t_ws_f_Aos.get_translation(), t_ws_f_Ros.get_translation());
		//delta_ws.set_sub( delta_ws, m_transform_Ros_Aos.get_translation());

		delta_ws.set_sub( delta_ws, shift_ws_Ros);
	}
	{
		work.m_correction(3) = tau_factor * m_tau * pi.get_inv_delta_time() * delta_ws.dot( perp_axis_ws );
		work.m_correction(4) = tau_factor * m_tau * pi.get_inv_delta_time() * delta_ws.dot( perp_axis2_ws );
	}

	hk_Vector3 pos_ws;
	pos_ws.set_interpolate( t_ws_f_Ros.get_translation(), t_ws_f_Aos.get_translation(), m_pos_interpolation_value );


	if (  this->m_limits[0].m_limit_is_enabled | m_limits[0].m_friction_is_enabled ){   // joint friction
		hk_Constraint_Limit_Util::do_linear_limit(	pi, b0, b1, pos_ws, axis_ws, delta_ws, m_limits[0], tau_factor, strength_factor );
	}


	hk_VM_Query_Builder< hk_VMQ_Storage<5> >& query_engine = work.query_engine;
	query_engine.begin(5);

	{
		query_engine.begin_entries(5);
		{
			query_engine.add_angular(0, HK_BODY_A, b0, t_ws_f_Ros.get_column(0),  1.0f);
			query_engine.add_angular(0, HK_BODY_B, b1, t_ws_f_Ros.get_column(0), -1.0f);

			query_engine.add_angular(1, HK_BODY_A, b0, t_ws_f_Ros.get_column(1),  1.0f);
			query_engine.add_angular(1, HK_BODY_B, b1, t_ws_f_Ros.get_column(1), -1.0f);

			query_engine.add_angular(2, HK_BODY_A, b0, t_ws_f_Ros.get_column(2),  1.0f);
			query_engine.add_angular(2, HK_BODY_B, b1, t_ws_f_Ros.get_column(2), -1.0f);
		}
		{
			hk_Vector3 pos;
			pos.set_interpolate( t_ws_f_Ros.get_translation(), t_ws_f_Aos.get_translation(), m_pos_interpolation_value );
			hk_Mass_Relative_Vector3 mcr_0( b0, pos_ws);
			hk_Mass_Relative_Vector3 mcr_1( b1, pos_ws);

			query_engine.add_linear( 3, HK_BODY_A, b0, mcr_0, perp_axis_ws,  1.0f );
			query_engine.add_linear( 3, HK_BODY_B, b1, mcr_1, perp_axis_ws, -1.0f );

			query_engine.add_linear( 4, HK_BODY_A, b0, mcr_0, perp_axis2_ws, 1.0f );
			query_engine.add_linear( 4, HK_BODY_B, b1, mcr_1, perp_axis2_ws,-1.0f );
		}
		query_engine.commit_entries(5);
	}

	query_engine.commit(HK_BODY_A, b0);
	query_engine.commit(HK_BODY_B, b1);


	hk_Fixed_Dense_Vector<5> delta;
	{
		const hk_real* vmq_velocity = query_engine.get_vmq_storage().get_velocities();
		delta.set_add_mul( work.m_correction, -1.0f * m_strength * strength_factor , vmq_velocity );
	}


	hk_Fixed_Dense_Matrix<5>& mass_matrix = query_engine.get_vmq_storage().get_fixed_dense_matrix();
	hk_result invert_ok = hk_Dense_Matrix_Util::invert_5x5( mass_matrix, 0.0f);

	HK_ASSERT(invert_ok == HK_OK);

	hk_Fixed_Dense_Vector<5> impulses;
	hk_Dense_Matrix_Util::mult( mass_matrix, delta, impulses);

	query_engine.apply_impulses( HK_BODY_A, b0, impulses.get_const_real_pointer() );
	query_engine.apply_impulses( HK_BODY_B, b1, impulses.get_const_real_pointer() );

	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Prismatic_Work));
}

/*
 * STEP
 */

void hk_Prismatic_Constraint::step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real strength_factor )
{
	hk_Prismatic_Work& work = *(hk_Prismatic_Work*)mem;
	hk_VM_Query_Builder< hk_VMQ_Storage<5> >& query_engine = work.query_engine;

	hk_Rigid_Body *b0 = get_rigid_body(0);
	hk_Rigid_Body *b1 = get_rigid_body(1);

	if (  this->m_limits[0].m_limit_is_enabled | m_limits[0].m_friction_is_enabled ){   // joint friction
			const hk_Transform &t_ws_f_Ros = b0->get_cached_transform();
			const hk_Transform &t_ws_f_Aos = b1->get_cached_transform();
			hk_Vector3 axis_ws; axis_ws.set_rotated_dir(t_ws_f_Ros, m_axis_Ros);
			hk_Vector3 pos_ws;
			pos_ws.set_interpolate( t_ws_f_Ros.get_translation(), t_ws_f_Aos.get_translation(), m_pos_interpolation_value );

			// this section used to read like this:
			//   hk_Vector3 delta_ws;
			//   delta_ws.set_sub( t_ws_f_Aos.get_translation(), t_ws_f_Ros.get_translation());
			//   delta_ws.set_sub( delta_ws, m_transform_Ros_Aos.get_translation());
			//
			// Rotate m_transform_Ros_Aos.get_translation() into worldspace using the reference
			// object transform (this wasn't happening before - delta_ws wasn't being calculated
			// in worldspace). This code is essentially cribbed from the same section in
			// setup_and_step_constraint, above. Code CB, code review OS.
			hk_Vector3 shift_ws_Ros; shift_ws_Ros.set_rotated_dir( t_ws_f_Ros, m_transform_Ros_Aos.get_translation());
			hk_Vector3 delta_ws;
			delta_ws.set_sub( t_ws_f_Aos.get_translation(), t_ws_f_Ros.get_translation());
			delta_ws.set_sub( delta_ws, shift_ws_Ros);

			hk_Constraint_Limit_Util::do_linear_limit(	pi, b0, b1, pos_ws, axis_ws, delta_ws, m_limits[0], tau_factor, strength_factor );
	}

	query_engine.get_vmq_storage().clear_velocities();
	query_engine.update_velocities(HK_BODY_A, b0);
	query_engine.update_velocities(HK_BODY_B, b1);

	hk_real* vmq_velocity = query_engine.get_vmq_storage().get_velocities();

	hk_Fixed_Dense_Vector<5> delta;
	delta.set_mul_add_mul( tau_factor, work.m_correction, -1.0f * m_strength * strength_factor, vmq_velocity );

	hk_Dense_Matrix& mass_matrix = query_engine.get_vmq_storage().get_dense_matrix();

	hk_Fixed_Dense_Vector<5> impulses;
	hk_Dense_Matrix_Util::mult( mass_matrix, delta, impulses);

	query_engine.apply_impulses( HK_BODY_A, b0, impulses.get_const_real_pointer() );
	query_engine.apply_impulses( HK_BODY_B, b1, impulses.get_const_real_pointer() );

}

void hk_Prismatic_Constraint::apply_effector_PSI(
		hk_PSI_Info& pi, hk_Array<hk_Entity*>* )
{
	hk_Prismatic_Work work_mem;
	hk_Prismatic_Constraint::setup_and_step_constraint( pi,(void *)&work_mem, 1.0f, 1.0f );
}

void hk_Prismatic_Constraint::set_limits( hk_real min, hk_real max )
{
	m_limits[0].set_limits(min,max);
}

void hk_Prismatic_Constraint::set_friction( hk_real friction )
{
	m_limits[0].set_friction( friction );
}

void hk_Prismatic_Constraint::set_motor( hk_real desired_vel, hk_real max_force )
{
	m_limits[0].set_motor( desired_vel, max_force );
}

// HAVOK DO NOT EDIT



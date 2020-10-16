
#include <hk_physics/physics.h>
#include <hk_physics/simunit/psi_info.h>
#include <hk_math/interval.h>
#include <hk_physics/constraint/limited_ball_socket/limited_ball_socket_constraint.h>
#include <hk_physics/constraint/limited_ball_socket/limited_ball_socket_bp.h>
#include <hk_physics/core/vm_query_builder/vm_query_builder.h>

#include <hk_math/densematrix_util.h>
#include <hk_math/dense_vector.h>
#include <hk_math/eulerangles.h>

/* Local class */

//#define DISABLE_ANGULAR_FORCES

#ifdef HK_ARCH_PPC
#include <stddef.h> // for size_t
#endif


// IVP_EXPORT_PUBLIC

class hk_Limited_Ball_Socket_Work
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
		/* linear */
		hk_Vector3 dir;
		hk_VM_Query_Builder< hk_VMQ_Storage<3> > query_engine;

		/* angular*/
		hk_Vector3 joint_angles;
		hk_real virt_mass[3];
		hk_VM_Query_Builder< hk_VMQ_Storage<1> > query_engine_angle[3];
};

void hk_Limited_Ball_Socket_Constraint::init_constraint( const void* vbp )
{
	const hk_Limited_Ball_Socket_BP* bp = static_cast<const hk_Limited_Ball_Socket_BP*>(vbp);
	this->init_ball_socket_constraint(bp);
}

void hk_Limited_Ball_Socket_Constraint::init_ball_socket_constraint( const hk_Limited_Ball_Socket_BP *bp )
{
	m_tau					= bp->m_tau;
	m_strength				= bp->m_strength;
	m_transform_os_ks[0]	= bp->m_transform_os_ks[0];
	m_transform_os_ks[1]	= bp->m_transform_os_ks[1];

	for(int i=0; i<3; i++)
	{
		m_angular_limits[i] = bp->m_angular_limits[i];
	}
	m_constrainTranslation = bp->m_constrainTranslation;
}

hk_Limited_Ball_Socket_Constraint::hk_Limited_Ball_Socket_Constraint(
		hk_Environment *env,
		const hk_Limited_Ball_Socket_BP *bp,
		hk_Rigid_Body* a,
		hk_Rigid_Body* b )
	: hk_Constraint( env, a, b, HK_PRIORITY_LOCAL_CONSTRAINT)
{
	init_ball_socket_constraint(bp);
}


hk_Limited_Ball_Socket_Constraint::hk_Limited_Ball_Socket_Constraint( 
		hk_Local_Constraint_System* constraint_system,
		const hk_Limited_Ball_Socket_BP* bp,
		hk_Rigid_Body* a ,
		hk_Rigid_Body* b )
	: hk_Constraint( constraint_system, a, b,
			HK_PRIORITY_LOCAL_CONSTRAINT,
			HK_NEXT_MULTIPLE_OF(16,sizeof(hk_Limited_Ball_Socket_Work)))
{
	init_ball_socket_constraint(bp);
}


int hk_Limited_Ball_Socket_Constraint::get_vmq_storage_size()
{
	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Limited_Ball_Socket_Work));
}

/*
 * SETUP AND STEP
 */

int	hk_Limited_Ball_Socket_Constraint::setup_and_step_constraint(
		hk_PSI_Info& pi, void *mem,
		hk_real tau_factor, hk_real strength_factor )
{
	hk_Limited_Ball_Socket_Work &work = *new (mem) hk_Limited_Ball_Socket_Work;
	hk_Rigid_Body* b0 = get_rigid_body(0);
	hk_Rigid_Body* b1 = get_rigid_body(1);


#ifndef DISABLE_ANGULAR_FORCES
	{ /* setup and step ANGULAR PARTS */
		/* get the body joint axes, if specified */

		hk_Rigid_Body* body[2];
		body[0] = b0;
		body[1] = b1;

		hk_Rotation t_ws_ks[2];
		for(int k=0; k<2; ++k)
		{
			//XXX get rid of unnecessary transform
			t_ws_ks[k].set_mul3( body[k]->get_cached_transform(),
										m_transform_os_ks[k]);
			//t_ws_ks[k] = body[k]->get_cached_transform();
		}

		/* get the relative joint axes and angles */

		hk_Matrix3 joint_axes;
		if (0){ // use euler
			hk_Euler_Angles::create_rot_axis_and_angles(
					t_ws_ks[0], t_ws_ks[1],
					0,1,2,
					joint_axes, work.joint_angles);
			work.joint_angles *= -1.0f;
		}else{ // use quaternion as in ipion
 			hk_Rotation m_ks0_ks1;
			m_ks0_ks1.set_mul3_inv(t_ws_ks[0], t_ws_ks[1]);

			hk_Quaternion q_ks0_ks1( m_ks0_ks1 );

			{ // calculate joint axes
				hk_Quaternion half_qrot;
				half_qrot.set_slerp( hk_Quaternion( 0.0f,0.0f,0.0f,1.0f), q_ks0_ks1, 0.5f);
				const hk_Rotation half_rot(half_qrot);

				joint_axes.set_mul3( t_ws_ks[0], half_rot );
			}

			work.joint_angles.set(	2.0f * hk_Math::asin(q_ks0_ks1.x),
									2.0f * hk_Math::asin(q_ks0_ks1.y),
									2.0f * hk_Math::asin(q_ks0_ks1.z));
			if ( q_ks0_ks1.w < 0.0f){
				work.joint_angles *= -1.0f;
			}
		}
		/* apply impulses as necessary */

		for(int i=0; i<3; ++i)
		{
			hk_VM_Query_Builder< hk_VMQ_Storage<1> >& qe = work.query_engine_angle[i];
			qe.begin(1);
			qe.begin_entries(1);

			qe.add_angular(0, HK_BODY_A, b0, joint_axes.get_column(i),  1.0f);
			qe.add_angular(0, HK_BODY_B, b1, joint_axes.get_column(i), -1.0f);

			qe.commit_entries(1);
			qe.commit(HK_BODY_A, b0);
			qe.commit(HK_BODY_B, b1);

			hk_Dense_Matrix& mass_matrix = qe.get_vmq_storage().get_dense_matrix();
			work.virt_mass[i] = 1.0f/mass_matrix(0,0);

			hk_real joint_vel = *qe.get_vmq_storage().get_velocities();
			hk_real joint_angle_now	 = work.joint_angles(i);
			hk_real joint_angle_next = joint_angle_now - pi.get_delta_time() * joint_vel;
			hk_real joint_angle_delta;

			if( joint_angle_next >= m_angular_limits[i].m_min )
			{
				if( joint_angle_next <= m_angular_limits[i].m_max )
				{
					continue;
				}
				else // above upper
				{
					joint_angle_delta = joint_angle_next - m_angular_limits[i].m_max;
				}
			}
			else // below lower
			{
				joint_angle_delta = joint_angle_next - m_angular_limits[i].m_min;
			}

			hk_real tau_part = m_tau * pi.get_inv_delta_time() * joint_angle_delta;
			hk_real delta =	 tau_factor * tau_part;
							//-1.0f * strength_factor * m_strength * joint_vel;

			hk_real impulse = work.virt_mass[i] * delta;

			qe.apply_impulses( HK_BODY_A, b0, &impulse );
			qe.apply_impulses( HK_BODY_B, b1, &impulse );
		}
	}
#endif

	if ( m_constrainTranslation )
	{	/* setup and step LINEAR PART */
		hk_VM_Query_Builder< hk_VMQ_Storage<3> > &query_engine = work.query_engine;

		hk_Vector3 position_ws[2];
		position_ws[0]._set_transformed_pos( b0->get_cached_transform(),
							m_transform_os_ks[0].get_translation());
		position_ws[1]._set_transformed_pos( b1->get_cached_transform(),
							m_transform_os_ks[1].get_translation());

		hk_Vector3 &dir = work.dir;
		dir.set_sub( position_ws[1], position_ws[0] );

		query_engine.begin(3);
		{
			hk_Mass_Relative_Vector3 mcr_0( b0, position_ws[0]);
			hk_Mass_Relative_Vector3 mcr_1( b1, position_ws[1]);

			query_engine.begin_entries(3);

			query_engine.add_linear_xyz( 0, HK_X_DIRECTION, HK_BODY_A, b0, mcr_0,  1.0f );
			query_engine.add_linear_xyz( 0, HK_X_DIRECTION, HK_BODY_B, b1, mcr_1, -1.0f );
		
			query_engine.add_linear_xyz( 1, HK_Y_DIRECTION, HK_BODY_A, b0, mcr_0,  1.0f );
			query_engine.add_linear_xyz( 1, HK_Y_DIRECTION, HK_BODY_B, b1, mcr_1, -1.0f );
		
			query_engine.add_linear_xyz( 2, HK_Z_DIRECTION, HK_BODY_A, b0, mcr_0,  1.0f );
			query_engine.add_linear_xyz( 2, HK_Z_DIRECTION, HK_BODY_B, b1, mcr_1, -1.0f );
			
			query_engine.commit_entries(3);
		}
		query_engine.commit(HK_BODY_A, b0);
		query_engine.commit(HK_BODY_B, b1);

		hk_real *approaching_velocity = query_engine.get_vmq_storage().get_velocities();

		hk_Vector3 delta_dist_3;
		delta_dist_3.set_mul( m_tau * pi.get_inv_delta_time(), dir );
		delta_dist_3.add_mul( -1.0f * m_strength, *(const hk_Vector3 *)approaching_velocity );

		hk_Fixed_Dense_Matrix<3>& mass_matrix = query_engine.get_vmq_storage().get_fixed_dense_matrix();

		hk_Dense_Matrix_Util::invert_3x3_symmetric( mass_matrix, 0.0f );
		hk_Vector3 impulses;
		hk_Dense_Matrix_Util::mult_3_symmetric( mass_matrix, delta_dist_3, impulses);
		query_engine.apply_impulses( HK_BODY_A, b0, (hk_real *)&impulses(0) );
		query_engine.apply_impulses( HK_BODY_B, b1, (hk_real *)&impulses(0) );
	}

	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Limited_Ball_Socket_Work));
}

/*
 * STEP
 */

void hk_Limited_Ball_Socket_Constraint::step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real strength_factor )
{
	hk_Rigid_Body *b0 = get_rigid_body(0);
	hk_Rigid_Body *b1 = get_rigid_body(1);
	hk_Limited_Ball_Socket_Work &work = *(hk_Limited_Ball_Socket_Work *)mem;

#ifndef DISABLE_ANGULAR_FORCES
	{	/* step ANGULAR */
		for(int i=0; i<3; ++i)
		{
			hk_VM_Query_Builder< hk_VMQ_Storage<1> >& qe = work.query_engine_angle[i];
			qe.get_vmq_storage().clear_velocities();
			qe.update_velocities(HK_BODY_A, b0);
			qe.update_velocities(HK_BODY_B, b1);

			hk_real joint_vel = *qe.get_vmq_storage().get_velocities();
			hk_real joint_angle_now	 = work.joint_angles(i);
			hk_real joint_angle_next = joint_angle_now - pi.get_delta_time() * joint_vel;
			hk_real joint_angle_delta;

			if( joint_angle_next >= m_angular_limits[i].m_min )
			{
				if( joint_angle_next <= m_angular_limits[i].m_max )
				{
					continue;
				}
				else // above upper
				{
					joint_angle_delta = joint_angle_next - m_angular_limits[i].m_max;
				}
			}
			else // below lower
			{
				joint_angle_delta = joint_angle_next - m_angular_limits[i].m_min;
			}

			hk_real tau_part = m_tau * pi.get_inv_delta_time() * joint_angle_delta;
			hk_real delta = tau_factor * tau_part;
							//-1.0f * strength_factor * m_strength * joint_vel;

			hk_real impulse = work.virt_mass[i] * delta;

			qe.apply_impulses( HK_BODY_A, b0, &impulse );
			qe.apply_impulses( HK_BODY_B, b1, &impulse );
		}
	}
#endif

	{ /* step LINEAR */
		hk_Vector3 &dir = work.dir;
		hk_VM_Query_Builder< hk_VMQ_Storage<3> > &query_engine = work.query_engine;

		query_engine.get_vmq_storage().clear_velocities();
		query_engine.update_velocities(HK_BODY_A, b0);
		query_engine.update_velocities(HK_BODY_B, b1);
		
		hk_Vector3 delta_dist_3;
		hk_Preallocated_Dense_Vector delta (delta_dist_3.get_real_pointer(), 3, 4);

		hk_real *approaching_velocity = query_engine.get_vmq_storage().get_velocities();
		delta_dist_3.set_mul( tau_factor * m_tau * pi.get_inv_delta_time(), dir );
		delta_dist_3.add_mul( -1.0f * strength_factor *m_strength, *(hk_Vector3 *) approaching_velocity );

		hk_Vector3 impulses;
		hk_Fixed_Dense_Matrix<3>& mass_matrix = query_engine.get_vmq_storage().get_fixed_dense_matrix();
		hk_Dense_Matrix_Util::mult_3_symmetric( mass_matrix, delta_dist_3, impulses);

		query_engine.apply_impulses( HK_BODY_A, b0, (hk_real *)&impulses(0) );
		query_engine.apply_impulses( HK_BODY_B, b1, (hk_real *)&impulses(0) );
	}

}



void hk_Limited_Ball_Socket_Constraint::apply_effector_PSI(
		hk_PSI_Info& pi, hk_Array<hk_Entity*>* )
{
	hk_Limited_Ball_Socket_Work work_mem;
	hk_Limited_Ball_Socket_Constraint::setup_and_step_constraint( pi,(void *)&work_mem, 1.0f, 1.0f );
}

void hk_Limited_Ball_Socket_Constraint::set_angular_limits( int axis, hk_real min, hk_real max )
{
	m_angular_limits[axis].set(min,max);


}

// HAVOK DO NOT EDIT



#include <hk_physics/physics.h>
#include <hk_physics/simunit/psi_info.h>
#include <hk_physics/constraint/ball_socket/ball_socket_constraint.h>
#include <hk_physics/constraint/ball_socket/ball_socket_bp.h>
#include <hk_physics/core/vm_query_builder/vm_query_builder.h>
#include <hk_math/densematrix_util.h>
#include <hk_math/dense_vector.h>

#ifdef HK_ARCH_PPC
#include <stddef.h> // for size_t
#endif

// IVP_EXPORT_PUBLIC

class hk_Ball_Socket_Work
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
		
		static inline void operator delete (void *, void *){
		}

#endif

		
		hk_Vector3 dir;
		hk_VM_Query_Builder< hk_VMQ_Storage<3> > query_engine;
};


hk_Ball_Socket_Constraint::hk_Ball_Socket_Constraint(
		hk_Environment *env,
		const hk_Ball_Socket_BP *bp,
		hk_Rigid_Body* a,
		hk_Rigid_Body* b )
	: hk_Constraint( env, a, b, HK_PRIORITY_LOCAL_CONSTRAINT)
{
	init_ball_socket_constraint(bp);}


hk_Ball_Socket_Constraint::hk_Ball_Socket_Constraint( 
	hk_Local_Constraint_System* constraint_system,
	const hk_Ball_Socket_BP* bp,
	hk_Rigid_Body* a ,
	hk_Rigid_Body* b )
	: hk_Constraint( constraint_system, a, b, HK_PRIORITY_LOCAL_CONSTRAINT, HK_NEXT_MULTIPLE_OF(16,sizeof(hk_Ball_Socket_Work)))
{
	init_ball_socket_constraint(bp);
}


void hk_Ball_Socket_Constraint::init_constraint(const void* vbp)
{
	const hk_Ball_Socket_BP* bp = static_cast<const hk_Ball_Socket_BP*>(vbp);
	init_ball_socket_constraint(bp);
}

void hk_Ball_Socket_Constraint::init_ball_socket_constraint(const hk_Ball_Socket_BP* bp)
{
	m_tau					= bp->m_tau;
	m_strength				= bp->m_strength;
	m_translation_os_ks[0]	= bp->m_translation_os_ks[0];
	m_translation_os_ks[1]	= bp->m_translation_os_ks[1];
}

void hk_Ball_Socket_Constraint::write_to_blueprint( hk_Ball_Socket_BP *bp )
{
	bp->m_tau = m_tau;
	bp->m_strength = m_strength;
	bp->m_translation_os_ks[0] = m_translation_os_ks[0];
	bp->m_translation_os_ks[1] = m_translation_os_ks[1];
}

int hk_Ball_Socket_Constraint::get_vmq_storage_size()
{
	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Ball_Socket_Work));
}

int	hk_Ball_Socket_Constraint::setup_and_step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor )
{
	hk_Ball_Socket_Work &work = *new (mem) hk_Ball_Socket_Work;
	hk_VM_Query_Builder< hk_VMQ_Storage<3> > &query_engine = work.query_engine;

	
	hk_Rigid_Body *b0 = get_rigid_body(0);
	hk_Rigid_Body *b1 = get_rigid_body(1);

	hk_Vector3 translation_ws_ks[2];
	translation_ws_ks[0]._set_transformed_pos( b0->get_cached_transform(), m_translation_os_ks[0]);
	translation_ws_ks[1]._set_transformed_pos( b1->get_cached_transform(), m_translation_os_ks[1]);

	hk_Vector3 &dir = work.dir;
	dir.set_sub( translation_ws_ks[1], translation_ws_ks[0] );

	query_engine.begin(3);

	{
		hk_Mass_Relative_Vector3 mcr_0( b0, translation_ws_ks[0]);
		hk_Mass_Relative_Vector3 mcr_1( b1, translation_ws_ks[1]);

		query_engine.begin_entries(3);
		{
			query_engine.add_linear_xyz( 0, HK_X_DIRECTION, HK_BODY_A, b0, mcr_0,  1.0f );
			query_engine.add_linear_xyz( 0, HK_X_DIRECTION, HK_BODY_B, b1, mcr_1, -1.0f );
		}
		{
			query_engine.add_linear_xyz( 1, HK_Y_DIRECTION, HK_BODY_A, b0, mcr_0,  1.0f );
			query_engine.add_linear_xyz( 1, HK_Y_DIRECTION, HK_BODY_B, b1, mcr_1, -1.0f );
		}
		{
			query_engine.add_linear_xyz( 2, HK_Z_DIRECTION, HK_BODY_A, b0, mcr_0,  1.0f );
			query_engine.add_linear_xyz( 2, HK_Z_DIRECTION, HK_BODY_B, b1, mcr_1, -1.0f );
		}
		query_engine.commit_entries(3);
	}
	query_engine.commit(HK_BODY_A, b0);
	query_engine.commit(HK_BODY_B, b1);

	hk_real *approaching_velocity = query_engine.get_vmq_storage().get_velocities();

	const hk_real *pos = &b0->get_cached_transform().get_translation().x;
	hk_Vector3 delta_dist_3;
	delta_dist_3.set_mul( tau_factor * m_tau * pi.get_inv_delta_time(), dir );
	delta_dist_3.add_mul( -1.0f * m_strength * damp_factor, *(const hk_Vector3 *)approaching_velocity );

	hk_Fixed_Dense_Matrix<3>& mass_matrix = query_engine.get_vmq_storage().get_fixed_dense_matrix();

	hk_Dense_Matrix_Util::invert_3x3_symmetric( mass_matrix, 0.0f );
	hk_Vector3 impulses;
	hk_Dense_Matrix_Util::mult_3_symmetric( mass_matrix, delta_dist_3, impulses);


	query_engine.apply_impulses( HK_BODY_A, b0, (hk_real *)&impulses(0) );
	query_engine.apply_impulses( HK_BODY_B, b1, (hk_real *)&impulses(0) );

	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Ball_Socket_Work));
}

void hk_Ball_Socket_Constraint::step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor )
{
	hk_Ball_Socket_Work &work = *(hk_Ball_Socket_Work *)mem;
	hk_Vector3 &dir = work.dir;
	hk_VM_Query_Builder< hk_VMQ_Storage<3> > &query_engine = work.query_engine;

	hk_real *approaching_velocity = query_engine.get_vmq_storage().get_velocities();
	((hk_Vector3 *)approaching_velocity)->set_zero();

	hk_Rigid_Body *b0 = get_rigid_body(0);
	hk_Rigid_Body *b1 = get_rigid_body(1);

	query_engine.update_velocities(HK_BODY_A, b0);
	query_engine.update_velocities(HK_BODY_B, b1);
	
	hk_Vector3 delta_dist_3;
	hk_Preallocated_Dense_Vector delta (delta_dist_3.get_real_pointer(), 3, 4);

	delta_dist_3.set_mul( tau_factor * m_tau * pi.get_inv_delta_time(), dir );
	delta_dist_3.add_mul( -1.0f * damp_factor * m_strength, *(hk_Vector3 *) approaching_velocity );

	hk_Vector3 impulses;
	hk_Fixed_Dense_Matrix<3>& mass_matrix = query_engine.get_vmq_storage().get_fixed_dense_matrix();
	hk_Dense_Matrix_Util::mult_3_symmetric( mass_matrix, delta_dist_3, impulses);

	query_engine.apply_impulses( HK_BODY_A, b0, (hk_real *)&impulses(0) );
	query_engine.apply_impulses( HK_BODY_B, b1, (hk_real *)&impulses(0) );
}


void hk_Ball_Socket_Constraint::apply_effector_PSI( hk_PSI_Info& pi, hk_Array<hk_Entity*>* )
{
	hk_Ball_Socket_Work work_mem;
	hk_Ball_Socket_Constraint::setup_and_step_constraint( pi,(void *)&work_mem, 1.0f, 1.0f );
}


// HAVOK DO NOT EDIT



#include <hk_physics/physics.h>

#include <hk_physics/simunit/psi_info.h>
#include <hk_physics/core/vm_query_builder/vm_query_builder.h>

#include <hk_physics/constraint/pulley/pulley_bp.h>
#include <hk_physics/constraint/pulley/pulley_constraint.h>

#ifdef HK_ARCH_PPC
#include <stddef.h> // for size_t
#endif

class hk_Pulley_Work
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

		static inline void operator delete (void *, void *){;}
#endif

		hk_VM_Query_Builder< hk_VMQ_Storage<1> > query_engine;
		hk_real current_length;
};

void hk_Pulley_Constraint::init_constraint( const void* vbp )
{
	const hk_Pulley_BP* bp = static_cast<const hk_Pulley_BP*>(vbp);
	this->init_pulley_constraint(bp);
}

void hk_Pulley_Constraint::init_pulley_constraint( const hk_Pulley_BP *bp )
{
	m_tau		= bp->m_tau;
	m_strength	= bp->m_strength;
	m_length 	= bp->m_length;
	m_gearing	= bp->m_gearing;
	m_translation_os_ks[0]	= bp->m_translation_os_ks[0];
	m_translation_os_ks[1]	= bp->m_translation_os_ks[1];
	m_worldspace_point[0] = bp->m_worldspace_point[0];
	m_worldspace_point[1] = bp->m_worldspace_point[1];
	m_is_rigid = bp->m_is_rigid;
}

void hk_Pulley_Constraint::write_to_blueprint( hk_Pulley_BP *bp )
{
	bp->m_tau = m_tau;
	bp->m_strength = m_strength;
	bp->m_length = m_length;
	bp->m_gearing = m_gearing;
	bp->m_translation_os_ks[0] = m_translation_os_ks[0];
	bp->m_translation_os_ks[1] = m_translation_os_ks[1];
	bp->m_worldspace_point[0] = m_worldspace_point[0];
	bp->m_worldspace_point[1] = m_worldspace_point[1];
	bp->m_is_rigid = m_is_rigid;
}

hk_Pulley_Constraint::hk_Pulley_Constraint(
		hk_Environment *env,
		const hk_Pulley_BP *bp,
		hk_Rigid_Body* a,
		hk_Rigid_Body* b )
	: hk_Constraint( env, a, b, HK_PRIORITY_LOCAL_CONSTRAINT)
{
	init_pulley_constraint( bp );
}

hk_Pulley_Constraint::hk_Pulley_Constraint( 
	hk_Local_Constraint_System* constraint_system,
	const hk_Pulley_BP* bp,
	hk_Rigid_Body* a ,
	hk_Rigid_Body* b )
	: hk_Constraint( constraint_system, a, b, HK_PRIORITY_LOCAL_CONSTRAINT, HK_NEXT_MULTIPLE_OF(16,sizeof(hk_Pulley_Work)))
{
	init_pulley_constraint( bp );
}

void hk_Pulley_Constraint::set_length( hk_real length )
{
	m_length = length;
}

void hk_Pulley_Constraint::set_gearing( hk_real gearing )
{
	m_gearing = gearing;
}

void hk_Pulley_Constraint::set_worldspace_point( int index, const hk_Vector3& ws_point )
{
	m_worldspace_point[index] = ws_point;
}

int hk_Pulley_Constraint::get_vmq_storage_size()
{
	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Pulley_Work));
}

int hk_Pulley_Constraint::setup_and_step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor )
{
	hk_Pulley_Work &work = *new (mem) hk_Pulley_Work;
	hk_VM_Query_Builder< hk_VMQ_Storage<1> > &query_engine = work.query_engine;
	
	hk_Rigid_Body *b0 = get_rigid_body(0);
	hk_Rigid_Body *b1 = get_rigid_body(1);

	hk_Vector3 translation_ws_ks[2];

	translation_ws_ks[0]._set_transformed_pos( b0->get_cached_transform(), m_translation_os_ks[0]);
	translation_ws_ks[1]._set_transformed_pos( b1->get_cached_transform(), m_translation_os_ks[1]);

	hk_Vector3 dir[2];
	dir[0].set_sub( m_worldspace_point[0], translation_ws_ks[0] );
	dir[1].set_sub( m_worldspace_point[1], translation_ws_ks[1] );
	dir[1]*=m_gearing;

	work.current_length = dir[0].normalize_with_length() + dir[1].normalize_with_length() - m_length;

	query_engine.begin(1);

	{
		query_engine.begin_entries(1);
		{
			query_engine.add_linear( 0, HK_BODY_A, b0, translation_ws_ks[0], dir[0],  1.0f );
			query_engine.add_linear( 0, HK_BODY_B, b1, translation_ws_ks[1], dir[1],  1.0f );
		}
		query_engine.commit_entries(1);
	}
	query_engine.commit(HK_BODY_A, b0);
	query_engine.commit(HK_BODY_B, b1);

	hk_Dense_Matrix& mass_matrix = query_engine.get_vmq_storage().get_dense_matrix();
	mass_matrix(0,0) = 1.0f / mass_matrix(0,0); // invert in place

	{ // step
		if ( m_is_rigid || work.current_length >= 0 )
		{
			hk_real *approaching_velocity = query_engine.get_vmq_storage().get_velocities();
			hk_real delta_dist = tau_factor * m_tau * pi.get_inv_delta_time() * work.current_length - damp_factor * m_strength * approaching_velocity[0];

			hk_Vector3 impulses;
			impulses(0) = delta_dist * mass_matrix(0,0);

			query_engine.apply_impulses( HK_BODY_A, b0, (hk_real *)&impulses(0) );
			query_engine.apply_impulses( HK_BODY_B, b1, (hk_real *)&impulses(0) );
		}
	}
	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Pulley_Work));
}

void hk_Pulley_Constraint::step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor )
{
	hk_Pulley_Work &work = *(hk_Pulley_Work *)mem;
	hk_VM_Query_Builder< hk_VMQ_Storage<1> > &query_engine = work.query_engine;

	hk_real *approaching_velocity = query_engine.get_vmq_storage().get_velocities();
	approaching_velocity[0] = 0.0f;

	hk_Rigid_Body *b0 = get_rigid_body(0);
	hk_Rigid_Body *b1 = get_rigid_body(1);

	query_engine.update_velocities(HK_BODY_A, b0);
	query_engine.update_velocities(HK_BODY_B, b1);
	if ( m_is_rigid || work.current_length >= 0 )
	{
		hk_real delta_dist = tau_factor * m_tau * pi.get_inv_delta_time() * work.current_length - damp_factor * m_strength * approaching_velocity[0];

		hk_Vector3 impulses;
		hk_Dense_Matrix& mass_matrix = query_engine.get_vmq_storage().get_dense_matrix();
		impulses(0) = delta_dist * mass_matrix(0,0);

		query_engine.apply_impulses( HK_BODY_A, b0, (hk_real *)&impulses(0) );
		query_engine.apply_impulses( HK_BODY_B, b1, (hk_real *)&impulses(0) );
	}
}


void hk_Pulley_Constraint::apply_effector_PSI( hk_PSI_Info& pi, hk_Array<hk_Entity*>* )
{
	hk_Pulley_Work work_mem;
	hk_Pulley_Constraint::setup_and_step_constraint( pi,(void *)&work_mem, 1.0f, 1.0f );
}



// HAVOK DO NOT EDIT


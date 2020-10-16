// TKBMS v1.0 -----------------------------------------------------
//
// PLATFORM		: ALL
// PRODUCT		: VPHYSICS
// VISIBILITY	: INTERNAL
//
// ------------------------------------------------------TKBMS v1.0


#include <hk_physics/physics.h>
#include <hk_physics/simunit/psi_info.h>
#include <hk_physics/constraint/fixed/fixed_constraint.h>
#include <hk_physics/constraint/fixed/fixed_bp.h>
#include <hk_physics/core/vm_query_builder/vm_query_builder.h>
#include <hk_math/vector3/vector3_util.h>

#include <hk_math/densematrix_util.h>
#include <hk_math/dense_vector.h>

/* Local class */
#ifdef HK_ARCH_PPC
#include <stddef.h> // for size_t
#endif

// IVP_EXPORT_PUBLIC

class hk_Fixed_Work
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
		hk_Fixed_Dense_Vector<6> m_correction;
		hk_VM_Query_Builder< hk_VMQ_Storage<6> > qe;
};

void hk_Fixed_Constraint::init_constraint( const void* vbp )
{
	const hk_Fixed_BP* bp = static_cast<const hk_Fixed_BP*>(vbp);
	this->init_fixed_constraint(bp);
}

void hk_Fixed_Constraint::init_fixed_constraint( const hk_Fixed_BP *bp )
{
	m_tau = bp->m_tau;
	m_strength = bp->m_strength;
	m_transform_os_ks = bp->m_transform_os_ks;
}

void hk_Fixed_Constraint::write_to_blueprint( hk_Fixed_BP *pOutBP )
{
	pOutBP->m_tau = m_tau;
	pOutBP->m_strength = m_strength;
	pOutBP->m_transform_os_ks = m_transform_os_ks;
}

hk_Fixed_Constraint::hk_Fixed_Constraint(hk_Environment* env, const hk_Fixed_BP *bp, hk_Rigid_Body* a, hk_Rigid_Body* b)
	: hk_Constraint( env, a, b, HK_PRIORITY_LOCAL_CONSTRAINT)
{
	init_fixed_constraint(bp);
}

hk_Fixed_Constraint::hk_Fixed_Constraint(hk_Local_Constraint_System* constraint_system, const hk_Fixed_BP* bp, hk_Rigid_Body* a, hk_Rigid_Body* b )
	: hk_Constraint( constraint_system, a, b, HK_PRIORITY_LOCAL_CONSTRAINT, HK_NEXT_MULTIPLE_OF(16,sizeof(hk_Fixed_Work)))
{
	init_fixed_constraint(bp);
}


int hk_Fixed_Constraint::get_vmq_storage_size()
{
	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Fixed_Work));
}

// setup and step
int hk_Fixed_Constraint::setup_and_step_constraint(hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real strength_factor )
{
	hk_Rigid_Body* b0 = get_rigid_body(0);
	hk_Rigid_Body* b1 = get_rigid_body(1);
	const hk_Transform& t0 = b0->get_cached_transform();
	const hk_Transform& t1 = b1->get_cached_transform();

	// angular part
	hk_Matrix3 joint_axes_ws;
	joint_axes_ws.set_mul3( t0, m_transform_os_ks );

	// linear part
	hk_Vector3 position_ws;
	position_ws._set_transformed_pos( t0, m_transform_os_ks.get_translation());

	// step the constraint
	hk_Fixed_Work &work = *new (mem) hk_Fixed_Work;
	hk_VM_Query_Builder< hk_VMQ_Storage<6> >& qe = work.qe;
	qe.begin(6);
	
	{
		qe.begin_entries(6);
		{
			hk_Mass_Relative_Vector3 mcr_0( b0, position_ws);
			hk_Mass_Relative_Vector3 mcr_1( b1, position_ws);

			qe.add_linear_xyz( 0, HK_X_DIRECTION, HK_BODY_A, b0, mcr_0,  1.0f );
			qe.add_linear_xyz( 0, HK_X_DIRECTION, HK_BODY_B, b1, mcr_1, -1.0f );
	
			qe.add_linear_xyz( 1, HK_Y_DIRECTION, HK_BODY_A, b0, mcr_0,  1.0f );
			qe.add_linear_xyz( 1, HK_Y_DIRECTION, HK_BODY_B, b1, mcr_1, -1.0f );
	
			qe.add_linear_xyz( 2, HK_Z_DIRECTION, HK_BODY_A, b0, mcr_0,  1.0f );
			qe.add_linear_xyz( 2, HK_Z_DIRECTION, HK_BODY_B, b1, mcr_1, -1.0f );

			qe.add_angular(3, HK_BODY_A, b0, joint_axes_ws.get_column(0),  1.0f);
			qe.add_angular(3, HK_BODY_B, b1, joint_axes_ws.get_column(0), -1.0f);

			qe.add_angular(4, HK_BODY_A, b0, joint_axes_ws.get_column(1),  1.0f);
			qe.add_angular(4, HK_BODY_B, b1, joint_axes_ws.get_column(1), -1.0f);

			qe.add_angular(5, HK_BODY_A, b0, joint_axes_ws.get_column(2),  1.0f);
			qe.add_angular(5, HK_BODY_B, b1, joint_axes_ws.get_column(2), -1.0f);
		}
		qe.commit_entries(6);
	}
	
	qe.commit(HK_BODY_A, b0);
	qe.commit(HK_BODY_B, b1);

	const hk_real* vmq_velocity = qe.get_vmq_storage().get_velocities();

	hk_Fixed_Dense_Vector<6> delta;
	{
		hk_Vector3& delta_dist = *(hk_Vector3*)&work.m_correction(0);
		delta_dist.set_sub( t1.get_translation(), position_ws );
		delta_dist.set_mul( pi.get_inv_delta_time(), delta_dist );

		work.m_correction(3) = pi.get_inv_delta_time() * t1.get_column(1).dot(joint_axes_ws.get_column(2));
		work.m_correction(4) = pi.get_inv_delta_time() * -t1.get_column(0).dot(joint_axes_ws.get_column(2));
		work.m_correction(5) = pi.get_inv_delta_time() * t1.get_column(0).dot(joint_axes_ws.get_column(1));

		delta.set_mul_add_mul( m_tau * tau_factor, work.m_correction, -1.0f * m_strength * strength_factor, vmq_velocity );
	}
	
	hk_Fixed_Dense_Matrix<6>& mass_matrix = qe.get_vmq_storage().get_fixed_dense_matrix();
	hk_Dense_Matrix_Util::invert_6x6( mass_matrix, 0.0f );
	
	hk_Fixed_Dense_Vector<6> impulses;
	hk_Dense_Matrix_Util::mult( mass_matrix, delta, impulses );

	qe.apply_impulses( HK_BODY_A, b0, impulses.get_const_real_pointer() );
	qe.apply_impulses( HK_BODY_B, b1, impulses.get_const_real_pointer() );
	
	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Fixed_Work) );
}

// step
void hk_Fixed_Constraint::step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real strength_factor )
{
	hk_Rigid_Body *b0 = get_rigid_body(0);
	hk_Rigid_Body *b1 = get_rigid_body(1);

	hk_Fixed_Work &work = *(hk_Fixed_Work*)mem;
	hk_VM_Query_Builder< hk_VMQ_Storage<6> >& qe = work.qe;
	qe.get_vmq_storage().clear_velocities();
	qe.update_velocities(HK_BODY_A, b0);
	qe.update_velocities(HK_BODY_B, b1);
	
	hk_real* vmq_velocity = qe.get_vmq_storage().get_velocities();
	
	hk_Fixed_Dense_Vector<6> delta;
	delta.set_mul_add_mul( tau_factor, work.m_correction, -1.0f * m_strength * strength_factor, vmq_velocity );
		
	hk_Dense_Matrix& mass_matrix = qe.get_vmq_storage().get_dense_matrix();
	hk_Fixed_Dense_Vector<6> impulses;
	hk_Dense_Matrix_Util::mult( mass_matrix, delta, impulses );
		
	qe.apply_impulses( HK_BODY_A, b0, impulses.get_const_real_pointer() );
	qe.apply_impulses( HK_BODY_B, b1, impulses.get_const_real_pointer() );
}

void hk_Fixed_Constraint::apply_effector_PSI(hk_PSI_Info& pi, hk_Array<hk_Entity*>* )
{
	hk_Fixed_Work work_mem;
	hk_Fixed_Constraint::setup_and_step_constraint( pi,(void *)&work_mem, 1.0f, 1.0f );
}


// HAVOK DO NOT EDIT


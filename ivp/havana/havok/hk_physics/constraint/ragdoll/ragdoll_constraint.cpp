
#include <hk_physics/physics.h>
#include <hk_physics/simunit/psi_info.h>
#include <hk_physics/constraint/ragdoll/ragdoll_constraint.h>
#include <hk_physics/constraint/ragdoll/ragdoll_constraint_bp.h>
#include <hk_physics/core/vm_query_builder/vm_query_builder.h>

#include <hk_math/densematrix_util.h>
#include <hk_math/dense_vector.h>
#include <hk_math/eulerangles.h>
#include <hk_math/quaternion/quaternion_util.h>

#include <hk_physics/constraint/util/constraint_limit_util.h>
#include <hk_physics/constraint/local_constraint_system/local_constraint_system.h>

// IVP_EXPORT_PUBLIC

#ifdef HK_ARCH_PPC
#include <stddef.h> // for size_t
#endif


/****************************************************************
 * Local class
 */ 
class hk_Ragdoll_Constraint_Work
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
		hk_VM_Query_Builder< hk_VMQ_Storage<3> > query_engine_linear;

		/* angular*/
		hk_Vector3 joint_angles;
		hk_Vector3 twist_axis_ws;	// w part is the length

		hk_Vector3 twist_axis_Ref_ws;
		hk_Vector3 twist_axis_Att_ws;

		hk_Vector3 plane_Axis_Ref_ws;

		hk_VM_Query_Builder< hk_VMQ_Storage<1> > query_engine_angle[3];

		inline hk_real get_twist_axis_length(){
			return twist_axis_ws.w;
		}	
};


/****************************************************************
 * Constructors
 */

void hk_Ragdoll_Constraint::init_constraint(const void* vbp)
{
	const hk_Ragdoll_Constraint_BP* bp = static_cast<const hk_Ragdoll_Constraint_BP*>(vbp);
	this->init_ragdoll_constraint(bp);
}

void hk_Ragdoll_Constraint::init_ragdoll_constraint(const hk_Ragdoll_Constraint_BP *bp, hk_Local_Constraint_System *sys)
{
	m_transform_os_ks[0]	= bp->m_transform_os_ks[0];
	m_transform_os_ks[1]	= bp->m_transform_os_ks[1];

	m_tau					= bp->m_tau;
	m_strength				= bp->m_strength;
	m_constrainTranslation  = bp->m_constrainTranslation;

	const hk_real factor = (sys) ? sys->get_epsilon() : 1.0f;
	for(int i=0; i<3; i++)
	{
		m_limits[i].init_angular_limit(bp->m_limits[i], factor);
	}

	// check for unsymmetric cone limits
	hk_real center_cone = (bp->m_limits[HK_LIMIT_CONE].m_limit_max  + bp->m_limits[HK_LIMIT_CONE].m_limit_min) * 0.5f;
	hk_real diff_cone   =  bp->m_limits[HK_LIMIT_CONE].m_limit_max  - bp->m_limits[HK_LIMIT_CONE].m_limit_min;

	if ( hk_Math::fabs( center_cone ) > 0.001f ){
		m_transform_os_ks[0].rotate( 2, center_cone);
	}
	m_limits[HK_LIMIT_CONE].m_limit_min = hk_Math::cos(0.5f * diff_cone);
	m_limits[HK_LIMIT_CONE].m_limit_max = 100.0f;


	m_limits[HK_LIMIT_PLANES].m_limit_min = -hk_Math::sin( bp->m_limits[HK_LIMIT_PLANES].m_limit_max );
	m_limits[HK_LIMIT_PLANES].m_limit_max = -hk_Math::sin( bp->m_limits[HK_LIMIT_PLANES].m_limit_min );
	// note: the max min and directions are reversed !!!!!!!!!
	m_inputLimits[0] = bp->m_limits[0];
	m_inputLimits[1] = bp->m_limits[1];
	m_inputLimits[2] = bp->m_limits[2];
}

void hk_Ragdoll_Constraint::write_to_blueprint( hk_Ragdoll_Constraint_BP *bp )
{
	bp->m_transform_os_ks[0] = m_transform_os_ks[0];
	bp->m_transform_os_ks[1] = m_transform_os_ks[1];
	bp->m_limits[0] = m_inputLimits[0];
	bp->m_limits[1] = m_inputLimits[1];
	bp->m_limits[2] = m_inputLimits[2];
	bp->m_strength = m_strength;
	bp->m_tau = m_tau;
}


hk_Ragdoll_Constraint::hk_Ragdoll_Constraint(
		hk_Environment *env,
		const hk_Ragdoll_Constraint_BP *bp,
		hk_Rigid_Body* a,
		hk_Rigid_Body* b )
	: hk_Constraint( env, a, b, HK_PRIORITY_LOCAL_CONSTRAINT)
{	
	init_ragdoll_constraint(bp, HK_NULL);
}


hk_Ragdoll_Constraint::hk_Ragdoll_Constraint( 
		hk_Local_Constraint_System* constraint_system,
		const hk_Ragdoll_Constraint_BP* bp,
		hk_Rigid_Body* a ,
		hk_Rigid_Body* b )
	: hk_Constraint( constraint_system, a, b,
			HK_PRIORITY_LOCAL_CONSTRAINT,
			HK_NEXT_MULTIPLE_OF(16,sizeof(hk_Ragdoll_Constraint_Work)))
{
	init_ragdoll_constraint(bp, constraint_system);
}


int hk_Ragdoll_Constraint::get_vmq_storage_size()
{
	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Ragdoll_Constraint_Work));
}

/********************************************************************************
 * Common function between setup_ang_step and step_constraint
 */

void hk_Ragdoll_Constraint::apply_angular_part( hk_PSI_Info &pi, hk_Ragdoll_Constraint_Work& work,
												hk_real tau_factor, hk_real strength_factor)
{
	hk_Rigid_Body* b0 = get_rigid_body(0);
	hk_Rigid_Body* b1 = get_rigid_body(1);
	// twist first
	hk_real twist_factor = work.get_twist_axis_length();
	hk_Constraint_Limit_Util::do_angular_limit( pi, b0, work.twist_axis_ws, work.joint_angles( HK_LIMIT_TWIST ), b1,
		m_limits[ HK_LIMIT_TWIST], tau_factor * twist_factor, strength_factor * twist_factor );

	// planar parts
	int i = 2; do {
		hk_VM_Query_Builder< hk_VMQ_Storage<1> >& qe = work.query_engine_angle[i];
		hk_Constraint_Limit &limit = m_limits[ i ];

		hk_real current_pos = work.joint_angles( i );

		hk_Vector3 &plane_dir_ws = (i==HK_LIMIT_CONE) ? work.twist_axis_Ref_ws : work.plane_Axis_Ref_ws;

		hk_Vector3 &dir_ws = work.twist_axis_Att_ws;

		hk_Constraint_Limit_Util::do_angular_plane_limit( pi, b0, b1, dir_ws, plane_dir_ws, current_pos, limit, tau_factor, strength_factor);
	}while (--i >=1 );
}



/********************************************************************************
 * SETUP AND STEP
 */

int	hk_Ragdoll_Constraint::setup_and_step_constraint(
		hk_PSI_Info& pi, void *mem,
		hk_real tau_factor, hk_real strength_factor )
{
	hk_Ragdoll_Constraint_Work& work = *new (mem) hk_Ragdoll_Constraint_Work;
	hk_Rigid_Body* b0 = get_rigid_body(0);
	hk_Rigid_Body* b1 = get_rigid_body(1);

	const hk_Transform &t0 = b0->get_cached_transform();
	const hk_Transform &t1 = b1->get_cached_transform();

	{ /* setup and step ANGULAR PARTS */
		/* get the body joint axes, if specified */
		work.twist_axis_Ref_ws.set_mul3( t0, m_transform_os_ks[0].get_column(0));
		work.plane_Axis_Ref_ws.set_mul3( t0, m_transform_os_ks[0].get_column(2));
		work.twist_axis_Att_ws.set_mul3( t1, m_transform_os_ks[1].get_column(0));

		/* get the relative joint axes and angles */
		{ // twist setup
			work.twist_axis_ws.set_add( work.twist_axis_Ref_ws, work.twist_axis_Att_ws);
			hk_real length = work.twist_axis_ws.length();
			hk_real length_inv = 1.0f / length;

			work.twist_axis_ws *= length_inv;
			work.twist_axis_ws.w = length * 0.5f; // the tau for the twist

			hk_Rotation m_ws_us;
			m_ws_us.get_column(0) = work.twist_axis_ws;
			m_ws_us.get_column(1).set_cross ( work.twist_axis_ws, work.plane_Axis_Ref_ws );
			m_ws_us.get_column(1).normalize();
			m_ws_us.get_column(2).set_cross ( m_ws_us.get_column(1), m_ws_us.get_column(0) );

			hk_Vector3 perp_axes_Att_ws; perp_axes_Att_ws.set_rotated_dir( t1, m_transform_os_ks[1].get_column(2) );

			hk_Vector3 a_us; a_us.set_rotated_inv_dir ( m_ws_us, perp_axes_Att_ws );

			work.joint_angles( HK_LIMIT_TWIST ) = -hk_Math::atan2( a_us.y, a_us.z);
		}
		// planes setup
		work.joint_angles( HK_LIMIT_CONE )   = work.twist_axis_Ref_ws.dot( work.twist_axis_Att_ws );
		work.joint_angles( HK_LIMIT_PLANES ) = work.plane_Axis_Ref_ws.dot( work.twist_axis_Att_ws );

		this->apply_angular_part(pi, work, tau_factor, strength_factor );
	}

	if ( m_constrainTranslation )
	{	/* setup and step LINEAR PART */
		hk_VM_Query_Builder< hk_VMQ_Storage<3> > &query_engine = work.query_engine_linear;

		hk_Vector3 position_ws[2];
		position_ws[0]._set_transformed_pos( t0,	m_transform_os_ks[0].get_translation());
		position_ws[1]._set_transformed_pos( t1, 	m_transform_os_ks[1].get_translation());

		hk_Vector3 &dir = work.dir;
		dir.set_sub( position_ws[1], position_ws[0] );
#if 0

		// UNDONE: store per joint rescue teleport distance squared?
		// UNDONE: Then enable this to fix stretchy ragdolls?
		if ( dir.length_squared() > 0.01 )
		{
			IVP_U_Quat rot;
			IVP_U_Point position;
			b0->get_quat_world_f_object_AT( &rot, &position );
			position.k[0] = position_ws[1].x;
			position.k[1] = position_ws[1].y;
			position.k[2] = position_ws[1].z;
			b0->beam_object_to_new_position( &rot, &position, IVP_FALSE );
			position_ws[0] = position_ws[1];
			dir.set_sub( position_ws[1], position_ws[0] );
		}
#endif

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
		delta_dist_3.set_mul( tau_factor * m_tau * pi.get_inv_delta_time(), dir );
		delta_dist_3.add_mul( -1.0f * m_strength * strength_factor, *(const hk_Vector3 *)approaching_velocity );

		hk_Fixed_Dense_Matrix<3>& mass_matrix = query_engine.get_vmq_storage().get_fixed_dense_matrix();

		hk_Dense_Matrix_Util::invert_3x3_symmetric( mass_matrix, 0.0f );
		hk_Vector3 impulses;
		hk_Dense_Matrix_Util::mult_3_symmetric( mass_matrix, delta_dist_3, impulses);
		query_engine.apply_impulses( HK_BODY_A, b0, (hk_real *)&impulses(0) );
		query_engine.apply_impulses( HK_BODY_B, b1, (hk_real *)&impulses(0) );
	}

	return HK_NEXT_MULTIPLE_OF(16, sizeof(hk_Ragdoll_Constraint_Work));
}

/********************************************************************************
 * STEP
 */

void hk_Ragdoll_Constraint::step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real strength_factor )
{
	hk_Rigid_Body *b0 = get_rigid_body(0);
	hk_Rigid_Body *b1 = get_rigid_body(1);
	hk_Ragdoll_Constraint_Work& work = *(hk_Ragdoll_Constraint_Work*)mem;

	this->apply_angular_part(pi, work, tau_factor, strength_factor );

	if ( m_constrainTranslation )
	{ /* step LINEAR */
		hk_Vector3 &dir = work.dir;
		hk_VM_Query_Builder< hk_VMQ_Storage<3> > &query_engine = work.query_engine_linear;

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

/****************************************************************
 * Apply constraint only once
 */

void hk_Ragdoll_Constraint::apply_effector_PSI(
		hk_PSI_Info& pi, hk_Array<hk_Entity*>* )
{
	hk_Ragdoll_Constraint_Work work_mem;
	hk_Ragdoll_Constraint::setup_and_step_constraint( pi,(void *)&work_mem, 1.0f, 1.0f );
}

// HAVOK DO NOT EDIT


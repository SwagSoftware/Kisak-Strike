#include <hk_physics/physics.h>
#include <hk_physics/constraint/hinge/hinge_bp_builder.h>

// IVP_EXPORT_PUBLIC

void hk_Hinge_BP_Builder::set_position_os( int body_index, const hk_Vector3 &position )
{
	m_hinge_bp.m_axis_os[ body_index ].m_origin = position;
}

void hk_Hinge_BP_Builder::set_axis_ws( hk_Rigid_Body *a, hk_Rigid_Body *b, const hk_Vector3& axis_ws )
{
	hk_Vector3 n_axis = axis_ws;
	n_axis.normalize();
	m_hinge_bp.m_axis_os[ 0 ].m_direction.set_rotated_inv_dir( a->get_cached_transform(), n_axis );
	m_hinge_bp.m_axis_os[ 1 ].m_direction.set_rotated_inv_dir( b->get_cached_transform(), n_axis );
	
}

//lwss add - rebuilt from debug bins
void hk_Hinge_BP_Builder::set_axis_perp_os(int obj_index, const hk_Vector3 &perp_os)
{
    float normalizedLength = 1.0f / sqrtf(perp_os.length_squared());
    m_hinge_bp.m_axis_perp_os[ obj_index ].x = (perp_os.x * normalizedLength);
    m_hinge_bp.m_axis_perp_os[ obj_index ].y = (perp_os.y * normalizedLength);
    m_hinge_bp.m_axis_perp_os[ obj_index ].z = (perp_os.z * normalizedLength);
}
//lwss end

void hk_Hinge_BP_Builder::set_angular_motor(hk_real angular_velocity, hk_real max_torque)
{
	m_hinge_bp.m_limit.set_motor( angular_velocity, max_torque );
}

void hk_Hinge_BP_Builder::set_angular_limits( hk_real lower, hk_real upper )
{
	m_hinge_bp.m_limit.set_limits( lower, upper );
}

void hk_Hinge_BP_Builder::set_joint_friction(hk_real friction)
{
	m_hinge_bp.m_limit.set_friction( friction );
}

void hk_Hinge_BP_Builder::set_tau( hk_real tau)
{
	m_hinge_bp.m_tau = tau;
}

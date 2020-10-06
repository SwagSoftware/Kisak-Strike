#include <hk_physics/physics.h>
#include <hk_physics/constraint/ragdoll/ragdoll_constraint_bp_builder.h>
#include <hk_physics/constraint/limited_ball_socket/limited_ball_socket_bp.h>
#include <hk_physics/core/vm_query_builder/vm_query_builder.h>

// IVP_EXPORT_PUBLIC

hk_result hk_Ragdoll_Constraint_BP_Builder::initialize_from_limited_ball_socket_bp( const hk_Limited_Ball_Socket_BP *bp, hk_Rigid_Body *a, hk_Rigid_Body *b)
{
	hk_Ragdoll_Constraint_BP& r = this->m_ragdoll_constraint_bp;
	r.m_transform_os_ks[0] = bp->m_transform_os_ks[0];
	r.m_transform_os_ks[1] = bp->m_transform_os_ks[1];

	r.m_constrainTranslation = bp->m_constrainTranslation;
	// ok, let's analyse the system
	hk_real limit_diff[3];
	hk_real limit_mid[3];
	int number_of_freedom = 0;
	{
		for (int i = 0; i<3; i++)
		{
			limit_diff[i] = bp->m_angular_limits[i].m_max - bp->m_angular_limits[i].m_min;
			limit_mid[i] = 0.5f * (bp->m_angular_limits[i].m_max + bp->m_angular_limits[i].m_min);
			if ( limit_diff[i] > HK_REAL_EPS)
			{
				number_of_freedom++;
			}
		}
	}
	{ // set all limits to 0
		for (int i = 0; i<3; i++)
		{
			r.m_limits[i].set_limits( 0.0f, 0.0f);
		}
	}

	switch ( number_of_freedom )
	{
		case 0: // shit 
		{
			r.m_limits[ HK_LIMIT_CONE ].set_limits( -0.1f, 0.1f);
			return HK_FAULT;
		}

		case 1:
		{	// limited hinge
			// search axis with the freedom
			int axis = -1;
			{
				for ( int i = 0; i<3; i++)
				{
					if ( limit_diff[i] > HK_REAL_EPS)
					{
						axis = i;
					}
				}
			}
			hk_Rotation m0 = r.m_transform_os_ks[0];
			hk_Rotation m1 = r.m_transform_os_ks[1];

			// rotate matrix to center axis
			m0.rotate( axis, limit_mid[axis]);

			// resort matrix and limits
			int naxis = (axis == 2)? 0 : axis+1; // next axis
			int paxis = (axis == 0)? 2 : axis-1; // previous axis

			r.m_transform_os_ks[0].get_column(0) = m0.get_column( naxis );
			r.m_transform_os_ks[1].get_column(0) = m1.get_column( naxis );

			r.m_transform_os_ks[0].get_column(1) = m0.get_column( paxis );
			r.m_transform_os_ks[1].get_column(1) = m1.get_column( paxis );

			r.m_transform_os_ks[0].get_column(2) = m0.get_column( axis );
			r.m_transform_os_ks[1].get_column(2) = m1.get_column( axis );

			r.m_limits[ HK_LIMIT_CONE ].set_limits( -limit_diff[axis] * 0.5f, limit_diff[axis] * 0.5f);
			break;
		}
		
		case 2:
		case 3:
		{
			{ // first search the axis with the lowest rot inertia, which is should be used for the twist axis in Aos
				hk_real min_inertia_inv = -1.0f;
				int axis_of_min_inertia = 0;
				const hk_Transform &t0 = a->get_cached_transform();
				const hk_Transform &t1 = b->get_cached_transform();

				// get the pivot point
				hk_Vector3 pos_Ref_ws;  pos_Ref_ws.set_transformed_pos( t0, r.m_transform_os_ks[0].get_translation());
				hk_Vector3 pos_Att_ws;  pos_Att_ws.set_transformed_pos( t1, r.m_transform_os_ks[1].get_translation());

				// subtract the center of mass
				pos_Ref_ws -= a->get_center_of_mass();
				pos_Att_ws -= b->get_center_of_mass();

				for (int i = 0; i< 3; i++)
				{
					// get the axis
					hk_Vector3 axis_Ref_ws; axis_Ref_ws.set_rotated_dir( t0, r.m_transform_os_ks[0].get_column( i ));	
					hk_Vector3 axis_Att_ws; axis_Att_ws.set_rotated_dir( t1, r.m_transform_os_ks[1].get_column( i ));

					hk_VM_Query_Builder< hk_VMQ_Storage<1> > query;
					query.begin(1);
					{
						query.begin_entries(1);
						query.add_angular( 0, HK_BODY_A, a, axis_Ref_ws,  1.0f );
						query.add_angular( 0, HK_BODY_B, b, axis_Ref_ws,  -1.0f );
						query.commit_entries(1);
					}
					query.commit( HK_BODY_A, a );
					query.commit( HK_BODY_B, a );
					hk_real inv_virt_mass = query.get_vmq_storage().get_dense_matrix()(0,0);

					hk_Vector3 cross_a; cross_a.set_cross( pos_Ref_ws, axis_Ref_ws);
					hk_Vector3 cross_b; cross_b.set_cross( pos_Att_ws, axis_Att_ws);
					
					hk_Rigid_Body_Core  *core_a = a->get_rigid_body_core();
					hk_Rigid_Body_Core *core_b = b->get_rigid_body_core();

					inv_virt_mass += cross_a.length_squared() * core_a->get_inv_mass();
					inv_virt_mass += cross_b.length_squared() * core_b->get_inv_mass();

					if ( inv_virt_mass > min_inertia_inv)
					{
						min_inertia_inv = inv_virt_mass;
						axis_of_min_inertia = i;
					}
				}

				// now we now the twist axis

				// next search for the bigger of the rest axis
				int l_axis = (axis_of_min_inertia == 2)? 0 : axis_of_min_inertia+1; // axis with the lower diff
				int u_axis = (axis_of_min_inertia == 0)? 2 : axis_of_min_inertia-1; // axis with the higher diff
				if ( limit_diff[ l_axis] > limit_diff[ u_axis ])
				{
					int x = l_axis; l_axis = u_axis; u_axis = x;
				}

				hk_Rotation m0 = r.m_transform_os_ks[0];
				hk_Rotation m1 = r.m_transform_os_ks[1];

				// now use u_axis for cone
				m0.rotate( u_axis, limit_mid[u_axis]);

				// resort matrix and limits
				r.m_transform_os_ks[0].get_column(0) = m0.get_column( axis_of_min_inertia );
				r.m_transform_os_ks[1].get_column(0) = m1.get_column( axis_of_min_inertia );

				r.m_transform_os_ks[0].get_column(1) = m0.get_column( u_axis );
				r.m_transform_os_ks[1].get_column(1) = m1.get_column( u_axis );

				r.m_transform_os_ks[0].get_column(2) = m0.get_column( l_axis );
				r.m_transform_os_ks[1].get_column(2) = m1.get_column( l_axis );

				r.m_limits[ HK_LIMIT_TWIST ].set_limits( bp->m_angular_limits[axis_of_min_inertia].m_min, bp->m_angular_limits[axis_of_min_inertia].m_max );
				r.m_limits[ HK_LIMIT_CONE ]. set_limits( - limit_diff[u_axis] * 0.5f, limit_diff[u_axis] * 0.5f);
				r.m_limits[ HK_LIMIT_PLANES].set_limits( bp->m_angular_limits[l_axis].m_min, bp->m_angular_limits[l_axis].m_max );
			}
			break;
		}

		default:
		{
			return HK_FAULT;
			break;
		}
	}
	return HK_OK;
}


void hk_Ragdoll_Constraint_BP_Builder::set_ragdoll_constraint( 
		hk_Rigid_Body *a, hk_Rigid_Body *b,
		const hk_Vector3 &pivot_point_ws,
		const hk_Vector3 &primary_axis_ws, const hk_Vector3 &planes_axis_ws,
		const hk_Interval<hk_real> &twist_limits, 
		const hk_Interval<hk_real> &cone_limits,
		const hk_Interval<hk_real> &plane_limits,
		bool constrainTranslation)
{
	hk_Ragdoll_Constraint_BP &r = this->m_ragdoll_constraint_bp;
	
	const hk_Transform &t0 = a->get_cached_transform();
	const hk_Transform &t1 = b->get_cached_transform();

	hk_Transform &m0 = r.m_transform_os_ks[0];
	hk_Transform &m1 = r.m_transform_os_ks[1];

	hk_Vector3 c0; c0 =  primary_axis_ws; c0.normalize();
	hk_Vector3 c2; c2 =  planes_axis_ws; c2.normalize();
	hk_Vector3 c1; c1.set_cross( c0, c2 );

	m0.get_column(0).set_rotated_inv_dir( t0, c0 );
	m0.get_column(1).set_rotated_inv_dir( t0, c1 );
	m0.get_column(2).set_rotated_inv_dir( t0, c2 );
	m0.get_column(3).set_transformed_inv_pos( t0, pivot_point_ws );

	m1.get_column(0).set_rotated_inv_dir( t1, c0 );
	m1.get_column(1).set_rotated_inv_dir( t1, c1 );
	m1.get_column(2).set_rotated_inv_dir( t1, c2 );
	m1.get_column(3).set_transformed_inv_pos( t1, pivot_point_ws );

	r.m_limits[ HK_LIMIT_TWIST ].set_limits( twist_limits.m_min, twist_limits.m_max );
	r.m_limits[ HK_LIMIT_CONE ]. set_limits( cone_limits.m_min, cone_limits.m_max);
	r.m_limits[ HK_LIMIT_PLANES].set_limits( plane_limits.m_min, plane_limits.m_max );
	r.m_constrainTranslation = constrainTranslation;
}

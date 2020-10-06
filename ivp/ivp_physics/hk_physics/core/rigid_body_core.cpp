#include <ivp_physics.hxx>
#include <hk_physics/physics.h>


void hk_Rigid_Body_Core::add_to_mass_matrix_inv(
		hk_Core_VMQ_Input &input,
		hk_Dense_Matrix& matrix_out,
		hk_real  velocities_out[])
{
	if ( physical_unmoveable ) return;
	hk_Single_Rigid_Body_CFAD *delta_spins = (hk_Single_Rigid_Body_CFAD *)input.m_buffer;

	{ // first step multiply all angular forces with the inertia tensor and calculate the velocities
		int i = 0;
		hk_Virtual_Mass_Query *mq_a = input.m_vmq;
		hk_Single_Rigid_Body_CFAD *ds_a = &delta_spins[0];
		hk_real *vel = &velocities_out[0];

		do {
			hk_real spin_projected_vel   = mq_a->m_angular.dot( _get_spin() );  // IPION
			hk_real linear_projected_vel = mq_a->m_linear.dot( _get_linear_velocity() );		// IPION

			ds_a->m_delta_spin.set_mul3( _get_inv_body_inertia(), mq_a->m_angular );

			int i_dest_index = mq_a->m_matrix_index;
			vel[i_dest_index] +=  spin_projected_vel + linear_projected_vel;  // Note: a double dot will perform faster on PS2

			hk_Virtual_Mass_Query *mq_b = input.m_vmq;

			for ( int j = i-1; j>=0; j--){
				hk_real spin_part = ds_a->m_delta_spin.dot( mq_b->m_angular );  // Note double dot will be faster on PS2
				hk_real linear_part = get_inv_mass() * mq_a->m_linear.dot( mq_b->m_linear );
				int j_dest_index = mq_b->m_matrix_index;
				matrix_out(i_dest_index,j_dest_index) +=  spin_part + linear_part;
				matrix_out(j_dest_index,i_dest_index) +=  spin_part + linear_part;
				mq_b++;
			}

			hk_real spin_part = ds_a->m_delta_spin.dot( mq_a->m_angular );  // Note double dot will be faster on PS2
			hk_real linear_part = get_inv_mass() * mq_a->m_linear.dot( mq_a->m_linear );
			matrix_out(i_dest_index,i_dest_index) +=  spin_part + linear_part;
			
			mq_a ++;
			ds_a ++;
		} while ( ++i < input.m_n_queries );
	}
}

void hk_Rigid_Body_Core::apply_impulses( hk_Core_VMQ_Input &input,
		const hk_real impulse_strength[])
{
	if ( physical_unmoveable ) return;
	int i = input.m_n_queries-1;
	hk_Impulse_Info *mq = input.m_vmq;
	hk_Single_Rigid_Body_CFAD *ds = (hk_Single_Rigid_Body_CFAD *)input.m_buffer;
	do {
		_get_spin().add_mul(   impulse_strength[mq->m_matrix_index], ds->m_delta_spin );
		_get_linear_velocity().add_mul( impulse_strength[mq->m_matrix_index] * this->get_inv_mass(), mq->m_linear );
		mq ++;
		ds ++;
	} while ( --i >= 0);
}

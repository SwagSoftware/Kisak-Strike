#ifndef HK_PHYSICS_CORE_H
#define HK_PHYSICS_CORE_H

class hk_Entity;
class hk_Synchronized_Core;
class hk_PSI_Info;
class hk_PSI_Event_Collector;
class hk_Dense_Matrix;

#include <hk_physics/core/vm_query.h>


//: Internal class which is responsible for the integration,
// force calculation and interpolating of a group of rigid bodies,
// Note: this class is not to be used by simple users
class hk_Core: public hk_Object
{
	public:

		inline virtual ~hk_Core(){}
		inline hk_Core(hk_Environment *env): hk_Object(env){;}

		enum HK_CORE_MOVEMENT_STATE
		{
			HK_CORE_MOVING,
			HK_CORE_SLOW,
			HK_CORE_STILL
		};

		struct hk_Worst_Case
		{
			hk_real m_worst_case_velocity;
			hk_real m_worst_case_acceleration;
		};
		
	public:
		virtual hk_real get_mass(hk_Entity_Core_ID) const = 0;
		virtual void set_mass(hk_Entity_Core_ID, hk_real) = 0;


		/*** for collision events (which are outside the PSI) ***/

		virtual hk_Synchronized_Core* begin_synchronize( hk_PSI_Info & ) = 0;
			//: change the velocities to reflect the simplified interpolated movement used for collision detection
			// called at the beginning of a collision

		virtual hk_Synchronized_Core *init_for_PSI( hk_PSI_Info & ) = 0;
			//: called at beginning of each PSI
			//: called to update internal caches to optimize the hk_Synchronized_Core function calls

		virtual void update_body_transforms_and_positions(hk_timestamp time_stamp, hk_time ) = 0;
			//: updates all transforms of all controlled rigid bodies

		virtual void async_apply_impulse( hk_Impulse_Info &impulse, hk_real impulse_strength ) = 0;
		//: apply an impulse which will become effective at the next commit_async_impulses() call

		//virtual void async_apply_impulses( int n_impulses, hk_Impulse_Info axis[], hk_real impulse_strength[] );

		/* for collision events that are (possibly) inside the PSI */
		virtual void get_worst_case_values( hk_Entity *entity, hk_Vector3& separating_plane_norm, hk_Worst_Case& wc ) = 0;
			//: this method will give back the worst case vel. and accel. tangents (to the separating plane).
	
		virtual void set_simulation_delta_time( hk_real dt ){;}
	protected:

		friend class hk_Sim_Unit;

		virtual void freeze_core() = 0;
			//: sets all velocities to zero and transform1=transform0.
			// With this, later extrapolation is avoided when core is queried.

		virtual void defreeze_core() = 0;
			//: just to inform simulation that velocities might change again.

};


//: the next functions require that the core is synchronized, either by explicitely calling synchronize core
// or during a PhysicalSynchronousInstance (PSI)    
class hk_Synchronized_Core : public hk_Core
{
	public:
		inline virtual ~hk_Synchronized_Core(){;}
		hk_Synchronized_Core( hk_Environment *env): hk_Core(env){;}
		virtual void speculative_step(	hk_PSI_Info&,
										hk_PSI_Event_Collector &)=0;
			//: Predicts the new transforms of the hk_Bodies at the next PSI.
			//: At the beginning of a new time period do_step has to be called to
			//: set transform0=transform1 and transform1+=delta_time * velocity

		virtual void abort_synchronize( hk_PSI_Info & ) = 0;
			//: abort synchronize, undo the begin_synchronize() call

		virtual void commit_synchronize(hk_PSI_Info &) = 0;
			//: tell the core that no abort_synchronize() will be called

		virtual HK_CORE_MOVEMENT_STATE check_for_movement(hk_PSI_Info &) = 0;
			//: check the current velocities or check against a reference position

		virtual void add_to_mass_matrix_inv( hk_Core_VMQ_Input &input,hk_Dense_Matrix &matrix_out, hk_real velocities_out[]){;};
		virtual void add_to_velocities(      hk_Core_VMQ_Input &input, hk_real velocities_out[]){;};

		
		virtual void apply_impulses( hk_Core_VMQ_Input &input,
				const hk_real impulse_strength[])
		{
		}


		//virtual void add_to_sparse_mass_matrix_inv( int n_axis, hk_Impulse_Info axis[], hk_sparse_matrix &matrix_out) = 0;

		//virtual void build_3x3_mass_matrix_inv( hk_Vector3 &position_ws, hk_Matrix3 &inv_mass_matrix_out ) const = 0;
		//: build a 3x3 mass matrix for the 3 world space directions at a given position

		//virtual void get_projected_velocities( int n_vel, hk_Impulse_Info axis[], hk_real velocities_out[]) const = 0;

		//virtual void get_velocity( hk_Body &body, hk_Vector3 &position_ws, hk_Vector3 &velocity_out) =0;
		//: get the vecocity at a given point



		virtual void apply_impulses( int n_impulses,	hk_Impulse_Info axis[],		const hk_real impulse_strength[] )
		{

		}
			//: apply a list of impulses

		virtual void commit_async_impulses(const hk_Vector3& gravity_impulse) = 0;
			//: e.g. add the accumulated async velocities to the current velocities

		//virtual void abort_async_impulses() = 0;
		//: e.g. clear the accumulated async velocities
};

/******************************* Notes ***************************************
*     Typical usage of core for collisions:
*              begin_synchronize
*                  hk_Sim_Unit::simulate_collision
*                  get_projected_velocities
*                  add_to_mass_matrix_inv
*                   ... collision resolution
*              commit_synchronize
*              apply_impulses
*              speculative_step
******************************************************************************/

 
/******************************** Notes ***************************************
*     Typical usage of core at PSI:
*              init_for_PSI
*                  hk_Sim_Unit::simulate_PSI
*				   commit_async_impulses
*                  get_projected_velocities
*                  add_to_mass_matrix_inv
*                  collision resolution
*                  apply_impulses
*              speculative_step
******************************************************************************/

#endif /* HK_PHYSICS_CORE_H */

#ifndef HK_PHYSICS_RIGID_BODY_CORE_H
#define HK_PHYSICS_RIGID_BODY_CORE_H

#ifndef HK_PHYSICS_CORE_H
#include <hk_physics/core/core.h>
#endif


class hk_Rigid_Body;
class hk_Rigid_Body_BP;

#define HK_CORE_NUM_STATES 2


//: Internal class which is responsible for the integration,
// force calculation and interpolating of a group of rigid bodies,
// Note: this class is not to be used by simple users
class hk_Rigid_Body_Core : public hk_Synchronized_Core
{
	public:

		inline virtual ~hk_Rigid_Body_Core(){;};
		inline hk_Rigid_Body_Core( hk_Environment *env) : hk_Synchronized_Core(env){;};
			//:
				
		virtual void calc_transform(hk_Entity*, hk_time, hk_Transform& ) = 0;
			//: updates a single transform for a body

		virtual void get_linear_velocity(hk_Entity_Core_ID, hk_Vector3& vel_out_ws) const = 0;	

		virtual void get_angular_velocity(hk_Entity_Core_ID, hk_Vector3& avel_out_ws) const = 0;
	
		virtual void warp_to_position( hk_Entity *, const hk_Vector3& p ) =0;
			//: set position and advance tim of entity

		virtual void detach_rigid_body(hk_Entity_Core_ID) = 0;				// e.g called by the deconstructor of the entity
		virtual hk_Entity_Core_ID attach_rigid_body(hk_Rigid_Body *,const hk_Rigid_Body_BP *) = 0;       // e.g called by the deconstructor of the entity

		HK_NEW_DELETE_FUNCTION_VIRTUAL_CLASS( hk_Rigid_Body_Core, HK_MEMORY_CLASS_CORE ); // e.g. defines new as   new (hk_Memory *)hk_Class_Name

		virtual hk_real get_inv_mass(){ return 0.0f; }

	protected:
};


// the next functions require that the core is synchronized
// either by explicitely calling synchronize core
// or during a PhysicalSynchronousInstance (PSI)    
class hk_Synchronized_Rigid_Body_Core : public hk_Rigid_Body_Core
{
	public:
		inline hk_Synchronized_Rigid_Body_Core(hk_Environment *env) : hk_Rigid_Body_Core(env){;}
};

#include <hk_physics/core/rigid_body_core.inl>

#endif // HK_PHYSICS_RIGID_BODY_CORE_H

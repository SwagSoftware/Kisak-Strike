#ifndef IVP_PHYSICS_PHYSICS_H
#define IVP_PHYSICS_PHYSICS_H

#include <hk_base/base.h>
#include <hk_math/vecmath.h>
#include <hk_math/densematrix.h>

#include <hk_physics/constraint/constraint_limit.h>

#include <ivp_physics.hxx>
#include <ivu_matrix_macros.hxx>
#include <ivp_controller.hxx>


class hk_Rigid_Body;

typedef IVP_CONTROLLER_PRIORITY hk_effector_priority;
typedef hk_int32 hk_Entity_Core_ID;


#define HK_PRIORITY_LOCAL_CONSTRAINT_SYSTEM IVP_CP_CONSTRAINTS
#define HK_PRIORITY_LOCAL_CONSTRAINT IVP_CP_CONSTRAINTS
#define HK_PRIORITY_NONE			IVP_CP_NONE
class hk_Environment : public IVP_Environment
{
public:
	hk_Environment(const IVP_Environment &env):IVP_Environment(env){}

};

class hk_Rigid_Body: public IVP_Real_Object
{
public:
	hk_Rigid_Body(const IVP_Real_Object &real):IVP_Real_Object(real) {}

	/**************************************************************************************
	 *	Section:	    havana	
	 *	Description:	Havana compatibility functions
	 **************************************************************************************/
	inline const hk_Vector3 get_center_of_mass(){
		 return hk_Vector3(&get_core()->get_position_PSI()->k[0]); 
	}

	inline hk_Transform get_cached_transform(){
		hk_Transform t;
		if ( this->flags.shift_core_f_object_is_zero )
		{
			this->get_core()->get_m_world_f_core_PSI()->get_4x4_column_major( (hk_real *)&t );
		}
		else
		{
			IVP_U_Matrix coreShiftMatrix;
			coreShiftMatrix.set_matrix( this->get_core()->get_m_world_f_core_PSI() );
			coreShiftMatrix.vmult4( this->get_shift_core_f_object(), &coreShiftMatrix.vv );
			coreShiftMatrix.get_4x4_column_major( (hk_real *)&t );
		}
		return t;
	}
	inline hk_Entity_Core_ID get_entity_core_id() {
		return 0;
	}

	class hk_Rigid_Body_Core *get_rigid_body_core(){
		return (hk_Rigid_Body_Core *)this->get_core();
	}

	inline float get_mass() { return this->get_core()->get_mass(); }
};

#define HK_TRANSFORM_TO_CORE_SPACE(body, vec) body->get_core()->get_m_world_f_core_PSI()->inline_vimult3((IVP_U_Float_Point *)&vec, (IVP_U_Float_Point *)&vec);
//#define HK_TRANSFORM_TO_CORE_SPACE(body, vec) vec.set_zero();

#include <hk_physics/core/vm_query.h>
class hk_Dense_Matrix;


class hk_Single_Rigid_Body_CFAD
{
public:
    hk_Vector3 m_delta_spin; // based on a unit push
};


class hk_Rigid_Body_Core: protected IVP_Core {
public:
	hk_Rigid_Body_Core(const IVP_Core &core): IVP_Core(core) {} 

	hk_Vector3 &_get_spin(){
		return *(hk_Vector3*)&rot_speed;
	}

	hk_Vector3 &_get_linear_velocity(){
		return *(hk_Vector3*)&speed;
	}

	inline hk_real get_mass(){
		return IVP_Core::get_mass();
	}

	inline hk_real get_inv_mass(){
		return IVP_Core::get_inv_mass();
	}

	hk_Diagonal_Matrix &_get_inv_body_inertia(){
		return *(hk_Diagonal_Matrix *)get_inv_rot_inertia();
	}

	hk_Diagonal_Matrix &_get_body_inertia(){
		return *(hk_Diagonal_Matrix *)get_rot_inertia();
	}

	void add_to_mass_matrix_inv(	hk_Core_VMQ_Input &input,	hk_Dense_Matrix& matrix_out,	hk_real velocities_out[]);

	inline void add_to_velocities(      hk_Core_VMQ_Input &input, hk_real velocities_out[])
	{
		int i = input.m_n_queries - 1;

		hk_Virtual_Mass_Query *mq_a = input.m_vmq;
		hk_real *vel = &velocities_out[0];

		do {
			hk_real spin_projected_vel =   mq_a->m_angular.dot( _get_spin() );
			hk_real linear_projected_vel = mq_a->m_linear.dot(_get_linear_velocity() );
			int i_dest_index = mq_a->m_matrix_index;

			vel[i_dest_index] +=  spin_projected_vel + linear_projected_vel; 

			mq_a ++;
		} while ( --i >= 0 );
	}

	void apply_impulses( hk_Core_VMQ_Input &input,		const hk_real impulse_strength[]);

};




typedef hk_Rigid_Body hk_Entity;

class hk_Rigid_Body_Binary_EF {
protected:
	hk_Rigid_Body *m_entities[2];
	hk_Environment *m_environment;

protected:
	hk_Rigid_Body_Binary_EF( hk_Environment *, hk_Rigid_Body *, hk_Rigid_Body *, hk_effector_priority );
	~hk_Rigid_Body_Binary_EF(){

	}
	
public:
	virtual void get_effected_entities(hk_Array<hk_Entity*> &ent_out)
	{
		ent_out.add_element( m_entities[0] );
		ent_out.add_element( m_entities[1] );
	}

	inline hk_Environment *get_environment();
	inline hk_Rigid_Body	  *get_rigid_body(int i){
		return m_entities[i];
	}	
protected:
};



// this is the ipion interface for the local_constraint_system
class hk_Link_EF : public IVP_Controller_Dependent
{
    friend class IVP_Actuator_Manager;
protected:
	hk_Environment *m_environment;
public:
    //Controller Section
    IVP_U_Vector<IVP_Core> actuator_controlled_cores;
    IVP_U_Vector<IVP_Core> *get_associated_controlled_cores() { return &actuator_controlled_cores; };
    IVP_CONTROLLER_PRIORITY get_controller_priority() { return IVP_CP_CONSTRAINTS_MIN; };

	virtual void apply_effector_PSI(	class hk_PSI_Info& pi, hk_Array<hk_Entity*>* ) = 0;

	//lwss- remove extra qualifier
	//void hk_Link_EF::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> * /*core_list*/)
	void do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> * /*core_list*/)
	{
		apply_effector_PSI( *(hk_PSI_Info *)(es), NULL );
	}

    hk_Link_EF(hk_Environment *env)
	{
			m_environment = env;
	}

    virtual void anchor_will_be_deleted_event(IVP_Anchor *del_anchor){ // when an object is deleted it sends events to its connected actuators
		delete this;
	}

    virtual void core_is_going_to_be_deleted_event(IVP_Core *my_core){
		delete this;
	}

    virtual ~hk_Link_EF(){
		;
	}

	inline hk_Environment *get_environment() const
	{
		return m_environment;
	}
};
class hk_Limited_Ball_Socket_BP *ivp_create_limited_ball_socket_bp(IVP_Template_Constraint *tmpl );
class hk_Constraint *ivp_create_ragdoll_constraint_from_local_constraint_system(class hk_Local_Constraint_System *, IVP_Template_Constraint * );


#include <hk_physics/constraint/constraint.h>
#include <hk_physics/core/vm_query.h>

#endif /*IVP_PHYSICS_PHYSICS_H*/

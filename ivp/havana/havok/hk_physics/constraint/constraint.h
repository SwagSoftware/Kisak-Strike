#ifndef HK_PHYSICS_CONSTRAINT_H
#define HK_PHYSICS_CONSTRAINT_H

// IVP_EXPORT_PUBLIC

class hk_PSI_Info;
class hk_Local_Constraint_System;

class hk_Constraint : public hk_Rigid_Body_Binary_EF
{

	hk_Local_Constraint_System* m_constraint_system;
	void					*m_client_data;

public:

	inline hk_Constraint(hk_Environment *env, hk_Rigid_Body *a, hk_Rigid_Body *b, hk_effector_priority redundend_prio)
		:	hk_Rigid_Body_Binary_EF( env, a, b, redundend_prio),
			m_constraint_system (HK_NULL)
	{
	}

	hk_Constraint(hk_Local_Constraint_System *sys, hk_Rigid_Body *a, hk_Rigid_Body *b, hk_effector_priority redundend_prio, int redundant_storage_size);

	virtual ~hk_Constraint();


	virtual int get_vmq_storage_size() = 0;

	virtual int	setup_and_step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor ) = 0;
		//: uses the mem as a vmq storage, returns the bytes needed to store its vmq_storage

	virtual void step_constraint( hk_PSI_Info& pi, void *mem, hk_real tau_factor, hk_real damp_factor ) = 0;
		//: use the mem as a vmq storage setup before

	inline hk_Local_Constraint_System *get_constraint_system() // returns the constraint system, if the constraint is used in a system mode
	{
		return m_constraint_system;
	}

	inline void constraint_system_deleted_event( hk_Local_Constraint_System *pSystem )
	{
		if ( m_constraint_system == pSystem )
			m_constraint_system = NULL;
	}
	inline void set_client_data( void *client_data ) { m_client_data = client_data; }
	inline void *get_client_data() const { return m_client_data; }

	virtual void init_constraint(const void /*blueprint*/ *) = 0;
		//: Set the constraint parameters from the blueprrint
};

#endif /* HK_PHYSICS_CONSTRAINT_H */


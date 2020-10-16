
#include <hk_physics/physics.h>
#include <hk_physics/simunit/psi_info.h>
#include <hk_physics/constraint/constraint.h>
#include <hk_physics/constraint/local_constraint_system/local_constraint_system.h>

// IVP_EXPORT_PUBLIC

hk_Constraint::hk_Constraint(hk_Local_Constraint_System *sys, hk_Rigid_Body *a, hk_Rigid_Body *b, hk_effector_priority redundend_prio, int storage_size)
	: hk_Rigid_Body_Binary_EF( HK_NULL, a, b, HK_PRIORITY_NONE),	// do not link yet
		m_constraint_system (sys)
{ 
	m_constraint_system->add_constraint(this, storage_size);
}


hk_Constraint::~hk_Constraint()
{ 
	if (m_constraint_system)
	{
		m_constraint_system->constraint_deletion_event(this);
	}
}

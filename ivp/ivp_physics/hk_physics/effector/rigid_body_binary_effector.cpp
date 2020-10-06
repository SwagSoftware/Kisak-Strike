// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <hk_physics/physics.h>

hk_Rigid_Body_Binary_EF::hk_Rigid_Body_Binary_EF( hk_Environment *env, hk_Rigid_Body *a, hk_Rigid_Body *b, hk_effector_priority )
{
		m_entities[0] = a;
		m_entities[1] = b;
		m_environment = env;
}

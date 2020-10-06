// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <ivp_mindist_intern.hxx>
#include <ivp_friction.hxx>
#include <ivp_debug_manager.hxx>
#include <ivp_i_friction_hash.hxx>

IVP_Friction_Hash::~IVP_Friction_Hash() {
    IVP_ASSERT( this->n_elems() == 0 );
};

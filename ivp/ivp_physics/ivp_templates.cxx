// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <string.h>
#include <ivp_templates.hxx>
#include <ivp_cache_object.hxx>

IVP_Template_Object::IVP_Template_Object()
{
    P_MEM_CLEAR(this);
    this->name = NULL;
    return;
}

IVP_Template_Object::~IVP_Template_Object()
{
    P_FREE(this->name);
    return;
}

void IVP_Template_Object::set_name(const char *s)
{
    P_FREE(this->name);
    this->name = p_strdup(s);
    return;
}

/////////////////////

IVP_Template_Real_Object::IVP_Template_Real_Object()
{
    P_MEM_CLEAR(this);
    this->mass = 1.0f; // default;
    this->rot_inertia_is_factor = IVP_TRUE;
    this->rot_inertia.set(1.0f, 1.0f, 1.0f);
    this->auto_check_rot_inertia = 0.03f;
    this->speed_damp_factor = 0.01f;
    this->rot_speed_damp_factor.set( 0.01f, 0.01f, 0.01f);
	this->pinned = IVP_FALSE;
    return;
}

IVP_Template_Real_Object::~IVP_Template_Real_Object()
{
    return;
}


void IVP_Template_Real_Object::set_nocoll_group_ident(const char *id){
    if (!id){
	nocoll_group_ident[0] = 0;
	return;
    }
    if (strlen(id) > IVP_NO_COLL_GROUP_STRING_LEN){
	CORE;
    }
    strncpy( nocoll_group_ident, id, IVP_NO_COLL_GROUP_STRING_LEN);
}




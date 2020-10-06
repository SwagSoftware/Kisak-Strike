// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#if defined(LINUX)
#	include <string.h>
#endif

#include "ivp_object_polygon_tetra.hxx"

#ifndef WIN32
#	pragma implementation "ivp_i_point_vhash.hxx"
#endif

#include <ivp_i_point_vhash.hxx>


IVP_point_hash::~IVP_point_hash(){;}

int IVP_point_hash::point_to_index(IVP_Poly_Point *point)
{
    // ATTENTION: assumes that doubles are taken for IVP_Poly_Point coords
    return hash_index( (char *)&point->k[0], sizeof(IVP_DOUBLE)*3);
}

IVP_BOOL IVP_point_hash::compare(void *elem0, void *elem1) const {
    if( memcmp(elem0, elem1, sizeof(IVP_DOUBLE)*3) == 0){
	return IVP_TRUE;
    }
    return IVP_FALSE;
}


IVP_I_Point_VHash::~IVP_I_Point_VHash(){;}

int IVP_I_Point_VHash::point_to_index(IVP_U_Point *point)
{
    // ATTENTION: assumes that doubles are taken for IVP_Poly_Point coords
    return hash_index( (char *)&point->k[0], sizeof(IVP_DOUBLE)*3);
}

IVP_BOOL IVP_I_Point_VHash::compare(void *elem0, void *elem1) const {
    if( memcmp(elem0, elem1, sizeof(IVP_DOUBLE)*3) == 0){
	return IVP_TRUE;
    }
    return IVP_FALSE;
}






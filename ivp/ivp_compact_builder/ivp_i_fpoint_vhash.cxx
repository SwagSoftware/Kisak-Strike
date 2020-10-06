// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#if defined(LINUX)
#	include <string.h>
#endif

#include <ivu_linear.hxx>

#ifndef WIN32
#	pragma implementation "ivp_i_fpoint_vhash.hxx"
#endif

#include <ivp_i_fpoint_vhash.hxx>


IVP_I_FPoint_VHash::~IVP_I_FPoint_VHash(){;}

int IVP_I_FPoint_VHash::point_to_index(IVP_U_Float_Point *point)
{
    // ATTENTION: assumes that doubles are taken for IVP_Poly_Point coords
    return hash_index( (char *)&point->k[0], sizeof(IVP_FLOAT)*3);
}

IVP_BOOL IVP_I_FPoint_VHash::compare(void *elem0, void *elem1) const {
    if( memcmp(elem0, elem1, sizeof(IVP_FLOAT)*3) == 0){
	return IVP_TRUE;
    }
    return IVP_FALSE;
}






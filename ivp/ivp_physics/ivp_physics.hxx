// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

/********************************************************************************
 *	File:	       	ivp_physics.hxx	
 *	Description:	The main include file for the physics API.
 *	Attention:	This file should be included prior to other 'ivp*.hxx' files.
 ********************************************************************************/

#ifndef _IVP_PHYSICS_INCLUDED
#define _IVP_PHYSICS_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

#define IVP_MAX_DELTA_PSI_TIME (1.0f/10.0f)
#define IVP_MIN_DELTA_PSI_TIME (1.0f/200.0f)

#define IVP_BLOCKING_EVERY_MIN

#include <stdio.h>

#include <math.h>
#if defined(LINUX)
#	include <string.h>
#endif

#ifndef _IVP_U_TYPES_INCLUDED
#	include "ivu_types.hxx"
#endif


#ifndef _IVP_VECTOR_INCLUDED
#	include "ivu_vector.hxx"
#endif


#ifndef _IVP_BIGVECTOR_INCLUDED
#	include "ivu_bigvector.hxx"
#endif


#ifndef _IVP_U_LINEAR_INCLUDED
#	include <ivu_linear.hxx>
#endif


#ifndef _IVP_U_LINEAR_MACROS_INCLUDED
#	include <ivu_linear_macros.hxx>
#endif


#ifndef _IVP_SURFACE_MANAGER_INCLUDED
#	include <ivp_surface_manager.hxx>
#endif


#ifndef _IVP_OBJECT_INCLUDED
#	include <ivp_object.hxx>
#endif


#ifndef _IVP_REAL_OBJECT_INCLUDED
#	include <ivp_real_object.hxx>
#endif


#ifndef _IVP_BALL_INCLUDED
#	include <ivp_ball.hxx>
#endif


#ifndef _IVP_POLYGON_INCLUDED
#	include <ivp_polygon.hxx>
#endif


#ifndef _IVP_ENVIRONMENT_INCLUDED
#	include <ivp_environment.hxx>
#endif

#ifndef _IVP_CORE_INCLUDED
#	include <ivp_core.hxx>
#endif


#include <ivu_string.hxx>

#define IVP_NO_MD_INTERPOLATION 

#endif


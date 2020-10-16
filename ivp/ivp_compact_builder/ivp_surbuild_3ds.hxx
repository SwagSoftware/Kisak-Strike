// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_surbuild_polyhedron_concave.hxx
 *  Description:    This file provides you with a builder class for easy
 *		    easy generation of (concave) compact surfaces from concave
 *		    object data.
 *  Classes:	    IVP_SurfaceBuilder_Polyhedron_Concave
 ********************************************************************************/

#ifndef IVP_SURBUILD_3DS_INCLUDED
#define IVP_SURBUILD_3DS_INCLUDED

class IVP_Compact_Surface;
class IVP_Compact_Ledge;
class IVP_Concave_Polyhedron;

class IVP_Template_SurfaceBuilder_3ds {
public:
    IVP_FLOAT scale;    
    IVP_Template_SurfaceBuilder_3ds();
};

class IVP_SurfaceBuilder_3ds {
public:
    static IVP_Concave_Polyhedron* convert_3ds_to_concave( const char *filename, IVP_Template_SurfaceBuilder_3ds *);
};


#endif

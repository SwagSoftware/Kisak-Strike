// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_surbuild_polyhedron_concave.hxx
 *  Description:    This file provides you with a builder class for easy
 *		    easy generation of (concave) compact surfaces from concave
 *		    object data.
 *  Classes:	    IVP_SurfaceBuilder_Polyhedron_Concave
 ********************************************************************************/

#ifndef _IVP_SURBUILD_POLYHEDRON_CONCAVE_INCLUDED
#define _IVP_SURBUILD_POLYHEDRON_CONCAVE_INCLUDED


class IVP_Compact_Surface;


/********************************************************************************
 *  Class:	    IVP_SurfaceBuilder_Polyhedron_Concave
 *  Description:    This builder class allows for easy generation of (concave)
 *		    compact surfaces from concave object data.
 *******************************************************************************/

class IVP_SurfaceBuilder_Polyhedron_Concave {
public:

    /******************************************************************************
     *  Method:		convert_concave_polyhedron_to_compact_ledges
     *  Description:    This method will convert the supplied concave object into
     *			a set of (convex) compact ledges.
     *	Input:		<concave_polyhedron_in>  data on the concave object
     *		        <params>                 some user-definable parameters
     *			<ledges_out>             address of vector to fill with
     *			                         the resulting compact ledges
     *	Output:		Number of compact ledges.
     *****************************************************************************/
    static int convert_concave_polyhedron_to_compact_ledges(IVP_Concave_Polyhedron *concave_polyhedron_in,
							    IVP_Convex_Decompositor_Parameters *params,
							    IVP_U_BigVector<IVP_Compact_Ledge> *ledges_out);

    /******************************************************************************
     *  Method:		convert_concave_face_soup_to_compact_ledges
     *  Description:    This method will convert the supplied concave object into
     *			a set of (convex) compact ledges, where each face is
     *			treated as a convex ledge.
     *	Input:		<concave_polyhedron_in>  data on the concave object
     *			<ledges_out>             address of vector to fill with
     *			                         the resulting compact ledges
     *****************************************************************************/
    static void convert_concave_face_soup_to_compact_ledges( IVP_Concave_Polyhedron *concave_polyhedron_in, IVP_U_BigVector<IVP_Compact_Ledge> *ledges_out);
    
    /******************************************************************************
     *  Method:		convert_concave_polyhedron_to_single_compact_surface
     *  Description:    This method will compile a (concave) compact surface from
     *			the supplied concave object data.
     *	Input:		<concave_polyhedron_in>  data on the concave object
     *		        <params>                 some user-definable parameters
     *	Output:		Pointer to IVP_Compact_Surface structure.
     *  Note:           Don't use this function, use convert_concave_face_soup_to_compact_ledges and
     *			
     *****************************************************************************/
    static IVP_Compact_Surface *convert_concave_polyhedron_to_single_compact_surface(IVP_Concave_Polyhedron *concave_polyhedron_in,
										     IVP_Convex_Decompositor_Parameters *params);
    
};

#endif

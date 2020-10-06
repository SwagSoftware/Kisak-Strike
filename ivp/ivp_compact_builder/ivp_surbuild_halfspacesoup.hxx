// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_surbuild_halfspacesoup.hxx
 *  Description:    This file provides you with a builder class for easy
 *		    conversion of halfspace planes into several different forms
 *		    for further use within the Ipion engine (e.g. for creating
 *		    objects).
 *  Classes:	    IVP_SurfaceBuilder_Halfspacesoup
 ********************************************************************************/

#ifndef _IVP_SURBUILD_HALFSPACESOUP_INCLUDED
#define _IVP_SURBUILD_HALFSPACESOUP_INCLUDED


/********************************************************************************
 *  Class:	    IVP_SurfaceBuilder_Halfspacesoup
 *  Description:    Ipion's halfspacesoup builder calculates the intersections
 *		    between all supplied halfspace planes while dropping from the
 *		    resulting set of points any vertices outside of the enclosed
 *		    volume. It then will return the convex set of points or,
 *		    alternatively, further convert it to a compact ledge or
 *		    compact surface
 *		    (by internally using the IVP_SurfaceBuilder_Pointsoup class).
 *  Note:	    Points that are closer to each other than a certain threshold
 *		    will be merged to avoid numerical problems.
 *  Note:	    The resulting set of points might well be empty (depending,
 *		    of course, on the input data)!
 *  Note:	    Be careful not to provide a set of halfspace planes that
 *		    defines an infinite volume!
 *******************************************************************************/

class IVP_SurfaceBuilder_Halfspacesoup {
    friend class IVP_SurfaceBuilder_Q12;

private:
    // internal methods
    static IVP_U_Point *insert_point_into_list(IVP_U_Point *point, IVP_U_Vector<IVP_U_Point> *points, IVP_DOUBLE quad_threshold);
	
public:

    /******************************************************************************
     *  Method:		convert_halfspacesoup_to_points
     *  Description:    This function will convert the supplied set of halfspace
     *			planes into the set of outer points
     *	Input:		<halfspaces>           set of halfspace planes
     *			<pointmerge_threshold> merge all points that are closer
     *			                       to each other than this distance
     *			<points>               address of vector to fill with
     *			                       resulting points
     *	Output:		Number of resulting points
     *****************************************************************************/
    static int convert_halfspacesoup_to_points(IVP_Halfspacesoup *halfspaces,
					       IVP_DOUBLE pointmerge_threshold,
					       IVP_U_Vector<IVP_U_Point> *points);

    /******************************************************************************
     *  Method:		convert_halfspacesoup_to_compact_ledge
     *  Description:    This method will build a IVP_Compact_Ledge from the
     *			supplied set of halfspace planes
     *	Input:		<halfspaces>           set of halfspace planes
     *			<pointmerge_threshold> merge all points that are closer
     *			                       to each other than this distance
     *	Output:		Pointer to IVP_Compact_Ledge structure
     *****************************************************************************/
    static IVP_Compact_Ledge *convert_halfspacesoup_to_compact_ledge(IVP_Halfspacesoup *halfspaces,
								     IVP_DOUBLE pointmerge_threshold);

    /******************************************************************************
     *  Method:		convert_halfspacesoup_to_compact_surface
     *  Description:    This method will build a IVP_Compact_Surface from the
     *			supplied set of halfspace planes
     *	Input:		<halfspaces>           set of halfspace planes
     *			<pointmerge_threshold> merge all points that are closer
     *			                       to each other than this distance
     *	Output:		Pointer to IVP_Compact_Surface structure
     *****************************************************************************/
    static IVP_Compact_Surface *convert_halfspacesoup_to_compact_surface(IVP_Halfspacesoup *halfspaces,
									 IVP_DOUBLE pointmerge_threshold);

};

#endif

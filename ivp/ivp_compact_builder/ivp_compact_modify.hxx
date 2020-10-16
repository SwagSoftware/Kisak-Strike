// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_compact_modify.hxx
 *  Description:    This file provides you with some useful classes and methods
 *		    to modify any compact surface after it has been created.
 *  Classes:	    IVP_Compact_Modify
 ********************************************************************************/

#ifndef _IVP_COMPACT_MODIFY_INCLUDED
#define _IVP_COMPACT_MODIFY_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

/********************************************************************************
 *  Name:	    IVP_Compact_Modify	
 *  Description:    A set of static methods to easily modify a compact surface
 *		    (i.e. the shape of an object)
 ********************************************************************************/

class IVP_Compact_Modify {
public:

    /******************************************************************************
     *  Method:		chop
     *  Description:    Chop off a slice of an object
     *	Input:		<c_surface_in> a given compact surface
     *			<chop_vector>  halfspace plane defining the orientation of
     *			               the cut
     *			<chop_depth>   the 'thickness' of the slice to be chopped
     *			               off (i.e. the distance between the slicing
     *			               plane and the furthest chopped-off point
     *			               orthogonal to this plane)
     *	Output:		Pointer to the new compact surface.
     *	Note:		The halfspace plane's normal vector has to point AT the
     *			object.
     *	Note:		For now this method only supports chopping of 'one-ledged'
     *			compact surfaces (i.e. convex objects)!
     *	Warning:	Function may fail due to numerical errors (unlikely but
     *			possible)
     *	Example:	If you have a pole (pointing in x-direction) and you want
     *			to slice off 1 cm, you should call:
     *			"chop(pole_cs, IVP_U_Float_Point(1.0 ,0.0, 0.0), 0.01);"
     *****************************************************************************/
    static IVP_Compact_Surface *chop(const IVP_Compact_Surface *c_surface_in,
				     const IVP_U_Float_Point *chop_vector,
				     IVP_FLOAT chop_depth);

    /******************************************************************************
     *  Method:		shrink
     *  Description:    Reduce the size of the given ledge by shifting each of its
     *			planes towards the geometrical center of the object
     *	Input:		<ledge_in>     compact ledge to shrink
     *			<shrink_value> value to shrink ledge in all directions
     *			               (unit: m)
     *			<pointmerge_threshold> merge all points that are closer
     *			                       to each other than this distance
     *	Output:		Pointer to the new compact ledge.
     *	Warning:	Function may fail due to numerical errors (unlikely but
     *			possible)
     *****************************************************************************/
    static IVP_Compact_Ledge *shrink(const IVP_Compact_Ledge *ledge_in,
				     IVP_FLOAT shrink_value,
				     IVP_DOUBLE pointmerge_threshold);

    /******************************************************************************
     *  Method:		shrink
     *  Description:    Reduce the size of the given compact surface by shrinking
     *			all of its compact ledges.
     *	Input:		<c_surface_in> compact surface to shrink
     *			<shrink_value> value to shrink object in all directions
     *			               (unit: m)
     *			<pointmerge_threshold> merge all points that are closer
     *			                       to each other than this distance
     *	Output:		Pointer to the new compact surface.
     *  Note:		If your object consists of more than one ledge, the gap
     *			between two ledges will increase by 2*shrink_value
     *	Warning:	Function may fail due to numerical errors (unlikely but
     *			possible)
     *****************************************************************************/
    static IVP_Compact_Surface *shrink(const IVP_Compact_Surface *c_surface_in,
				       IVP_FLOAT shrink_value,
				       IVP_DOUBLE pointmerge_threshold);
};

#endif

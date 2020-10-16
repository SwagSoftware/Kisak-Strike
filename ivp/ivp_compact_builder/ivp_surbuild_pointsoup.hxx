// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_surbuild_pointsoup.hxx
 *  Description:    This file provides you with a builder class for easy
 *		    calculation of a pointsoup's convex hull (i.e. the minimum
 *		    geometrical shape to contain all given points). Additionally
 *		    the class will transform this convex hull into either a
 *		    (convex) compact ledge or a (convex) compact surface for
 *		    further processing by the Ipion engine.
 *  Additional
 *  Copyright
 *  Notices:	    Qhull; see below
 *  Classes:	    IVP_SurfaceBuilder_Pointsoup
 *		    IVP_SurMan_PS_Plane (internal class only!)
 ********************************************************************************/

// -------------------------------------------------------------
// Ipion Virtual Physics Engine utilizes:
//
//                    Qhull, Copyright (c) 1993-1999
//
//       The National Science and Technology Research Center for
//        Computation and Visualization of Geometric Structures
//                        (The Geometry Center)
//                       University of Minnesota
//                            400 Lind Hall
//                        207 Church Street S.E.
//                      Minneapolis, MN 55455  USA
//
//                       email: qhull@geom.umn.edu
//
// Qhull is copyrighted as noted above.  Qhull is free software
// and may be obtained via anonymous ftp from geom.umn.edu.
//
// See also 'qhull.readme'
// -------------------------------------------------------------

#ifndef _IVP_SURBUILD_POINTSOUP_INCLUDED
#define _IVP_SURBUILD_POINTSOUP_INCLUDED

class IVP_Template_Point;
class IVP_Template_Line;
class IVP_Template_Polygon;
class IVP_Compact_Surface;


/********************************************************************************
 *  Class:	    IVP_SurMan_PS_Plane
 *  Description:    an INTERNAL structure
 *		    Defines a plane by supplying a set of points and the plane's
 *		    normal vector.
 *  Note:	    INTERNAL CLASS! There should be no need for you to use this
 *		    class as it is only used by the ledgesoup- and pointsoup-
 *		    builder classes!
 *******************************************************************************/

class IVP_SurMan_PS_Plane : public IVP_U_Point {
public:
    IVP_U_Vector<IVP_U_Point> points;
  IVP_DOUBLE get_area_size();
  IVP_DOUBLE get_qlen_of_all_edges();
};


// a vector of 256 points, automatically increases if overflow
// if less than 256 elements are inserted, no malloc/free is called
class IVP_Vector_of_Points_256: public IVP_U_Vector<IVP_U_Point> {
    IVP_Core *elem_buffer[256];
public:
    IVP_Vector_of_Points_256(): IVP_U_Vector<IVP_U_Point>( (void **)&elem_buffer[0],256 ){;};
};


/********************************************************************************
 *  Class:	    IVP_SurfaceBuilder_Pointsoup
 *  Description:    Ipion's pointsoup builder calculates the convex hull of any
 *		    supplied set of points and returns either a single compact
 *		    ledge or an already compiled compact surface.
 *  Note:	    Ipion uses Qhull to calculate the convex hull.
 *		    Qhull used with kind permission from 'The Geometry Center'.
 *******************************************************************************/

class IVP_SurfaceBuilder_Pointsoup {
    friend class IVP_SurfaceBuilder_Ledge_Soup;
    friend class IVP_SurfaceBuilder_Halfspacesoup;
    friend class IVP_SurfaceBuilder_Q12;

protected:
    static IVP_Compact_Ledge *single_tri_ledge;
    // internal methods
    static int get_offset_from_pointlist(IVP_Template_Point *points, int length, IVP_U_Point *point);
    static int get_offset_from_lineslist(IVP_Template_Line *lines, int length, int pointnr1, int pointnr2, char *reverse);

    static IVP_Template_Polygon *planes_to_template(IVP_U_Vector<IVP_U_Point> *points, IVP_U_Vector<IVP_SurMan_PS_Plane> *planes);
    static void                  error_output(IVP_Template_Polygon *templ);
    static IVP_Compact_Ledge    *convert_pointsoup_to_compact_ledge_internal(IVP_U_Vector<IVP_U_Point> *points);
    static IVP_Compact_Ledge *try_to_build_convex_ledge_from_qhull_result(IVP_U_Vector<IVP_U_Point> *points, IVP_BOOL *skip_point, char *skip_list, char *use_list);
public:
    static void cleanup();
    
    static IVP_Compact_Ledge *convert_triangle_to_compace_ledge( IVP_U_Point *p0, IVP_U_Point *p1, IVP_U_Point *p2);

    /******************************************************************************
     *  Method:		convert_pointsoup_to_compact_ledge
     *  Description:    This method will build the convex hull for the supplied
     *			set of points, drop all interior points and create a
     *			IVP_Compact_Ledge structure
     *	Input:		<points> set of points
     *	Output:		Pointer to IVP_Compact_Ledge structure
     *	Note:		Usually the input data has to be 3-dimensional! You are
     *			allowed to create a 2-dimensional triangle by supplying
     *			EXACTLY 3 points, though!
     *****************************************************************************/
    static IVP_Compact_Ledge * IVP_CDECL convert_pointsoup_to_compact_ledge(IVP_U_Vector<IVP_U_Point> *points);

    /******************************************************************************
     *  Method:		convert_pointsoup_to_compact_surface
     *  Description:    This method will build the convex hull for the supplied
     *			set of points, drop all interior points and create a
     *			IVP_Compact_Surface structure
     *	Input:		<points> set of points
     *	Output:		Pointer to IVP_Compact_Surface structure
     *	Note:		Usually the input data has to be 3-dimensional! You are
     *			allowed to create a 2-dimensional triangle by supplying
     *			EXACTLY 3 points, though!
     *****************************************************************************/
    static IVP_Compact_Surface * IVP_CDECL convert_pointsoup_to_compact_surface(IVP_U_Vector<IVP_U_Point> *points);

};

#endif

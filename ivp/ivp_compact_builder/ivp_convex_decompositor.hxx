// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_convex_decompositor.hxx
 *  Description:    This file provides you with a simple tool to split a concave
 *		    object into convex subparts.
 *  Additional
 *  Copyright
 *  Notices:	    GEOMPACK; see below
 *  Classes:	    IVP_Convex_Decompositor
 ********************************************************************************/

// -------------------------------------------------------------
// Ipion Virtual Physics Engine utilizes:
//
// GEOMPACK Version 3 (Fortran77)
//
// Written and generously provided free of charge by
//
//   Dr. Barry Joe
//   Email: bjoe@netcom.ca
//
// -------------------------------------------------------------

#ifndef _IVP_CONVEX_DECOMPOSITOR_INCLUDED
#define _IVP_CONVEX_DECOMPOSITOR_INCLUDED


/********************************************************************************
 *  Class:	    IVP_Concave_Polyhedron_Face_Pointoffset
 *  Description:    Offset into concave polyhedron's pointlist.
 *  Note:	    Just a little helping class. :-)
 *******************************************************************************/

class IVP_Concave_Polyhedron_Face_Pointoffset {
public:
    int	offset;
};


/********************************************************************************
 *  Class:	    IVP_Concave_Polyhedron_Face
 *  Description:    One single face of a concave polyhedron.
 *  Note:	    This class only stores the OFFSETS of the face's points into
 *		    the polyhedron's pointlist (see IVP_Concave_Polyhedron)!
 *  Note:	    The order in which the offsets are supplied is VERY
 *		    important!
 *		    Always supply them counter-clockwise, i.e. look at the front
 *		    side of the face (which is usually the side pointing away
 *		    from the object), pick one point and then traverse the
 *		    remaining points in counter-clockwise direction.
 *******************************************************************************/

class IVP_Concave_Polyhedron_Face {
public:
    IVP_U_Vector<IVP_Concave_Polyhedron_Face_Pointoffset> point_offset;

    /******************************************************************************
     *  Method:		add_offset
     *  Description:    Use this method to add a new point offset to the list.
     *	Note:		This method will drop any duplicate offsets.
     *****************************************************************************/
    void add_offset(int offset);

    ~IVP_Concave_Polyhedron_Face();
};


/********************************************************************************
 *  Class:	    IVP_Concave_Polyhedron
 *  Description:    Simple parameter class for supplying the data of a concave
 *		    polyhedron.
 *  Note:	    The 'points' vector stores ALL the polyhedron's points, the
 *		    'faces' vector stores ALL the polyhedron's faces!
 *******************************************************************************/

class IVP_Concave_Polyhedron {
public:
    IVP_U_BigVector<IVP_U_Point>			points;
    IVP_U_BigVector<IVP_Concave_Polyhedron_Face>	faces;
};


/********************************************************************************
 *  Class:	    IVP_Convex_Subpart
 *  Description:    Simple parameter class for one single convex subpart. Vector
 *		    'points' will get filled with the subparts pointsoup.
 *******************************************************************************/

class IVP_Convex_Subpart {
public:
    IVP_U_Vector<IVP_U_Point> points;

    ~IVP_Convex_Subpart();
};


/********************************************************************************
 *  Class:	    IVP_Convex_Decompositor_Parameters
 *  Description:    User-definable parameters for convex decomposition.
 *******************************************************************************/

class IVP_Convex_Decompositor_Parameters {
public:
    float tolin; // tolerance in
//    float aspc2d;
//    float atol2d;
    float angacc;
    float rdacc;
};


/********************************************************************************
 *  Class:	    IVP_Convex_Decompositor
 *  Description:    This class allows for easy transforming of complex concave
 *		    objects into simple convex subparts.
 *******************************************************************************/

class IVP_Convex_Decompositor {
public:

    /******************************************************************************
     *  Method:		perform_convex_decomposition_on_concave_polyhedron
     *  Description:    This method will split a supplied concave polyhedron into
     *			several convex subparts. It will return a pointsoup for
     *			each created convex subpart.
     *	Input:		<concave_polyhedron_in>  data of the concave polyhedron
     *			<params>                 some user-definable parameters
     *			<convex_subparts_out>    vector that will be filled with
     *			                         pointers to IVP_Convex_Subpart
     *			                         objects
     *	Output:		Number of created subparts
     *****************************************************************************/
    static int perform_convex_decomposition_on_concave_polyhedron(IVP_Concave_Polyhedron *concave_polyhedron_in,
								  IVP_Convex_Decompositor_Parameters *params,
								  IVP_U_BigVector<IVP_Convex_Subpart> *convex_subparts_out);

};

#endif

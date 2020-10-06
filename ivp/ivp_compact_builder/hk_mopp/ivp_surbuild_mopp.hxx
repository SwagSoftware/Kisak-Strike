// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_surbuild_Mopp.hxx
 *  Description:    This file provides you with a builder class for assembling
 *		    different (convex) compact ledges into one (possibly concave)
 *		    compact surface which you can then use to create a physical
 *		    object. It is much like the IVP_SurfaceBuilder_Ledge_Soup class, but
 *          creates a compact surface with a different internal ledgetree 
 *			structure.
 *  Classes:	    IVP_SurfaceBuilder_Mopp
 ********************************************************************************/

#ifndef IVP_SURBUILD_MOPP_INCLUDED
#define IVP_SURBUILD_MOPP_INCLUDED

class IVP_Compact_Ledge;
class IVP_Compact_Mopp;
class IVP_Compact_Poly_Point;
class IVP_Template_Surbuild_Ledgesoup;
class IVP_I_FPoint_VHash;
class hkMoppCode; // bv - tree

/********************************************************************************
 *  Class:	    IVP_SurfaceBuilder_Ledge_Soup
 *  Description:    Ipion's ledgesoup builder takes one or more (convex) compact
 *		    ledges as input, combines them and builds a (possibly
 *		    concave) compact surface. You can then use this compact
 *		    surface to create a physical object.
 *  Note:	    Algorithm is fast but still O(n*sqrt(n));
 *******************************************************************************/
class IVP_SurfaceBuilder_Mopp {

private:
	// internal methods
	IVP_Compact_Ledge* convex_hull;
	IVP_Compact_Mopp* compact_mopp;
//	IVP_DOUBLE smallest_radius;
//	int size_of_tree_in_bytes; // benchmarking
	IVP_U_Vector<IVP_Compact_Ledge> c_ledge_vec;	// including inner recursive compact ledges

	IVP_Template_Surbuild_LedgeSoup* parameters;

	// compact surface creation
	IVP_Compact_Ledge* first_compact_ledge;
	IVP_Compact_Poly_Point* first_poly_point;
	int n_poly_points_allocated;

	IVP_I_FPoint_VHash* point_hash;

//	hkMoppCode* ledgetree_work;

//	char *clt_highmem; // debugging
//	char *clt_lowmem; // debugging

protected:

//	void ledges_to_boxes_and_spheres();

	int recompile_point_indizes_of_compact_ledge(IVP_Compact_Ledge *ledge_source, char *dest);

	void insert_compact_ledges();

	void insert_radius_in_compact_mopp();

//	void cleanup();

	void build_root_convex_hull();

	// finally call both functions to fill ledge
	IVP_Compact_Mopp* allocate_compact_mopp();

	hkMoppCode* create_compact_ledgetree();

public:

	/******************************************************************************
	 *  Method:		insert_ledge
	 *  Description:    Use this method to add additional ledges (i.e. convex
	 *			subparts of your object) to the ledgesoup builder
	 *	Input:		<ledge> new ledge to add to the soup
	 *	Output:		Pointer to the original (supplied) IVP_Compact_Ledge
	 *			(compatibility reason)
	 *****************************************************************************/
	void insert_ledge(IVP_Compact_Ledge* ledge);

	/******************************************************************************
	 *  Method:		compile
	 *  Description:    This method will actually build the new (possibly concave)
	 *			compact surface out of the (convex) ledges
	 *  build_convex_hull if set to true, the algorithm generates an additional
	 *			convex hull around the object. For moveable objects this
	 *			hull can increase performance to the same level as convex
	 *			objects.
	 *			Only for huge landscapes this optimization makes no sense at
	 *			all, as all interesting objects are penetrating the convex hull
	 *			all the time.
	 *	Output:		Pointer to the new IVP_Compact_Surface
	 *	Note:		Call this method only after inserting at least one ledge.
	 *			Otherwise this method will return NULL!
	 *	Note:		Algorithm is fast but still O(n*n);
	 *			E.g. 5000 ledges take 50 seconds to compute (on PIII 600)
	 *	Warning:	This method will [optionally] free all original ledges (as supplied
	 *			by the "insert_ledge()" method)! DO NOT FREE THEM
	 *			YOURSELF!
	 *	Note:		Invoking this function a second time (without adding
	 *			new ledges in the meantime) will return NULL.
	 *****************************************************************************/
	IVP_Compact_Mopp* compile(IVP_Template_Surbuild_LedgeSoup* templ = NULL);

	IVP_SurfaceBuilder_Mopp();

	~IVP_SurfaceBuilder_Mopp();
};

#endif

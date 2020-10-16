// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_compact_mopp.hxx
 *  Description:    This file contains the most important class to describe an
 *		    object's geometrical (and thus physical) shape, the COMPACT
 *		    SURFACE. This compact surface (a compact mopp) has an optimized internal
 *			ledge tree.
 *  Classes:	    IVP_Compact_MOPP
 ********************************************************************************/

#ifndef IVP_COMPACT_MOPP_INCLUDED
#define IVP_COMPACT_MOPP_INCLUDED

class hkMoppCode;

#define IVP_COMPACT_SURFACE_DEVIATION_STEP_SIZE (1.0f / 250.0f )

/********************************************************************************
 *  Class:	    IVP_Compact_MOPP
 *  Description:    The compact surface class is the most important class for all
 *		    objects in the Ipion engine as it describes the geometrical
 *		    topology and some additional basic values.
 *  Note:	    This class is just the HEADER for a far more complex (and
 *		    internal) data structure. So DO NOT change any of these
 *		    variables or you will certainly break the engine's neck! :-)
 *  Note:	    This structure has to be 16byte aligned!
 *  Note:	    Do not create a compact surface manually! Instead use one
 *		    of the various builder classes to create a compact surface.
 *  Important:	    Only use "ivp_free_aligned()" to free a compact surface!
 *******************************************************************************/

class IVP_Compact_Mopp {

public:
    IVP_U_Float_Point3 mass_center;

    IVP_U_Float_Point3 rotation_inertia;

    IVP_FLOAT upper_limit_radius;

    unsigned int max_factor_surface_deviation:8;

    int	byte_size:24;		// size of whole structure in bytes

    int	offset_ledgetree_root;	// offset to root node of internal ledgetree

    int offset_ledges;		// offset to the ledges

	int size_convex_hull;

    int	dummy;		// 16byte memory align

    /* Note: to get geom center use root node */
    int get_size() const 
	{ 
		return byte_size; 
    };

    /******************************************************************************
     *  Method:		get_compact_ledge_tree_root
     *  Description:    INTERNAL METHOD
     *****************************************************************************/
    const hkMoppCode* get_compact_ledge_tree_root() const 
	{
        //lwss - x64 fixes
		//char* base = (char*)(((int)this) + this->offset_ledgetree_root);
		char* base = (char*)(((intptr_t)this) + this->offset_ledgetree_root);
		//lwss end
		return((const hkMoppCode*)base);
    }

    // endian conversion support:
    void byte_swap_all(IVP_BOOL swap_points = IVP_TRUE, int point_estimate = 100); 
	// will recurse to all children / related data (once the offsets have been byte swapped of course!)
	// It will also swap all point data found if swap_points is true.

    void byte_swap(); // just do this data, do not recurse to nodes

private: 

    IVP_Compact_Mopp() {;};

};

#endif

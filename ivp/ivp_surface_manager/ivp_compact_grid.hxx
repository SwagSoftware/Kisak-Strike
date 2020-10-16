// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_compact_grid.hxx
 *  Description:    This file contains the most important class to describe a
 *		    grid's geometrical (and thus physical) shape, the COMPACT
 *		    GRID.
 *  Classes:	    IVP_Compact_Grid
 *		    IVP_Compact_Grid_Element
 ********************************************************************************/

#ifndef _IVP_COMPACT_GRID_INCLUDED
#define _IVP_COMPACT_GRID_INCLUDED

#include <cstdint> //lwss - x64 fixes

/********************************************************************************
 *  Class:	    IVP_Compact_Grid_Element
 *  Description:    an INTERNAL structure
 *******************************************************************************/

class IVP_Compact_Grid_Element {
    public:
    short compact_ledge_index[2]; // maximum of two ledges for one square
};


/********************************************************************************
 *  Class:	    IVP_Compact_Grid
 *  Description:    The compact grid class is the most important class for grids
 *		    in the Ipion engine as it describes the geometrical topology
 *		    and some additional basic values.
 *  Note:	    This class is just the HEADER for a far more complex (and
 *		    internal) data structure. So DO NOT change any of these
 *		    variables or you will certainly break the engine's neck! :-)
 *  Note:	    This structure has to be 16bit aligned!
 *  Note:	    Do not create a compact grid manually! Instead use the
 *		    IVP_GridBuilder_Array class to create a compact grid.
 *  Important:	    Only use "ivp_free_aligned()" to free a compact grid!
 *******************************************************************************/

class IVP_Compact_Grid {
public:
    IVP_U_Float_Hesse	center;
    IVP_U_Matrix	m_grid_f_object;	// scales to grid, virtual grid size 1.0
						// x and y are grid axles, z is height
						// center is at 0,0
    short		n_rows;			// number of grid's rows
    short		n_columns;		// number of grid's columns
    int			n_compact_ledges;	// number of ledges (i.e. convex subparts) in grid

    IVP_FLOAT		radius;

    int			byte_size;
    
    IVP_FLOAT		inv_grid_size;

    int offset_grid_elements;
    int offset_compact_ledge_array[1];		// array of offsets to compact ledges

    // grid elements follow    size n_rows * n_columns
    // compact ledges follow   size = sum of size of compact ledges

    /******************************************************************************
     *  Method:		get_grid_elements
     *  Description:    INTERNAL METHOD
     *****************************************************************************/
    const IVP_Compact_Grid_Element *get_grid_elements() {
        //lwss - x64 fixes
    	char *base = (char *)(((intptr_t)this) + this->offset_grid_elements);
    	//lwss end
	return((const IVP_Compact_Grid_Element *)base);
    }

    /******************************************************************************
     *  Method:		get_compact_ledge_at
     *  Description:    INTERNAL METHOD
     *****************************************************************************/
    const IVP_Compact_Ledge *get_compact_ledge_at(int i) {
    	char *base = (char *)(((intptr_t)this) + this->offset_compact_ledge_array[i]);
	return((const IVP_Compact_Ledge *)base);
    }

};

#endif

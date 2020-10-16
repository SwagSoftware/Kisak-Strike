// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_gridbuild_array.hxx
 *  Description:    This file provides you with a class for easy generation of
 *		    a grid (e.g. terrain) from a simple heightfield array.
 *  Classes:	    IVP_GridBuilder_Array
 *		    IVP_Template_Compact_Grid
 *		    IVP_Template_Grid_Axle_Descript
 ********************************************************************************/

#ifndef _IVP_GRIDBUILD_ARRAY_INCLUDED
#define _IVP_GRIDBUILD_ARRAY_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

#define IVP_GRID_MAX_ROWS 257
#define IVP_GRID_MAX_COLUMNS IVP_GRID_MAX_ROWS



class IVP_Compact_Edge;
class IVP_Compact_Grid;
class IVP_Template_Compact_Grid;
class IVP_Compact_Ledge;
class IVP_Compact_Poly_Point;
class IVP_U_Point;
class IVP_Compact_Grid_Element;


/********************************************************************************
 *  Class:	    IVP_Template_Grid_Axle_Descript
 *  Description:    Information class to supply data on the heightfield array.
 *******************************************************************************/

class IVP_Template_Grid_Axle_Descript {
public:
    int			 n_points;	// number of points (not fields) for a certain direction (NOTE: for a grid
                                        // consisting of 8x8 fields this value will be 9 for rows and columns)
    IVP_COORDINATE_INDEX maps_to;	// the coordinate index (IVP axis) the array's row or column is translated onto
    IVP_BOOL		 invert_axis;	// set to true if the grid's "axis" translates to negative IVP axis direction
};


/********************************************************************************
 *  Class:	    IVP_Template_Compact_Grid
 *  Description:    Information class to supply vital data for the heightfield
 *		    conversion.
 *******************************************************************************/

class IVP_Template_Compact_Grid {
public:
    IVP_Template_Grid_Axle_Descript row_info;	// see above...
    IVP_Template_Grid_Axle_Descript column_info;// see above...

    IVP_COORDINATE_INDEX    height_maps_to;	// the IVP coordinate index of the height axis
    IVP_BOOL		    height_invert_axis;	// set to true if the height values translate into negative IVP axis direction

    IVP_FLOAT		    grid_field_size;	// size between two grid points (unit: m)
    IVP_U_Float_Point	    position_origin_os;	// position of the grid's starting point (i.e. the array's first element) in object space

    IVP_Template_Compact_Grid();
};


/********************************************************************************
 *  Class:	    IVP_GridBuilder_Array
 *  Description:    This builder allows for easy creation of grids (most likely
 *		    terrain) from an array of height points.
 *******************************************************************************/

class IVP_GridBuilder_Array {
private:
    // internal data & methods
    int n_rows;
    int n_cols;
    IVP_FLOAT *height_field;
    IVP_Compact_Poly_Point *height_points;
    IVP_BOOL  is_left_handed;

    IVP_Compact_Grid_Element   *ledge_reference_field;
    const IVP_Template_Compact_Grid  *templ;
    IVP_U_Memory	*mm;

    // intermediately used for complete grid
    int *grid_point_to_ledge_point_array;		// -1 means point not in use, >=0 point is allocated and in
    IVP_Compact_Poly_Point *compact_poly_point_buffer; // array of compact poly points
    int	n_compact_poly_points_used;


    // intermediately used for fast conversion of one compact ledge
    inline IVP_Compact_Poly_Point *get_point_at_strip_pos(int pos);

    IVP_Compact_Ledge *c_ledge;
    IVP_Compact_Poly_Point *c_points;							// pointer into the compact_poly_point_buffer
    ushort c_point_to_point_index[IVP_GRID_MAX_ROWS * 2 + 2];		// converts a point in the compact ledge to a point used for c_points
    int triangle_count;

    inline int install_grid_point( int grid_point_index );  // returns the index into the compact_poly_point_buffer
    inline int install_point(const IVP_U_Float_Point *point);
    inline	void insert_opposite_index( const IVP_Compact_Edge *edge, int index);

    void convert_strip_to_compact_ledges(int row, IVP_U_Vector<IVP_Compact_Ledge> *ledges);
    void convert_array_to_compact_ledges( const IVP_Template_Compact_Grid *, IVP_U_Vector<IVP_Compact_Ledge> *ledges);

    IVP_Compact_Ledge *convert_convex_triangle_to_compact_ledge( int strip_points[] );
    IVP_Compact_Ledge *convert_convex_square_to_compact_ledge( int strip_points[], IVP_BOOL is_right_starter);


    IVP_Compact_Grid *compile_ledges_into_compact_grid(const IVP_Template_Compact_Grid *gp, IVP_U_Vector<IVP_Compact_Ledge> *ledges);
    IVP_GridBuilder_Array(IVP_U_Memory	*mm, const IVP_Template_Compact_Grid *gp,IVP_FLOAT *height_field);
    // converts a convex strip to a compact ledge, the reference points are needed for guaranteed convexity
    IVP_Compact_Ledge *convert_convex_stripe_to_compact_ledge_fast(int strip_points[],	int n_points,
											    IVP_U_Float_Point reference_points[2], IVP_BOOL is_single_starter);

    void add_triangle(int p0_off, int p1_off, int p2_off, int opp0, int opp1, int opp2);

public:

    /******************************************************************************
     *  Method:		convert_array_to_compact_grid
     *  Description:    This method will convert an arbitrary heightfield array
     *			into a compact grid by using the information supplied
     *			through the IVP_Template_Compact_Grid parameter.
     *	Input:		<environment>  your physical environment
     *			<parameters>   a filled-in grid template (see above)
     *			<height_field> pointer to your 2-dimensional heightfield
     *			               array
     *	Output:		Pointer to IVP_Compact_Grid structure
     *****************************************************************************/
    static IVP_Compact_Grid* IVP_CDECL convert_array_to_compact_grid(IVP_Environment *environment,
							   const IVP_Template_Compact_Grid *parameters,
							   IVP_FLOAT *height_field);

    ~IVP_GridBuilder_Array();
};


#endif

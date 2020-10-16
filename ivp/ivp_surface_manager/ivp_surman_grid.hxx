// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_surman_grid.hxx
 *  Description:    This file provides you with an interface class to the compact
 *		    grid class. This class is vital to generate physical
 *		    representations of the compact grid.
 *  Classes:	    IVP_SurfaceManager_Grid
 ********************************************************************************/

#ifndef _IVP_SURMAN_GRID_INCLUDED
#define _IVP_SURMAN_GRID_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

class IVP_Compact_Grid;
class IVP_Compact_Triangle;


/********************************************************************************
 *  Class:	    IVP_SurfaceManager_Grid
 *  Description:    A subclass of the basic IVP_SurfaceManager class. Used for
 *		    grid objects. This is sort of an interface to the compact grid
 *		    (geometrical topology) and some of its basic values.
 *  Note:	    Use these methods instead of directly accessing the compact
 *		    grid!
 *******************************************************************************/

class IVP_SurfaceManager_Grid : public IVP_SurfaceManager {
protected:
    // internal methods & data
    IVP_Compact_Grid *compact_grid;
    IVP_SurfaceManager_Grid();
    void traverse_grid(const IVP_U_Point &visitor_position_object_space, IVP_DOUBLE radius, IVP_U_BigVector<IVP_Compact_Ledge> *resulting_ledges) const;

public:

    /******************************************************************************
     *  Description:    INTERNAL METHODS
     *****************************************************************************/
    void add_reference_to_ledge(const IVP_Compact_Ledge *ledge);
    void remove_reference_to_ledge(const IVP_Compact_Ledge *ledge);
    void insert_all_ledges_hitting_ray(IVP_Ray_Solver *, IVP_Real_Object *);
    void get_radius_and_radius_dev_to_given_center(const IVP_U_Float_Point *center, IVP_FLOAT *radius, IVP_FLOAT *radius_deviation) const;
    virtual IVP_SURMAN_TYPE get_type();



    /******************************************************************************
     *  Method:		get_single_convex
     *	Output:		Returns the ledge if compact grid consists of exactly one
     *			(convex) ledge.
     *	Note:		If the compact grid is concave, but consists of two or more
     *			(convex) ledges (e.g. a cube made out of four subcubed)
     *			this method will return NULL
     *****************************************************************************/
    virtual const IVP_Compact_Ledge *get_single_convex() const;

    /******************************************************************************
     *  Method:		get_mass_center
     *	Output:		Returns the compact grid's mass center
     *****************************************************************************/
    void get_mass_center(IVP_U_Float_Point *mass_center_out) const;

    /******************************************************************************
     *  Method:		get_rotation_inertia
     *	Description:	Returns the compact grid's rotation inertia
     *	Input:		<rotation_inertia_out> vector to be filled with inertia
     *****************************************************************************/
    void get_rotation_inertia( IVP_U_Float_Point *rotation_inertia_out ) const;

    /******************************************************************************
     *  Method:		get_all_ledges_within_radius
     *	Description:	Fills a supplied vector with all of the compact grid's
     *			(convex) ledges which are within a certain radius around
     *			a given center point.
     *	Input:		<observer_os>      center point of observation (in compact
     *					   grid coordinate system!)
     *			<radius>           radius to check
     *			<resulting_ledges> vector to be filled with ledges
     *****************************************************************************/
    void get_all_ledges_within_radius(const IVP_U_Point *observer_os, IVP_DOUBLE radius,
				      const IVP_Compact_Ledge *root_ledge, IVP_Real_Object *other_object, const IVP_Compact_Ledge *other_reference_ledge,
				      IVP_U_BigVector<IVP_Compact_Ledge> *resulting_ledges);

    void get_all_terminal_ledges(IVP_U_BigVector<IVP_Compact_Ledge> *resulting_ledges);

    /******************************************************************************
     *  Method:		get_compact_grid
     *	Output:		Returns a pointer to the surface manager's compact grid
     *****************************************************************************/
    const IVP_Compact_Grid *get_compact_grid() const { return(this->compact_grid); }

    /******************************************************************************
     *  Method:		constructor
     *	Description:	Simply initializes the surface manager with the supplied
     *			compact grid.
     *	Input:		<compact_grid_in> compact grid to initilize the
     *			                  surface manager with
     *****************************************************************************/
    IVP_SurfaceManager_Grid(IVP_Compact_Grid *compact_grid_in) { this->compact_grid = compact_grid_in; }

    virtual ~IVP_SurfaceManager_Grid();

};

#endif

// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_surman_mopp.hxx
 *  Description:    This file provides you with an interface class to the compact
 *			surface of any polygonal object with a 'mopp' bounding volume tree. 
 *			This class is vital to generate physical objects.
 *  Classes:	    IVP_SurfaceManager_Mopp
 ********************************************************************************/

#ifndef IVP_SURMAN_MOPP_INCLUDED
#define IVP_SURMAN_MOPP_INCLUDED

#ifndef WIN32
#pragma interface
#endif

class IVP_Compact_Mopp;
class hkMoppCode;

/********************************************************************************
 *  Class:	    IVP_SurfaceManager_Mopp
 *  Description:    A subclass of the basic IVP_SurfaceManager class. Used for
 *		    polygonal objects with 'mopp' bounding volume trees. This is sort of 
 *			an interface to the compact surface (geometrical topology) and some 
 *			of its basic values.
 *  Note:	    Use these methods instead of directly accessing the compact
 *		    surface!
 *******************************************************************************/

class IVP_SurfaceManager_Mopp : public IVP_SurfaceManager 
{
protected:

    IVP_SurfaceManager_Mopp();				// internal methods & data

    const IVP_Compact_Mopp* compact_mopp;	// should be constructed using ivp_malloc_aligned

public:

	static IVP_FLOAT short_to_long_raycast;

	/******************************************************************************
	 *  Description:	INTERNAL METHODS
	 *****************************************************************************/
	void add_reference_to_ledge(const IVP_Compact_Ledge* ledge);

	void remove_reference_to_ledge(const IVP_Compact_Ledge* ledge);

	void insert_all_ledges_hitting_ray(IVP_Ray_Solver* ray_solver, 
		IVP_Real_Object* object);

	void get_radius_and_radius_dev_to_given_center(const IVP_U_Float_Point* center, 
		IVP_FLOAT* radius, 
		IVP_FLOAT* radius_deviation) const;

	virtual IVP_SURMAN_TYPE get_type();

	/******************************************************************************
	 *  Method:		is_single_convex
	 *	Output:		Returns the ledge if compact surface consists of exactly
	 *			one (convex) ledge.
	 *	Note:		If the compact surface is concave, but consists of two
	 *			or more (convex) ledges (e.g. a cube made out of four
	 *			subcubed) this method will return NULL!
	 *****************************************************************************/
	virtual const IVP_Compact_Ledge* get_single_convex() const;

	/******************************************************************************
	 *  Method:		get_mass_center
	 *	Output:		sets the compact surface's mass center
	 *****************************************************************************/
	void get_mass_center(IVP_U_Float_Point* mass_center_out) const;

	/******************************************************************************
	 *  Method:		get_rotation_inertia
	 *	Description:	Returns the compact surface's rotation inertia
	 *	Input:		<rotation_inertia_out> vector to be filled with inertia
	 *****************************************************************************/
	void get_rotation_inertia(IVP_U_Float_Point* rotation_inertia_out) const;

	/******************************************************************************
	 *  Method:		get_all_ledges_within_radius
	 *	Description:	Fills a supplied vector with all of the compact surface's
	 *			(convex) ledges which are within a certain radius around
	 *			a given center point.
	 *	Input:		<root_ledge>	   for recursive compact ledges start search
	 *					   at 'root_ledge'
	 *			<observer_os>      center point of observation (in compact
	 *					   surface coordinate system!)
	 *			<radius>           radius to check
	 *			<resulting_ledges> vector to be filled with ledges
	 *****************************************************************************/
	void get_all_ledges_within_radius(const IVP_U_Point* observer_os, 
		IVP_DOUBLE radius,
		const IVP_Compact_Ledge* root_ledge, 
		IVP_Real_Object* other_object, 
		const IVP_Compact_Ledge* other_reference_ledge,
		IVP_U_BigVector<IVP_Compact_Ledge>* resulting_ledges);

	void get_all_terminal_ledges(IVP_U_BigVector<IVP_Compact_Ledge>* resulting_ledges);

	/******************************************************************************
	 *  Method:		get_compact_mopp
	 *	Output:		Returns a pointer to the surface manager's compact mopp
	 *****************************************************************************/
	const IVP_Compact_Mopp* get_compact_mopp() const 
	{ 
		return compact_mopp;
	}

	/******************************************************************************
	 *  Method:		constructor
	 *	Description:	Simply initializes the surface manager with the supplied
	 *			compact mopp.
	 *	Input:		<compact_surface_in> compact surface to initilize the
	 *			                     surface manager with
	 *****************************************************************************/
	IVP_SurfaceManager_Mopp(const IVP_Compact_Mopp* compact_mopp_in) 
	{
		compact_mopp = compact_mopp_in;
	}

	virtual ~IVP_SurfaceManager_Mopp();
};

#endif // IVP_SURMAN_MOPP_INCLUDED

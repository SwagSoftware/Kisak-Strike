// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

/********************************************************************************
 *	File:	       	ivp_surface_manager.hxx	
 *	Description:	
 ********************************************************************************/

#ifndef _IVP_SURFACE_MANAGER_INCLUDED
#define _IVP_SURFACE_MANAGER_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

class IVP_Compact_Ledge;
class IVP_Ray_Solver;
class IVP_Real_Object;


/********************************************************************************
 *	Name:	     	IVP_SURMAN_TYPE	
 *	Description:	type of surface manager
 ********************************************************************************/
enum IVP_SURMAN_TYPE {
    IVP_SURMAN_POLYGON,
    IVP_SURMAN_BALL
};

class IVP_Vector_of_Ledges_16: public IVP_U_BigVector<IVP_Compact_Ledge> {
    IVP_Compact_Ledge *elem_buffer[16];
public:
    IVP_Vector_of_Ledges_16(): IVP_U_BigVector<IVP_Compact_Ledge>( (void **)&elem_buffer[0],16 ){;};
};

class IVP_Vector_of_Ledges_256: public IVP_U_BigVector<IVP_Compact_Ledge> {
    IVP_Compact_Ledge *elem_buffer[256];
public:
    IVP_Vector_of_Ledges_256(): IVP_U_BigVector<IVP_Compact_Ledge>( (void **)&elem_buffer[0],256 ){;};
};


#define IVP_SURMAN_POLYGON_MAX_VALUE_CLIENTDATA (1<<32)


/********************************************************************************
 *	Name:	      	IVP_SurfaceManager
 *	Description:	A surface_manager is an abstract interface: It's used to
 *			find all ledges of an object that intrude a given ball.
 *	Note:		Different object types require different IVP_SurfaceManager
 *			implementations.
 *	See:		IVP_Environment::create_object
 *			IVP_SurfaceBuilder_Ledge_Soup
 ********************************************************************************/
class IVP_SurfaceManager {
protected:
public:    

    /******************************************************************************
     *  Method:		set/get ledge_client_data
     *  Description:	Allows the user to set and get a ledge specific client_data
     *	Note:		Range of values [0..IVP_SURMAN_POLYGON_MAX_VALUE_CLIENTDATA[
     *  Attention:      Only works for terminal ledges
     *****************************************************************************/
    static void set_ledge_specific_client_data( IVP_Compact_Ledge *, unsigned int value); // assert value < IVP_SURMAN_POLYGON_MAX_VALUE_CLIENTDATA
    static unsigned int get_ledge_specific_client_data( const IVP_Compact_Ledge *);
    
    
    /********************************************************************************
     *	Name:	     	get_single_convex    	
     *	Description:	returns the only compact ledge if surface_manager is a single convex polyhedron or a ball else NULL
     ********************************************************************************/
    virtual const IVP_Compact_Ledge *get_single_convex() const = 0;
  
    /********************************************************************************
     *	Name:	     	get_radius_and_radius_dev_to_given_center    	
     *	Description:	gets the radius and the max( length( surface_normal cross_product surface_point ))
     *                  if radius_deviation can not be calculated, set it to radius
     ********************************************************************************/
    virtual void get_mass_center(IVP_U_Float_Point *mass_center_out) const = 0;
    virtual void get_radius_and_radius_dev_to_given_center(const IVP_U_Float_Point *center, IVP_FLOAT *radius, IVP_FLOAT *radius_deviation) const = 0;
    virtual void get_rotation_inertia( IVP_U_Float_Point *rotation_inertia_out ) const = 0;
  

    /********************************************************************************
     *	Name:	     	get_all_ledges_within_radius    	
     *	Description:	the main function which a surface_manager manager has to implement!!!:
     *                  returns all ledges which intrude into a sphere around the observer's position 
     * 	Note:		Polygons only
     ********************************************************************************/
    virtual void get_all_ledges_within_radius(const IVP_U_Point *observer_position_object, IVP_DOUBLE radius,
					      const IVP_Compact_Ledge *root_ledge, IVP_Real_Object *other_object, const IVP_Compact_Ledge *other_reference_ledge,
					      IVP_U_BigVector<IVP_Compact_Ledge> *resulting_ledges) = 0;

    virtual void get_all_terminal_ledges(IVP_U_BigVector<IVP_Compact_Ledge> *resulting_ledges) = 0;

    /********************************************************************************
     *	Name:	     	insert_all_ledges_hitting_ray    	
     *	Description:	Used for ray casting
     *                  Fills all ledges which are hit by the specified ray into the ray solver
     * 	Note:		For polygons only
     ********************************************************************************/
    virtual void insert_all_ledges_hitting_ray(IVP_Ray_Solver *ray_solver,
					       IVP_Real_Object *object) = 0;
  
    /********************************************************************************
     *	Name:	     	add_/remove_reference_to_ledge    	
     *	Description:	Tells the surface manager that a synapse will use (stop use)
     *			a ledge
     * 	Note:		Polygons only
     ********************************************************************************/
    virtual void add_reference_to_ledge(const IVP_Compact_Ledge *){;};
    virtual void remove_reference_to_ledge(const IVP_Compact_Ledge *){;};

    virtual ~IVP_SurfaceManager() = 0;
    virtual IVP_SURMAN_TYPE get_type()=0;
};


#endif

// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PROTECTED

#ifndef WIN32
#	pragma interface
#endif

class IVP_Triangle;
class IVP_SurfaceManager_Polygon;
class IVP_Object_Polygon_Tetra;
class IVP_Template_Ledge_Polygon_Soup;
class IVP_point_hash;
class IVP_Template_Polygon;
/********************************************************************************
 *	Name:	      	IVP_SurfaceBuilder_Polygon_Convex
 *	Description:	The surface_manager for simple objects, which should be
 *					handled by the collision detection engine as a convex
 *					polyhedron
 *	Attention:	The base polygon HAS TO BE convex!
 *  Version Info:	This file is old, it's just used for IVP internals
 ********************************************************************************/
class IVP_SurfaceBuilder_Polygon_Convex {

    friend class IVP_SurfaceBuilder_Q12;
    friend class IVP_SurfaceBuilder_Halfspacesoup;
    friend class IVP_SurfaceBuilder_Pointsoup;
    
//protected:
public: // lwss: change to public to fix errors
    IVP_Object_Polygon_Tetra *tetras;
    IVP_Compact_Ledge *c_ledge;
    IVP_point_hash *poly_point_hash;
    void init_surface_manager_polygon();

    static IVP_Compact_Ledge *convert_template_to_ledge(IVP_Template_Polygon *templat);	// to be used by polygon soup
    static IVP_Compact_Ledge *convert_templateledgepolygonsoup_to_ledge(IVP_Template_Ledge_Polygon_Soup *templat);
    
    IVP_SurfaceBuilder_Polygon_Convex(IVP_Template_Polygon *templat);
    IVP_SurfaceBuilder_Polygon_Convex(IVP_Template_Ledge_Polygon_Soup *templat);
    
    void fill_list_with_all_triangles(IVP_U_Vector<IVP_Triangle>*tri_list);

public:

    IVP_Compact_Ledge *get_and_remove_compact_ledge()
    {
	IVP_Compact_Ledge *ret;
	ret = c_ledge;
	c_ledge = 0;
	return ret;
    };
    ///////// real public
    ~IVP_SurfaceBuilder_Polygon_Convex();

    /////// very public
    static IVP_SurfaceManager_Polygon *create_surface_manager(IVP_Template_Polygon *templ); 
};





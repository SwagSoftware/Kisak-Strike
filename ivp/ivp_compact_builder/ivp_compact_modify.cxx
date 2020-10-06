// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

/********************************************************************************
 *	File:	       	ivp_compact_modify.cxx	
 *	Description:	collection of functions that affect the shape of a compact surface
 ********************************************************************************/


#include <ivp_physics.hxx>
#include <ivu_float.hxx>

#include <ivp_compact_ledge.hxx>
#include <ivp_compact_modify.hxx>
#include <ivp_compact_surface.hxx>

#include <ivp_surman_polygon.hxx>
#include <ivp_halfspacesoup.hxx>
#include <ivp_surbuild_ledge_soup.hxx>
#include <ivp_surbuild_halfspacesoup.hxx>


///////////////////////////////////////////////////////////////////////////////////////


/********************************************************************************
 *	chop off a slice
 ********************************************************************************/
IVP_Compact_Surface *IVP_Compact_Modify::chop(const IVP_Compact_Surface *c_surface_in,const IVP_U_Float_Point *chop_vector, IVP_FLOAT chop_depth)
{
    // chop_vector is pointing to remaining body

    // pretty slow, totally unoptimized function

    // -----------------------------------------------------------------------
    // get the convex ledges from the surface using a temporary surface manager
    // -----------------------------------------------------------------------
    IVP_U_BigVector<IVP_Compact_Ledge> ledges;
    {
	IVP_SurfaceManager_Polygon temp_surman(c_surface_in);
	temp_surman.get_all_terminal_ledges( &ledges);
	IVP_ASSERT( ledges.len() == 1);	// currently, we just want to chop objects that consist of just ONE convex ledge
    }

    IVP_Compact_Ledge *the_ledge = ledges.element_at(0);

    // now lets cut off one edge by using the halfspace soup functions
    IVP_Halfspacesoup half_spaces(the_ledge);
    
    IVP_DOUBLE merge_eps = 0.01f;

    // find out nearest point to distant plane
    IVP_U_Point chop_vector_normized;
    chop_vector_normized.set(chop_vector);
    chop_vector_normized.normize();

    IVP_U_Hesse test_plane;
    test_plane.set(&chop_vector_normized);		// set direction of halfspace
    IVP_U_Point distant_point;
    distant_point.set(&chop_vector_normized);
    distant_point.mult(-1e6f); // set him far way. in future: check for far enough point.
    test_plane.calc_hesse_val(&distant_point);

    const IVP_Compact_Triangle *tri = the_ledge->get_first_triangle();
    IVP_DOUBLE min_d = -1.0f;
    for(int i=0; i<the_ledge->get_n_triangles(); i++, tri = tri->get_next_tri()) // needn't to be super-fast
    {
	for(int e=0; e<3; e++){
	    const IVP_Compact_Edge *edge = tri->get_edge(e);
	    const IVP_Compact_Poly_Point *start_point = edge->get_start_point(the_ledge);
	    IVP_DOUBLE dist = test_plane.get_dist(start_point);
	    IVP_ASSERT(dist>0.0f); // otherwise we are not far enough away
	    if(dist<min_d || min_d<0.0f) min_d = dist;
	}
    }

    IVP_DOUBLE off = -1e6f + min_d + chop_depth;

    IVP_U_Hesse chop_halfspace;
    chop_halfspace.set(&chop_vector_normized);

    IVP_U_Point chop_point;
    chop_point.set(&chop_vector_normized);
    chop_point.mult(off);
    chop_halfspace.calc_hesse_val(&chop_point);
    
    half_spaces.add_halfspace( &chop_halfspace);	

    IVP_Compact_Surface *chopped_compact_surface;
    chopped_compact_surface = IVP_SurfaceBuilder_Halfspacesoup::convert_halfspacesoup_to_compact_surface( &half_spaces, merge_eps);

    return chopped_compact_surface;
}


IVP_Compact_Ledge *IVP_Compact_Modify::shrink(const IVP_Compact_Ledge *ledge_in, IVP_FLOAT shrink_value, IVP_DOUBLE pointmerge_threshold){
    IVP_Halfspacesoup half_spaces(ledge_in);
    for (int i= half_spaces.len()-1; i>=0; i--){
	IVP_U_Hesse *half_space = half_spaces.element_at(i);
	half_space->hesse_val -= shrink_value;
    }

    IVP_Compact_Ledge *ledge = IVP_SurfaceBuilder_Halfspacesoup::convert_halfspacesoup_to_compact_ledge( &half_spaces, pointmerge_threshold);
    return ledge;
}


IVP_Compact_Surface *IVP_Compact_Modify::shrink(const IVP_Compact_Surface *c_surface_in, IVP_FLOAT shrink_value, IVP_DOUBLE pointmerge_threshold){
    IVP_U_BigVector<IVP_Compact_Ledge> ledges;
    // get all the ledges
    {
	IVP_SurfaceManager_Polygon temp_surman(c_surface_in);
	temp_surman.get_all_terminal_ledges( &ledges);
    }

    // shrink each ledge
    IVP_U_Vector<IVP_Compact_Ledge> ledges_dest;
    IVP_SurfaceBuilder_Ledge_Soup ledge_soup;

    for (int l = ledges.len()-1; l>=0; l--){
	IVP_Compact_Ledge *cl = shrink(ledges.element_at(l), shrink_value, pointmerge_threshold);
	if (cl){
	    ledges_dest.add( cl );
	    ledge_soup.insert_ledge(cl);
	}
    }

    // compile all ledges
    IVP_Compact_Surface *result_compact_surface = ledge_soup.compile();

    return result_compact_surface;
}

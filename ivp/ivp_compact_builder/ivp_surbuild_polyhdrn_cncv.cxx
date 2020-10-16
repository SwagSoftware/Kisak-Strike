// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <ivp_compact_surface.hxx>
#include <ivp_surbuild_pointsoup.hxx>
#include <ivp_template_surbuild.hxx>
#include <ivp_surbuild_ledge_soup.hxx>

#include <ivp_convex_decompositor.hxx>
#include <ivp_surbuild_polyhdrn_cncv.hxx>


/******************************************************************************
 *  Method:	    convert_concave_polyhedron_to_compact_ledges
 *  Description:    This method will convert the supplied concave object into
 *		    a set of (convex) compact ledges.
 *  Input:	    <concave_polyhedron_in>  data on the concave object
 *		    <params>                 some user-definable parameters
 *		    <ledges_out>             address of vector to fill with
 *		                             the resulting compact ledges
 *  Output:	    Number of compact ledges.
 *****************************************************************************/

int IVP_SurfaceBuilder_Polyhedron_Concave::convert_concave_polyhedron_to_compact_ledges(IVP_Concave_Polyhedron *concave_polyhedron_in,	IVP_Convex_Decompositor_Parameters *params,
		    								        IVP_U_BigVector<IVP_Compact_Ledge> *ledges_out) {

    IVP_U_BigVector<IVP_Convex_Subpart> convex_subparts;
    int n_subparts = IVP_Convex_Decompositor::perform_convex_decomposition_on_concave_polyhedron(concave_polyhedron_in,	 params, &convex_subparts);

    for (int x=0; x<n_subparts; x++) {
	IVP_Compact_Ledge *compact_ledge = IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge(&convex_subparts.element_at(x)->points);
	if ( compact_ledge ) {
	    ledges_out->add(compact_ledge);
	}
	delete (convex_subparts.element_at(x));
    }

    return(n_subparts);
}



void IVP_SurfaceBuilder_Polyhedron_Concave::convert_concave_face_soup_to_compact_ledges( IVP_Concave_Polyhedron *concave_polyhedron_in, IVP_U_BigVector<IVP_Compact_Ledge> *ledges_out){
    IVP_U_Vector<IVP_U_Point> points;

	int n_faces = concave_polyhedron_in->faces.len();
	IVP_U_BigVector<IVP_Concave_Polyhedron_Face> *faces = &concave_polyhedron_in->faces;

	IVP_U_BigVector<IVP_U_Point> *points_in = &concave_polyhedron_in->points;

	// sum up total number of all points of all faces
	for (int i=0; i<n_faces; i++) {
		IVP_Concave_Polyhedron_Face *face = faces->element_at(i);
		int n_points_of_face = face->point_offset.len();
		points.clear();
		for (int j=0; j<n_points_of_face; j++) {
		    IVP_U_Point *p = points_in->element_at( face->point_offset.element_at(j)->offset );
		    points.add(p);
		}
		IVP_Compact_Ledge *ledge = IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge( &points );
		ledges_out->add(ledge);
	}



}

/******************************************************************************
 *  Method:	    convert_concave_polyhedron_to_single_compact_surface
 *  Description:    This method will compile a (concave) compact surface from
 *		    the supplied concave object data.
 *  Input:	    <concave_polyhedron_in>  data on the concave object
 *		    <params>                 some user-definable parameters
 *  Output:	    Pointer to IVP_Compact_Surface structure.
 *****************************************************************************/

IVP_Compact_Surface *IVP_SurfaceBuilder_Polyhedron_Concave::convert_concave_polyhedron_to_single_compact_surface(IVP_Concave_Polyhedron *concave_polyhedron_in,
														 IVP_Convex_Decompositor_Parameters *params) {
    IVP_U_BigVector<IVP_Compact_Ledge> ledges_local;

    IVP_SurfaceBuilder_Polyhedron_Concave::convert_concave_polyhedron_to_compact_ledges(concave_polyhedron_in,	params,	&ledges_local);

    IVP_SurfaceBuilder_Ledge_Soup ledge_soup;
    for (int i=0; i<ledges_local.len(); i++) {
	ledge_soup.insert_ledge(ledges_local.element_at(i));
    }

    IVP_Template_Surbuild_LedgeSoup tls;
    tls.build_root_convex_hull = IVP_TRUE;
    tls.merge_points = IVP_SLMP_MERGE_AND_REALLOCATE;
    IVP_Compact_Surface *surface = ledge_soup.compile(&tls);

    return(surface);
}


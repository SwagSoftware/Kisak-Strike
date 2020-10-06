// Copyright (C) Ipion Software GmbH 2000. All rights reserved.


#include <ivp_physics.hxx>

//#include <ivp_surman_polygon.hxx>
#include <ivp_compact_ledge.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_compact_ledge_solver.hxx>
#include <ivp_surbuild_pointsoup.hxx>

#include <ivp_halfspacesoup.hxx>
#include <ivp_surbuild_ledge_soup.hxx>
#include <ivp_surbuild_halfspacesoup.hxx>


// ------------------------------------------------------------------------
// insert_point_into_list
// ======================
//
// check whether 'almost identical' point is already present in list:
// no : insert point and return its pointer
// yes: drop new point and return pointer to old point
//
// NOTE: the passed IVP_U_Point-pointer is invalid after function's return;
//       always use the returned pointer!
// ------------------------------------------------------------------------
IVP_U_Point *IVP_SurfaceBuilder_Halfspacesoup::insert_point_into_list(IVP_U_Point *point, IVP_U_Vector<IVP_U_Point> *points, IVP_DOUBLE quad_threshold)
{
    int i;
    for (i=0; i<points->len(); i++) {
	
	IVP_U_Point *old_point = points->element_at(i);
	if ( point->quad_distance_to(old_point) < quad_threshold ) {
#ifdef INSERT_POINT_INTO_LIST_DEBUG
	    printf("  +++ Dropping almost similar point {%f, %f, %f} in favour of {%f, %f, %f}\n",
		   point->k[0], point->k[1], point->k[2],
		   old_point->k[0], old_point->k[1], old_point->k[2]);
#endif	    
	    P_DELETE(point);
	    return(old_point);
	}
	
    }
    
    points->add(point);
#ifdef INSERT_POINT_INTO_LIST_DEBUG	
    printf("  +++ Insert point {%f, %f, %f}\n", point->k[0], point->k[1], point->k[2]);
#endif

    return(point);
}


// ------------------------------------------------------------------------
// convert_halfspacesoup_to_points
// ===============================
//
// find all intersections between all supplied planes; insert the
// resulting points into the pointlist
// ------------------------------------------------------------------------
int IVP_SurfaceBuilder_Halfspacesoup::convert_halfspacesoup_to_points(IVP_Halfspacesoup *halfspaces,
								      IVP_DOUBLE pointmerge_threshold,
								      IVP_U_Vector<IVP_U_Point> *points)
{
    int i;

    IVP_DOUBLE threshold = pointmerge_threshold * pointmerge_threshold;

    // process all existing triple-plane combinations
    for (i=0; i<halfspaces->len(); i++) {
	int j;
	for (j=i+1; j<halfspaces->len(); j++) {
	    int k;
	    for (k=j+1; k<halfspaces->len(); k++) {

		IVP_U_Hesse *plane1 = halfspaces->element_at(i);
		IVP_U_Hesse *plane2 = halfspaces->element_at(j);
		IVP_U_Hesse *plane3 = halfspaces->element_at(k);

		
		IVP_RETURN_TYPE rval;
		
		IVP_U_Point point;

#ifdef PLANES_TO_POINT_DEBUG
		if ( debug_plane ) {
		    printf("Combining:\n");
		    printf("  {%f,%f,%f}, %f\n", plane1->k[0], plane1->k[1], plane1->k[2], plane1->hesse_val);
		    printf("  {%f,%f,%f}, %f\n", plane2->k[0], plane2->k[1], plane2->k[2], plane2->hesse_val);
		    printf("  {%f,%f,%f}, %f\n", plane3->k[0], plane3->k[1], plane3->k[2], plane3->hesse_val);
		}
#endif
		
		rval = point.set_crossing(plane1, plane2, plane3);
		if ( rval ) {

#ifdef PLANES_TO_POINT_DEBUG			    
		    if ( debug_plane ) {
			printf(" --> intersection at {%f, %f, %f}\n", point.k[0], point.k[1], point.k[2]);
		    }
#endif
		    
		    // an intersection exists...
		    IVP_BOOL skip_point = IVP_FALSE;

		    // check whether point is inside or outside of our
		    // object
		    int l;
		    for (l=0; l<halfspaces->len(); l++) {
			IVP_U_Hesse *plane = halfspaces->element_at(l);
			if ( plane->get_dist(&point) < -HALFSPACESOUP_TOLERANCE ) {
			    skip_point = IVP_TRUE;
#ifdef PLANES_TO_POINT_DEBUG
			    if ( debug_plane ) {
				if ( debug_plane == plane ) {
				    printf("Combining:\n");
				    printf("  {%f,%f,%f}, %f\n", plane1->k[0], plane1->k[1], plane1->k[2], plane1->hesse_val);
				    printf("  {%f,%f,%f}, %f\n", plane2->k[0], plane2->k[1], plane2->k[2], plane2->hesse_val);
				    printf("  {%f,%f,%f}, %f\n", plane3->k[0], plane3->k[1], plane3->k[2], plane3->hesse_val);
				    printf("  +++ Dropping point {%f, %f, %f}\n      because outside of plane {%f, %f, %f}, %f\n      Distance = %f\n",
					   point.k[0], point.k[1], point.k[2],
					   plane->k[0], plane->k[1], plane->k[2], plane->hesse_val,
					   plane->get_dist(&point));
				}
			    }
#endif
			    break;
			}
		    }

		    if ( !skip_point ) {
			// inside... add point to planes;
			insert_point_into_list(new IVP_U_Point(point), points, threshold);
		    }
		    
		}else {
#ifdef P_LANES_TO_POINT_DEBUG
		    if ( debug_plane ) {
			printf(" --> no intersection\n");
		    }
#endif
		}
#ifdef PLANES_TO_POINT_DEBUG			    
		if ( debug_plane ) {
		    printf("\n");
		}
#endif		
	    }
	}
    }
    
    // -------------------------------------------------------------------------------
    // to avoid numerical problems when triangularizing the object we merge any points
    // that are closer than a user-defined threshold
    // -------------------------------------------------------------------------------
    for (i=0; i<points->len(); i++) {
	IVP_U_Point *p1 = points->element_at(i);
	int j;
	for (j=points->len()-1; j>i; j--) {
	    IVP_U_Point *p2 = points->element_at(j);
	    if ( p1->quad_distance_to(p2) < threshold ) {
		IVP_IF(1) {
		    printf("*** IVP_SurfaceBuilder_Halfspacesoup::convert_halfspacesoup_to_points - removing ");
		    p2->print();
		}
		points->remove(p2);
		P_DELETE(p2);
	    }
	}
    }

    return(points->len());
}


IVP_Compact_Ledge *IVP_SurfaceBuilder_Halfspacesoup::convert_halfspacesoup_to_compact_ledge(IVP_Halfspacesoup *halfspaces,
											    IVP_DOUBLE pointmerge_threshold)
{
    IVP_U_Vector<IVP_U_Point> points;

    IVP_SurfaceBuilder_Halfspacesoup::convert_halfspacesoup_to_points(halfspaces, pointmerge_threshold, &points);
    IVP_Compact_Ledge *ledge = IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge(&points);
	for (int i= points.len()-1;i>=0;i--){
		delete points.element_at(i);
	}

    return(ledge);
}


IVP_Compact_Surface *IVP_SurfaceBuilder_Halfspacesoup::convert_halfspacesoup_to_compact_surface(IVP_Halfspacesoup *halfspaces,
												IVP_DOUBLE pointmerge_threshold)
{
    IVP_SurfaceBuilder_Ledge_Soup ledge_soup;
    IVP_Compact_Surface *cs = NULL;

    IVP_Compact_Ledge *ledge = IVP_SurfaceBuilder_Halfspacesoup::convert_halfspacesoup_to_compact_ledge(halfspaces, pointmerge_threshold);
    if ( ledge ) {
	ledge_soup.insert_ledge(ledge);
	cs = ledge_soup.compile();
    } else {
	printf("*** IVP_SurfaceBuilder_Halfspacesoup::convert_halfspacesoup_to_compact_surface - skipping ledge due to invalid topology\n");
    }
    return(cs);
}







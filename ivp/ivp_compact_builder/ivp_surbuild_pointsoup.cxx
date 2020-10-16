// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC
#include <ivp_physics.hxx>

#include <qhull_a.hxx>

#include <ivp_betterdebugmanager.hxx>
#include <ivp_templates_intern.hxx>
#include <ivu_hash.hxx>
#include <ivp_i_point_vhash.hxx>
#include <ivp_surbuild_polygon_convex.hxx>
#include <ivp_compact_ledge.hxx>
#include <ivp_compact_ledge_solver.hxx>
#include <ivp_surbuild_ledge_soup.hxx>
#include <ivp_surbuild_pointsoup.hxx>

#if defined(WIN32) && !defined(_XBOX)
#include "wtypes.h"
#elif defined(_XBOX)
#	ifndef WINVER
#		define WINVER 0x0500
#	endif
#	ifndef _X86_
#		define _X86_
#	endif  /* _X86_ */
#	include <excpt.h>
#	include <stdarg.h>
#	include <windef.h>
#endif

int IVP_SurfaceBuilder_Pointsoup::get_offset_from_pointlist(IVP_Template_Point *points, int length, IVP_U_Point *point)
{
    int	i;

    for (i=0; i<length; i++) {

	if (
	    points[i].k[0] == point->k[0] &&
	    points[i].k[1] == point->k[1] &&
	    points[i].k[2] == point->k[2])
	{
	    return(i);
	}

    }
	
    return(0);
}


int IVP_SurfaceBuilder_Pointsoup::get_offset_from_lineslist(IVP_Template_Line *lines,
							    int length,
							    int pointnr1,
							    int pointnr2,
							    char *reverse)
{
    int	   x;

    for (x=0; x<length; x++) {
	if ( 
	     (pointnr1 == lines[x].p[0])
	  && (pointnr2 == lines[x].p[1])
	) {
	    *reverse = 0;
	    return((ushort)x);
	}
	if ( 
	     (pointnr1 == lines[x].p[1])
	  && (pointnr2 == lines[x].p[0])
	) {
	    *reverse = 1;
	    return(x);
	}
    }
	
    return(0);
}


struct point_hash_key2 {
    ushort offset_point1;
    ushort offset_point2;
};

IVP_Template_Polygon *IVP_SurfaceBuilder_Pointsoup::planes_to_template(IVP_U_Vector<IVP_U_Point> *points,
								       IVP_U_Vector<IVP_SurMan_PS_Plane> *planes)
{
    IVP_Template_Polygon *templ = new IVP_Template_Polygon();

    // ------------------------------------------------------------------------
    // Build Pointlist:
    // extract all points and add them to our
    // own pointlist. Merge all points within a certain distance (see also
    // 'get_offset_from_pointlist(...)'.

    int object_vertex_count = points->len();

    templ->n_points  = 0;
    templ->points   = new IVP_Template_Point[object_vertex_count];

    int x;
    for (x=0; x < object_vertex_count; x++) {

	IVP_U_Point *point = points->element_at(x);

	// check if point (or at least one extremely close to it) is already in list
	if ( get_offset_from_pointlist(templ->points, templ->n_points, point) == 0 ) {

	    // if not, insert it into our point list
	    templ->points[templ->n_points].set(point);
	    templ->n_points++;
	}

    }

    // At this point we have the pointlist completed (still missing are the linelist 
    // and the surfacelist).
    

    // ------------------------------------------------------------------------
    // Build Lineslist:

    IVP_Hash *lines_hash = new IVP_Hash(points->len()*2, 2*sizeof(ushort), 0);
    IVP_U_Vector<struct point_hash_key2> lines_vector;

    int face_count = planes->len();

    templ->n_surfaces = face_count;
    templ->surfaces = new IVP_Template_Surface[face_count];

    templ->n_lines = 0;

    for (x=0; x<face_count; x++) {

	IVP_SurMan_PS_Plane *plane = planes->element_at(x);
	
	int face_vertex_count = plane->points.len();

	// number of lines per polygon equals number of points per polygon
	templ->surfaces[x].n_lines = face_vertex_count;

	for (int pointnr=0; pointnr<face_vertex_count; pointnr++) {

	    IVP_U_Point *point;
	    
	    ushort offset_point1, offset_point2;

	    point = plane->points.element_at(pointnr);
	    offset_point1 = get_offset_from_pointlist(templ->points, templ->n_points, point);

	    if ( pointnr+1 == face_vertex_count ) {
		point = plane->points.element_at(0);
	    }
	    else {
		point = plane->points.element_at(pointnr+1);
	    }
	    offset_point2 = get_offset_from_pointlist(templ->points, templ->n_points, point);

	    // in 'offset_point1/2' we now have the points that build the current line...

	    // construct a unique hash key for the line (independent of pointoffset order)
	    struct point_hash_key2 phk;
	    if ( offset_point2 < offset_point1 ) {
		phk.offset_point1 = offset_point1;
		phk.offset_point2 = offset_point2;
	    }
	    else {
		phk.offset_point1 = offset_point2;
		phk.offset_point2 = offset_point1;
	    }

	    // add line to hash table (if not already there)
	    if ( lines_hash->find((char *)&phk) == 0 ) {
		lines_hash->add((char *)&phk, (void *)1);

		struct point_hash_key2 *nphk = (struct point_hash_key2 *)p_calloc(1, sizeof(struct point_hash_key2));
		nphk->offset_point1 = offset_point1;
		nphk->offset_point2 = offset_point2;
		lines_vector.add(nphk);

		templ->n_lines++;
	    }

	}

    }

    P_DELETE(lines_hash);
    
    // now we have all lines (without duplicates) in our 'lines_vector' list and just
    // need to copy the data to the IVP_Template_Line structure.

    templ->lines = new IVP_Template_Line[templ->n_lines];
    for (x=0; x<lines_vector.len(); x++) {
	struct point_hash_key2 *pphk = lines_vector.element_at(x);
	templ->lines[x].p[0] = pphk->offset_point1;
	templ->lines[x].p[1] = pphk->offset_point2;
	P_FREE(pphk);
    }

    // At this point we have the pointlist completed and the lineslist filled
    // with the appropriate linepoint indizes (still missing are major parts of 
    // the surfacelist and the linelist's 'left' and 'right' surface indizes)


    // ------------------------------------------------------------------------
    // Build Surfacelist:
    // NOTE: the surfacelist has already been (partially) initialized above!

    for (x=0; x<face_count; x++) {

	IVP_SurMan_PS_Plane *plane;
	
	plane = planes->element_at(x);

	templ->surfaces[x].templ_poly    = templ;
	templ->surfaces[x].normal.set(plane);
	templ->surfaces[x].lines         = (ushort *)p_calloc(templ->surfaces[x].n_lines, sizeof(ushort));
	templ->surfaces[x].revert_line   = new char[templ->surfaces[x].n_lines];


	int punktezahl = templ->surfaces[x].n_lines;
	ushort *offset_points = (ushort *)p_calloc(punktezahl+1, sizeof(ushort));

	int y;
	for (y=0; y<punktezahl; y++) {

	    IVP_U_Point *point;

	    point = plane->points.element_at(y);
	    offset_points[y] = get_offset_from_pointlist(templ->points, templ->n_points, point);

	}
	offset_points[y] = offset_points[0];


	// number of points equals number of lines!
	for (int pointnr=0; pointnr<punktezahl; pointnr++) {

	    char reverse;
	    int line_index = get_offset_from_lineslist(templ->lines, templ->n_lines,
						       offset_points[pointnr], offset_points[pointnr+1], &reverse);
	    templ->surfaces[x].lines[pointnr] = line_index;
	    templ->surfaces[x].revert_line[pointnr] = ((reverse*(-1))+1);

	}

	P_FREE(offset_points);

    }

    return(templ);
}

IVP_DOUBLE IVP_SurMan_PS_Plane::get_area_size(){
  IVP_DOUBLE sum = 0.0;
  IVP_U_Point *p0 = points.element_at(0);
  for (int i = 0; i< points.len(); i++){
    int in = (i+1) % points.len();
    IVP_U_Point *a = points.element_at(i);
    IVP_U_Point *b = points.element_at(in);
    IVP_U_Point diff0; diff0.subtract( b, a);
    IVP_U_Point diff1; diff1.subtract( a, p0);
    IVP_U_Point cross; cross.calc_cross_product( &diff0, &diff1);
    sum += cross.dot_product( this );
  }
  return sum;
}

IVP_DOUBLE IVP_SurMan_PS_Plane::get_qlen_of_all_edges(){
  IVP_DOUBLE sum = 0.0;
  for (int i = 0; i< points.len(); i++){
    int in = (i+1) % points.len();
    IVP_U_Point *a = points.element_at(i);
    IVP_U_Point *b = points.element_at(in);
    sum += a->quad_distance_to(b);
  }
  return sum;
}


void IVP_SurfaceBuilder_Pointsoup::error_output(IVP_Template_Polygon *templ)
{
//    ivp_debugmanager.dprint(IVP_DM_SURBUILD_POINTSOUP, "IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge_internal() - failed to triangulize object!\n");
    ivp_debugmanager.dprint(IVP_DM_SURBUILD_POINTSOUP, "Object points:\n");
    int i;
    for (i=0; i<templ->n_points; i++) {
	ivp_debugmanager.dprint(IVP_DM_SURBUILD_POINTSOUP, "Point [%d] : ", i); 
	templ->points[i].print();
    }
    ivp_debugmanager.dprint(IVP_DM_SURBUILD_POINTSOUP, "Object edges:\n");
    for (i=0; i<templ->n_lines; i++) {
	int p0 = templ->lines[i].p[0];
	int p1 = templ->lines[i].p[1];
	ivp_debugmanager.dprint(IVP_DM_SURBUILD_POINTSOUP, "Distance [%d] - [%d] : %f\n", p0, p1, templ->points[p0].quad_distance_to(&templ->points[p1]));
    }
    ivp_debugmanager.dprint(IVP_DM_SURBUILD_POINTSOUP, "\n");
    return;
}

class IVP_SB_PS_DUMMY {
  int dummy;
};

IVP_Compact_Ledge *IVP_SurfaceBuilder_Pointsoup::try_to_build_convex_ledge_from_qhull_result(IVP_U_Vector<IVP_U_Point> *points, IVP_BOOL *skip_point, char *skip_list, char *use_list){
  *skip_point = IVP_FALSE;
  // DEBUG: Print all vertices of convex hull
	IVP_IF(1) {
	    IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
		vertexT *vertex;	    /* set by FORALLvertices */
		FORALLvertices {
		    int point_index = qh_pointid(vertex->point);
		    ivp_debugmanager.dprint(IVP_DM_SURBUILD_POINTSOUP, "Point in convex hull: %f %f %f\n", points->element_at(point_index)->k[0], points->element_at(point_index)->k[1], points->element_at(point_index)->k[2]);
		}
	    }
	}
	
	IVP_U_Vector<IVP_SurMan_PS_Plane> planes;
	IVP_U_Vector<IVP_SB_PS_DUMMY> point_indizes; // array of point indizes
	
	facetT *facet;	    /* set by FORALLfacets */	
	FORALLfacets {
	    IVP_SurMan_PS_Plane *plane = new IVP_SurMan_PS_Plane();
	    planes.add(plane);
	    for (int k=0; k < qh hull_dim; k++) {
		plane->k[k] = facet->normal[k];
	    }
	    //plane->hesse_val = facet->offset;
	    
	    vertexT *vertex, **vertexp;
	    setT *vertices;
	    vertices= qh_facet3vertex (facet);
	    point_indizes.remove_all();
	    FOREACHvertex_(vertices) {
		int point_index = qh_pointid(vertex->point);
		point_indizes.add( (IVP_SB_PS_DUMMY *)point_index );
		plane->points.add(points->element_at(point_index));
		use_list[point_index] ++;
	    }
	    // check area size of facet
	    IVP_DOUBLE area_sum = plane->get_area_size();
	    IVP_DOUBLE qedge_sum = plane->get_qlen_of_all_edges();

	    // swap area
	    if (area_sum < 0){
		plane->mult(-1);
		area_sum *= -1;
		//plane->points.reverse();
		//point_indizes.reverse();
	    }
	    
	    if ( area_sum  <  0.5 * P_MIN_EDGE_LEN * IVP_Inline_Math::ivp_sqrtf(qedge_sum)){
	      // degenarete area -> find and delete interior point
	      // search point which is farest from point 0
	      IVP_DOUBLE max_dist = -1.0;
	      int max_index = 0;
	      int max_index2 = 0;
	      for (int i0 = 0; i0 < point_indizes.len(); i0++){
        //lwss - x64 fixes
        //int in = int(point_indizes.element_at(i0));
        intptr_t in = intptr_t(point_indizes.element_at(i0));
		//lwss end
		if (skip_list[in]) goto no_point_skipped;
		IVP_U_Point *p2 = plane->points.element_at( i0 );
		IVP_DOUBLE dist = plane->points.element_at( 0 )->quad_distance_to(p2);
		if (dist > max_dist){
		  max_index = i0;
		  max_dist = dist;
		}
	      }
	      // search point with a maximum distance to max_index
	      max_dist = -1.0;
	      {
		  for (int i1 = 0; i1 < point_indizes.len(); i1++){
		    *skip_point = IVP_TRUE;
		    IVP_U_Point *p2 = plane->points.element_at( i1 );
		    IVP_DOUBLE dist = plane->points.element_at( max_index )->quad_distance_to(p2);
		    if (dist > max_dist){
		      max_index2 = i1;
		      max_dist = dist;
		    }
		  }
	      }
	      IVP_ASSERT( max_dist < qedge_sum);
	      {
		  for (int i2 = 0; i2 < point_indizes.len(); i2++){
		    if (i2 != max_index2 && i2 != max_index) {
              //lwss - x64 fixes
		      //int in = int(point_indizes.element_at(i2));
		      intptr_t in = intptr_t(point_indizes.element_at(i2));
		      //lwss end
		      skip_list[in]++;
		      IVP_ASSERT( use_list[in] );
		      //printf("point removed %i %i\n",in, points->len());

		      break;
		    }
		  }
	      }
	    no_point_skipped:;
	    } // for all facets
	    
	    qh_settempfree(&vertices);
	    IVP_IF(0) {
		IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
		    ivp_debugmanager.dprint(IVP_DM_SURBUILD_POINTSOUP, "n_faces: %d\n", plane->points.len());
		    if ( plane->points.len() > 3 ) { //@@@SF: Debug [14.12.1999]
			ivp_debugmanager.dprint(IVP_DM_SURBUILD_POINTSOUP, "[Testing/Debug] Qhull : more than 3 points on plane!\n");
		    }
		}
	    }
	}

	IVP_Compact_Ledge *compact_ledge = NULL;
	// --------------------------------------------------
	// convert convex points & planes to polygon template
	// --------------------------------------------------
	if (*skip_point  == IVP_FALSE){
	  IVP_Template_Polygon *templ =  planes_to_template(points, &planes);
	  // -----------------------------------------
	  // convert polygon template to compact ledge
	  // -----------------------------------------
	  compact_ledge = IVP_SurfaceBuilder_Polygon_Convex::convert_template_to_ledge(templ);
	  IVP_IF(1) {
	    IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
	      if ( !compact_ledge ) {
		error_output(templ);
	      }
	    }
	  }
	  P_DELETE(templ);
	}
	
 	// --------------------
	// intermediate cleanup
 	// --------------------
	int i;
	for (i=0; i<planes.len(); i++) {
	    IVP_SurMan_PS_Plane *plane = planes.element_at(i);
	    P_DELETE(plane);
	}

	return compact_ledge;
}

IVP_Compact_Ledge *IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge_internal(IVP_U_Vector<IVP_U_Point> *points_in)
{
    IVP_IF(1) {
	IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
	    ivp_debugmanager.dprint(IVP_DM_SURBUILD_POINTSOUP, "*** Starting building new convex hull from pointsoup.\n");
	}
    }


    // --------------
    // starting qhull
    // --------------
    int dim = 3;              // dimension of points

    // prepare points
    IVP_U_Vector<IVP_U_Point> points( points_in->len() );
    coordT *points2 = (coordT *)p_calloc(points_in->len() * (dim+1), sizeof(coordT));
    {  // copy vector and remove duplicated points
	IVP_I_Point_VHash point_hash( 1024 ); // very rough estimate
	int d = 0;
	for (int i=0; i<points_in->len(); i++) {
	    IVP_U_Point *p = points_in->element_at(i);
	    IVP_U_Point *found_point = point_hash.find_point(p);
	    if (found_point){
		continue;
	    }
	    point_hash.add_point( p );
	    points.add(p);
	    points2[d+0] = p->k[0];
	    points2[d+1] = p->k[1];
	    points2[d+2] = p->k[2];
	    d += dim;
	}
    }

    int numpoints = points.len();
    char *skip_list = (char *)p_calloc(numpoints, sizeof(char));
    char *use_list = (char *)p_calloc(numpoints, sizeof(char));

    // --------------
    // starting qhull
    // --------------

#if defined(PSXII) || defined(WIN32) || defined(LINUX) || defined(GEKKO)
    FILE *outfile = NULL;   // output from qh_produce_output() use NULL to skip qh_produce_output()
    FILE *errfile = NULL;   // error messages from qhull code
#else
    FILE *outfile = stderr;     // stdout // output from qh_produce_output() use NULL to skip qh_produce_output()
    FILE *errfile = stderr;     // stderr // error messages from qhull code
#endif
    boolT ismalloc = False;    // True if qhull should free points in qh_freeqhull() or reallocation
    
    char flags[250];
    //sprintf(flags, "qhull Qx Pp E7.9e-20"); // "qhull QbB Pp"
    //sprintf(flags, "qhull QJ Pp E7.9e-20"); // "qhull QbB Pp"


    // first loop, try to randomize point 0 until we get an result
    IVP_Compact_Ledge *res = NULL;
    IVP_BOOL try_unjumbled = IVP_TRUE;
    IVP_BOOL qhull_free_flag = IVP_FALSE;
    for ( IVP_DOUBLE random_eps = 1e-12f; random_eps < 0.02f;  ){  
      sprintf(flags, "qhull Qs Pp C-0 W1e-14 E1.0e-18"); // "qhull QbB Pp"
      int exitcode = 0;
      if (try_unjumbled){
	  if (qhull_free_flag)	  qh_freeqhull(!qh_ALL);                   /* free long memory  */
	  exitcode = qh_new_qhull(dim, numpoints, points2, ismalloc, flags, outfile, errfile);
	  qhull_free_flag = IVP_TRUE;
      }
      if ( exitcode || !try_unjumbled) {      
	IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
	  ivp_debugmanager.dprint(IVP_DM_SURBUILD_POINTSOUP, "*** Qhull failed. Retrying with different parameters.\n");
	}

	sprintf(flags, "qhull Qs QJ%G C-0 Pp W1e-14 E1.0e-18",random_eps); // "qhull QbB Pp"
	if (qhull_free_flag)	  qh_freeqhull(!qh_ALL);                   /* free long memory  */
	exitcode = qh_new_qhull(dim, numpoints, points2, ismalloc, flags, outfile, errfile);
	qhull_free_flag = IVP_TRUE;
	if (exitcode){
    	    random_eps = ( random_eps + 1e-12f ) * 1.2f;
	    try_unjumbled = IVP_FALSE;
	    continue;
	}
      }      
      
      IVP_BOOL skip_point = IVP_FALSE;
      
      memset( use_list, 0, numpoints);  // reset list
      memset( skip_list, 0, numpoints);  // reset list
      res = try_to_build_convex_ledge_from_qhull_result(&points, &skip_point, skip_list, use_list);

      if (res) //@@CB
      {
	// associate points_in extra data with compact ledge points
	IVP_I_Point_VHash point_hash( 1024 );

	{
	    // add all non duplicate points to the hash for comparison
	    // against the actual compact ledge points (some points may
	    // get discarded by qhull)
	    for(int i = 0; i < points.len(); i++)
	    {
		point_hash.add_point(points.element_at(i));
	    }
	}

	// number of points in the compact ledge
	int compact_ledge_num_points = (res->get_size() / 16) - (res->get_n_triangles()) - 1;

	{
	    // cycle through the compact ledge points, find the corresponding point
	    // in the hash and copy the extra data (hesse_val) value to the compact
	    // point
	    for(int i = 0; i < compact_ledge_num_points; i++)
	    {
		IVP_Compact_Poly_Point* cpp = &((res->get_point_array())[i]);
		IVP_U_Point* associated_point = point_hash.find_point(reinterpret_cast<IVP_U_Point*>(cpp));

		if(associated_point)
		{
		    cpp->hesse_val = associated_point->hesse_val;
		}
	    }
	}

	break;
      }

      {
	int dest = 0;
	for (int x = 0; x < numpoints; x++){
	  if ( use_list[x] && !skip_list[x]){
	    points2[3*dest + 0] = points2[ 3*x + 0];
	    points2[3*dest + 1] = points2[ 3*x + 1];
	    points2[3*dest + 2] = points2[ 3*x + 2];
	    points.elems[dest] = points.elems[x];
	    dest ++;
	  }
	}
	points.n_elems = dest;
	if (numpoints == dest){
    	    random_eps = ( random_eps + 1e-12f ) * 1.2f;
	    try_unjumbled = IVP_FALSE;
	}
	numpoints = dest;
	if (numpoints == 3) {
	    res = convert_pointsoup_to_compact_ledge( &points );
	}	
	if (numpoints <= 3){
	  break;
	}
      }
    }

    // free all the memory
    {
	qh_freeqhull(!qh_ALL);                   /* free long memory  */
	int curlong, totlong;
	qh_memfreeshort( & curlong, &totlong );

	P_DELETE(points2);
	P_FREE(skip_list);
	P_FREE(use_list);
    }
    if (!res){
      IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
	ivp_debugmanager.dprint(IVP_DM_SURBUILD_POINTSOUP, "*** IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_template_polygon - couldn't build convex hull! Skipping object...\n");
	int i;
	for (i=0; i<points.len(); i++) {
	  //points->element_at(i)->print();
	}
      }

    }
    IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
      ivp_debugmanager.dprint(IVP_DM_SURBUILD_POINTSOUP, "*** Done with convex pointsoup.\n\n");
    }
    return res;
}

IVP_Compact_Ledge *IVP_SurfaceBuilder_Pointsoup::single_tri_ledge = NULL;


void IVP_SurfaceBuilder_Pointsoup::cleanup(){
    if (single_tri_ledge){
	ivp_free_aligned( single_tri_ledge );
    }
    single_tri_ledge = NULL;
}

IVP_Compact_Ledge *IVP_SurfaceBuilder_Pointsoup::convert_triangle_to_compace_ledge( IVP_U_Point *p0, IVP_U_Point *p1, IVP_U_Point *p2){
    if ( !single_tri_ledge ){
	IVP_Template_Ledge_Polygon_Soup ledge_templ;
	IVP_Template_Triangle templ_tri;
	
	templ_tri.tri_points[0].set(0,0,1);
	templ_tri.tri_points[1].set(0,1,0);
	templ_tri.tri_points[2].set(0,0,-1);
	
	ledge_templ.ledge_is_open = IVP_FALSE; // create other side automatically
	ledge_templ.n_templ_triangles = 2;
	ledge_templ.templ_triangles_array = &templ_tri;	
	single_tri_ledge = IVP_SurfaceBuilder_Polygon_Convex::convert_templateledgepolygonsoup_to_ledge(&ledge_templ);
    }

    IVP_U_Hesse th;    th.inline_set_vert_to_area_defined_by_three_points(p0,p1,p2);
    if (th.quad_length() < P_DOUBLE_RES) return NULL;

    IVP_Compact_Ledge *cl = (IVP_Compact_Ledge *)ivp_malloc_aligned(single_tri_ledge->get_size(),16);
    memcpy( (void *)cl, (void *)single_tri_ledge, single_tri_ledge->get_size());

    IVP_Compact_Poly_Point *pa = cl->get_point_array();
    pa[0].set(p0);
    pa[1].set(p1);
    pa[2].set(p2);
    return cl;
}


IVP_Compact_Ledge *IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge(IVP_U_Vector<IVP_U_Point> *points)
{
  /************************************************
  * FPU mode
  ************************************************/
  //doesnt work with threads !!
#ifdef WIN32
  WORD tmpflag;
  __asm FSTCW tmpflag;

  WORD newFPUflag = tmpflag | 0x0300;
  __asm FLDCW newFPUflag;
#endif
    int n_points = points->len();

    if (n_points <3) return NULL;
    if ( n_points == 3 ) { // special case: 2-dimensional triangles
	return convert_triangle_to_compace_ledge( points->element_at(0), points->element_at(1), points->element_at(2));
    } else { // use QHULL to convert pointsoup
	return IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge_internal(points);
    }
#ifdef WIN32
  __asm FLDCW tmpflag;
#endif
}


IVP_Compact_Surface *IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_surface(IVP_U_Vector<IVP_U_Point> *points)
{
    IVP_Compact_Surface *compact_surface = NULL;

    IVP_Compact_Ledge *ledge = IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge(points);
    if ( ledge ) {
#ifdef DEBUG
		IVP_Compact_Ledge_Solver::check_ledge(ledge);
#endif
	IVP_SurfaceBuilder_Ledge_Soup soup;
	soup.insert_ledge(ledge);
	compact_surface = soup.compile();
    }
    else {
	IVP_IF(1) {
	    IVP_IFDEBUG(IVP_DM_SURBUILD_POINTSOUP) {
		ivp_debugmanager.dprint(IVP_DM_SURBUILD_POINTSOUP, "*** IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_surface - skipping ledge due to invalid topology\n");
	    }
	}
    }

    return(compact_surface);
}




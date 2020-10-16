/*<html><pre>  -<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="TOP">-</a>

   poly2.c 
   implements polygons and simplices

   see qh-c.htm, poly.h and qhull.h

   frequently used code is in poly.c

   copyright (c) 1993-1999, The Geometry Center
*/

#include <ivp_physics.hxx>
#include "qhull_a.hxx"

/*======== functions in alphabetical order ==========*/

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="addhash">-</a>
  
  qh_addhash( newelem, hashtable, hashsize, hash )
    add newelem to linear hash table at hash if not already there
*/
void qh_addhash (void* newelem, setT *hashtable, int hashsize, unsigned hash) {
  int scan;
  void *elem;

  for (scan= (int)hash; (elem= SETelem_(hashtable, scan)); 
       scan= (++scan >= hashsize ? 0 : scan)) {
    if (elem == newelem)
      break;
  }
  /* loop terminates because qh_HASHfactor >= 1.1 by qh_initbuffers */
  if (!elem)
    SETelem_(hashtable, scan)= newelem;
} /* addhash */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="check_bestdist">-</a>
  
  qh_check_bestdist()
    check that all points are within max_outside of the nearest facet
    if qh.ONLYgood,
      ignores !good facets

  see: 
    qh_check_maxout(), qh_outerinner()

  notes:
    if notverified>0 at end of routine
      some points were well inside the hull.  If the hull contains
      a lens-shaped component, these points were not verified.  Use
      options 'Qi Tv' to verify all points.  (Exhaustive check also verifies)

  design:
    determine facet for each point (if any)
    for each point
      start with the assigned facet or with the first facet
      find the best facet for the point and check all coplanar facets
      error if point is outside of facet
*/
void qh_check_bestdist (void) {
  boolT waserror= False, isoutside, unassigned;
  facetT *facet, *bestfacet, *errfacet1= NULL, *errfacet2= NULL;
  facetT *facetlist; 
  realT dist, maxoutside, maxdist= -REALmax;
  pointT *point;
  int numpart, facet_i, facet_n, notgood= 0, notverified= 0;
  setT *facets;

  trace1((qh ferr, "qh_check_bestdist: check points below nearest facet.  Facet_list f%d\n",
      qh facet_list->id));
  maxoutside= qh_maxouter();
  maxoutside += qh DISTround;
  /* one more qh.DISTround for check computation */
  trace1((qh ferr, "qh_check_bestdist: check that all points are within %2.2g of best facet\n", maxoutside));
  facets= qh_pointfacet (/*qh facet_list*/);
  if (!qh_QUICKhelp && qh PRINTprecision)
    ivp_message( "\n\
qhull output completed.  Verifying that %d points are\n\
below %2.2g of the nearest %sfacet.\n",
	     qh_setsize(facets), maxoutside, (qh ONLYgood ?  "good " : ""));
  FOREACHfacet_i_(facets) {  /* for each point with facet assignment */
    if (facet)
      unassigned= False;
    else {
      unassigned= True;
      facet= qh facet_list;
    }
    point= qh_point(facet_i);
    if (point == qh GOODpointp)
      continue;
    bestfacet= qh_findbest (point, facet, qh_ALL, False, !qh_NOupper,
			    &dist, &isoutside, &numpart);
    /* occurs after statistics reported */
    maximize_(maxdist, dist);
    if (dist > maxoutside) {
      if (qh ONLYgood && !bestfacet->good 
	  && !((bestfacet= qh_findgooddist (point, bestfacet, &dist, &facetlist))
	       && dist > maxoutside))
	notgood++;
      else {
	waserror= True;
	ivp_message( "qhull precision error: point p%d is outside facet f%d, distance= %6.8g maxoutside= %6.8g\n", 
		facet_i, bestfacet->id, dist, maxoutside);
	errfacet2= errfacet1;
	errfacet1= bestfacet;		    
      }
    }else if (unassigned && dist < -qh MAXcoplanar)
      notverified++;
  }
  qh_settempfree (&facets);
  if (notverified && !qh DELAUNAY && !qh_QUICKhelp && qh PRINTprecision) 
    ivp_message( "\n%d points were well inside the hull.  If the hull contains\n\
a lens-shaped component, these points were not verified.  Use\n\
options 'Qci Tv' to verify all points.\n", notverified); 
  if (maxdist > qh outside_err) {
    ivp_message( "qhull precision error (qh_check_bestdist): a coplanar point is %6.2g from convex hull.  The maximum value (qh.outside_err) is %6.2g\n",
              maxdist, qh outside_err);
    qh_errexit2 (qh_ERRprec, errfacet1, errfacet2);
  }else if (waserror && qh outside_err > REALmax/2)
    qh_errexit2 (qh_ERRprec, errfacet1, errfacet2);
  else if (waserror){
                           /* the error was logged to qh_errlog() but does not effect the output */
    trace0((qh ferr, "qh_check_bestdist: max distance outside %2.2g\n", maxdist));
    int x = 0; x= 2*x;
  }
} /* check_bestdist */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="check_maxout">-</a>
  
  qh_check_maxout()
    updates qh.max_outside by checking all points against bestfacet
    if qh.ONLYgood, ignores !good facets

  returns:
    updates facet->maxoutside via qh_findbest()
    sets qh.maxoutdone
    if printing qh.min_vertex (qh_outerinner), 
      it is updated to the current vertices
    removes inside/coplanar points from coplanarset as needed

  notes:
    defines coplanar as min_vertex instead of MAXcoplanar 
    may not need to check near-inside points because of qh.MAXcoplanar 
      and qh.KEEPnearinside (before it was -DISTround)

  see also:
    qh_check_bestdist()

  design:
    if qh.min_vertex is needed
      for all neighbors of all vertices
        test distance from vertex to neighbor
    determine facet for each point (if any)
    for each point with an assigned facet
      find the best facet for the point and check all coplanar facets
        (updates outer planes)
    remove near-inside points from coplanar sets
*/
#ifndef qh_NOmerge
void qh_check_maxout (void) {
  facetT *facet, *bestfacet, *neighbor, **neighborp, *facetlist;
  realT dist, maxoutside, minvertex;
  pointT *point;
  int numpart, facet_i, facet_n, notgood= 0;
  setT *facets, *vertices;
  vertexT *vertex;

  trace1((qh ferr, "qh_check_maxout: check and update maxoutside for each facet.\n"));
  maxoutside= minvertex= 0;
  if (qh VERTEXneighbors 
  && (qh PRINTsummary || qh KEEPinside || qh KEEPcoplanar 
	|| qh TRACElevel || qh PRINTstatistics
	|| qh PRINTout[0] == qh_PRINTsummary || qh PRINTout[0] == qh_PRINTnone)) { 
    trace1((qh ferr, "qh_check_maxout: determine actual maxoutside and minvertex\n"));
    vertices= qh_pointvertex (/*qh facet_list*/);
    FORALLvertices {
      FOREACHneighbor_(vertex) {
        zinc_(Zdistvertex);  /* distance also computed by main loop below */
	qh_distplane (vertex->point, neighbor, &dist);
	minimize_(minvertex, dist);
	if (-dist > qh TRACEdist || dist > qh TRACEdist 
	|| neighbor == qh tracefacet || vertex == qh tracevertex)
	  ivp_message( "qh_check_maxout: p%d (v%d) is %.2g from f%d\n",
		    qh_pointid (vertex->point), vertex->id, dist, neighbor->id);
      }
    }
    if (qh MERGING) {
      wmin_(Wminvertex, qh min_vertex);
    }
    qh min_vertex= minvertex;
    qh_settempfree (&vertices);  
  }
  facets= qh_pointfacet (/*qh facet_list*/);
  FOREACHfacet_i_(facets) {     /* for each point with facet assignment */
    if (facet) { 
      point= qh_point(facet_i);
      if (point == qh GOODpointp)
	continue;
      zinc_(Ztotcheck);
      bestfacet= qh_findbest (point, facet, qh_ALL, False, !qh_NOupper,
			         &dist, NULL, &numpart);
      zadd_(Zcheckpart, numpart);
      if (bestfacet && dist > maxoutside) {
        if (qh ONLYgood && !bestfacet->good 
        && !((bestfacet= qh_findgooddist (point, bestfacet, &dist, &facetlist))
             && dist > maxoutside))
          notgood++;
        else
	  maxoutside= dist;
      }
      if (dist > qh TRACEdist || (bestfacet && bestfacet == qh tracefacet))
	ivp_message( "qh_check_maxout: p%d is %.2g above f%d\n",
		   qh_pointid (point), dist, bestfacet->id);
    }
  }
  qh_settempfree (&facets);
  wval_(Wmaxout)= maxoutside - qh max_outside;
  wmax_(Wmaxoutside, qh max_outside);
  qh max_outside= maxoutside;
  qh_nearcoplanar (/*qh.facet_list*/);
  qh maxoutdone= True;
  trace1((qh ferr, "qh_check_maxout: maxoutside %2.2g, min_vertex %2.2g, outside of not good %d\n",
       maxoutside, qh min_vertex, notgood));
} /* check_maxout */
#else /* qh_NOmerge */
void qh_check_maxout (void) {
}
#endif

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="check_output">-</a>
  
  qh_check_output()
    performs the checks at the end of qhull algorithm
*/
void qh_check_output (void) {
  int i;

  if (qh STOPcone)
    return;
  if (qh VERIFYoutput | qh IStracing | qh CHECKfrequently) {
    qh_checkpolygon (qh facet_list);
    qh_checkflipped_all (qh facet_list);
    qh_checkconvex (qh facet_list, qh_ALGORITHMfault);
  }else if (!qh MERGING && qh_newstats (qhstat precision, &i)) {
    qh_checkflipped_all (qh facet_list);
    qh_checkconvex (qh facet_list, qh_ALGORITHMfault);
  }
} /* check_output */



/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="check_point">-</a>
  
  qh_check_point( point, facet, maxoutside, maxdist, errfacet1, errfacet2 )
    check that point is less than maxoutside from facet
*/
void qh_check_point (pointT *point, facetT *facet, realT *maxoutside, realT *maxdist, facetT **errfacet1, facetT **errfacet2) {
  realT dist;

  /* occurs after statistics reported */
  qh_distplane(point, facet, &dist);
  if (dist > *maxoutside) {
    *errfacet2= *errfacet1;
    *errfacet1= facet;
    ivp_message( "qhull precision error: point p%d is outside facet f%d, distance= %6.8g maxoutside= %6.8g\n", 
	      qh_pointid(point), facet->id, dist, *maxoutside);
  }
  maximize_(*maxdist, dist);
} /* qh_check_point */


/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="check_points">-</a>
  
  qh_check_points()
    checks that all points are inside all facets

  notes:
    uses qh_findbest if lots of points
    ignores flipped facets
    maxoutside includes 2 qh.DISTrounds
      one qh.DISTround for the computed distances in qh_check_points
    qh_printafacet and qh_printsummary needs only one qh.DISTround
    the computation for qh.VERIFYdirect does not account for qh.other_points

  design:
    if many points
      use qh_check_bestdist()
    else
      for all facets
        for all points
          check that point is inside facet
*/
void qh_check_points (void) {
  facetT *facet, *errfacet1= NULL, *errfacet2= NULL;
  realT total, maxoutside, maxdist= -REALmax;
  pointT *point, **pointp, *pointtemp;
  boolT testouter;

  maxoutside= qh_maxouter();
  maxoutside += qh DISTround;
  /* one more qh.DISTround for check computation */
  trace1((qh ferr, "qh_check_points: check all points below %2.2g of all facet planes\n",
	  maxoutside));
  if (qh num_good)   /* miss counts other_points and !good facets */
     total= (float) qh num_good * qh num_points;
  else
     total= (float) qh num_facets * qh num_points;
  if (total >= qh_VERIFYdirect && !qh maxoutdone) {
    if (!qh_QUICKhelp && qh SKIPcheckmax && qh MERGING)
      ivp_message( "\n\
qhull input warning: merging without checking outer planes ('Q5').\n\
Verify may report that a point is outside of a facet.\n");
    qh_check_bestdist();
  }else {
    if (qh_MAXoutside && qh maxoutdone)
      testouter= True;
    else
      testouter= False;
    if (!qh_QUICKhelp) {
      if (qh MERGEexact || qh SKIPcheckmax || qh NOnearinside)
	ivp_message( "\n\
qhull input warning: exact merge ('Qx'), no outer plane check ('Q5'), or\n\
no processing of near-inside points ('Q8').  Verify may report that a point\n\
is outside of a facet.\n");
    }
    if (qh PRINTprecision) {
      if (testouter)
	ivp_message( "\n\
Output completed.  Verifying that all points are below outer planes of\n\
all %sfacets.  Will make %2.0f distance computations.\n", 
	      (qh ONLYgood ?  "good " : ""), total);
      else
	ivp_message( "\n\
Output completed.  Verifying that all points are below %2.2g of\n\
all %sfacets.  Will make %2.0f distance computations.\n", 
	      maxoutside, (qh ONLYgood ?  "good " : ""), total);
    }
    FORALLfacets {
      if (!facet->good && qh ONLYgood)
        continue;
      if (facet->flipped)
        continue;
      if (testouter) {
#if qh_MAXoutside
	maxoutside= facet->maxoutside + 2* qh DISTround;
	/* one DISTround to actual point and another to computed point */
#endif
      }
      FORALLpoints {
	if (point != qh GOODpointp)
	  qh_check_point (point, facet, &maxoutside, &maxdist, &errfacet1, &errfacet2);
      }
      FOREACHpoint_(qh other_points) {
	if (point != qh GOODpointp)
	  qh_check_point (point, facet, &maxoutside, &maxdist, &errfacet1, &errfacet2);
      }
    }
    if (maxdist > qh outside_err) {
      ivp_message( "qhull precision error (qh_check_points): a coplanar point is %6.2g from convex hull.  The maximum value (qh.outside_err) is %6.2g\n",
                maxdist, qh outside_err );
      qh_errexit2( qh_ERRprec, errfacet1, errfacet2 );
    }else if (errfacet1 && qh outside_err > REALmax/2)
        qh_errexit2( qh_ERRprec, errfacet1, errfacet2 );
    else if (errfacet1){
          /* the error was logged to qh.ferr but does not effect the output */
	trace0((qh ferr, "qh_check_points: max distance outside %2.2g\n", maxdist));
    }
  }
} /* check_points */


/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="checkconvex">-</a>
  
  qh_checkconvex( facetlist, fault )
    check that each ridge in facetlist is convex
    fault = qh_DATAfault if reporting errors
          = qh_ALGORITHMfault otherwise

  returns:
    counts Zconcaveridges and Zcoplanarridges
    errors if concaveridge or if merging an coplanar ridge

  note:
    if not merging, 
      tests vertices for neighboring simplicial facets
    else if ZEROcentrum, 
      tests vertices for neighboring simplicial   facets
    else 
      tests centrums of neighboring facets

  design:
    for all facets
      report flipped facets
      if ZEROcentrum and simplicial neighbors
        test vertices for neighboring simplicial facets
      else
        test centrum against all neighbors 
*/
void qh_checkconvex(facetT *facetlist, int fault) {
  facetT *facet, *neighbor, **neighborp, *errfacet1=NULL, *errfacet2=NULL;
  vertexT *vertex;
  realT dist;
  pointT *centrum;
  boolT waserror= False, tempcentrum= False, allsimplicial;
  int neighbor_i;

  trace1((qh ferr, "qh_checkconvex: check all ridges are convex\n"));
  if (!qh RERUN) {
    zzval_(Zconcaveridges)= 0;
    zzval_(Zcoplanarridges)= 0;
  }
  FORALLfacet_(facetlist) {
    if (facet->flipped) {
      qh_precision ("flipped facet");
      ivp_message( "qhull precision error: f%d is flipped (interior point is outside)\n",
	       facet->id);
      errfacet1= facet;
      waserror= True;
      continue;
    }
    if (qh MERGING && (!qh ZEROcentrum || !facet->simplicial))
      allsimplicial= False;
    else {
      allsimplicial= True;
      neighbor_i= 0;
      FOREACHneighbor_(facet) {
        vertex= SETelemt_(facet->vertices, neighbor_i++, vertexT);
	if (!neighbor->simplicial) {
	  allsimplicial= False;
	  continue;
	}
        qh_distplane (vertex->point, neighbor, &dist);
        if (dist > -qh DISTround) {
	  if (fault == qh_DATAfault) {
            qh_precision ("coplanar or concave ridge");
    	    if (qh ferr){
		ivp_message( "qhull precision error: initial simplex is not convex. Distance=%.2g\n", dist);
	    }
	    qh_errexit(qh_ERRsingular, NULL, NULL);
	  }
          if (dist > qh DISTround) {
            zzinc_(Zconcaveridges);
            qh_precision ("concave ridge");
    	    if (qh ferr){
		ivp_message( "qhull precision error: f%d is concave to f%d, since p%d (v%d) is %6.4g above\n",
		  facet->id, neighbor->id, qh_pointid(vertex->point), vertex->id, dist);
	    }
            errfacet1= facet;
            errfacet2= neighbor;
            waserror= True;
          }else if (qh ZEROcentrum) {
            if (dist > 0) {     /* qh_checkzero checks that dist < - qh DISTround */
              zzinc_(Zcoplanarridges); 
              qh_precision ("coplanar ridge");
	      if (qh ferr){
		  ivp_message( "qhull precision error: f%d is clearly not convex to f%d, since p%d (v%d) is %6.4g above\n",
		    facet->id, neighbor->id, qh_pointid(vertex->point), vertex->id, dist);
	      }
              errfacet1= facet;
              errfacet2= neighbor;
              waserror= True;
	    }
	  }else {              
            zzinc_(Zcoplanarridges);
            qh_precision ("coplanar ridge");
            trace0((qh ferr, "qhull precision error: f%d may be coplanar to f%d, since p%d (v%d) is within %6.4g during p%d\n",
              facet->id, neighbor->id, qh_pointid(vertex->point), vertex->id, dist, qh furthest_id));
          }
        }
      }
    }
    if (!allsimplicial) {
      if (qh CENTERtype == qh_AScentrum) {
        if (!facet->center)
          facet->center= qh_getcentrum (facet);
        centrum= facet->center;
      }else {
        centrum= qh_getcentrum(facet);
        tempcentrum= True;
      }
      FOREACHneighbor_(facet) {
	if (qh ZEROcentrum && facet->simplicial && neighbor->simplicial)
	  continue;
        zzinc_(Zdistconvex);
        qh_distplane (centrum, neighbor, &dist);
        if (dist > qh DISTround) {
          zzinc_(Zconcaveridges);
          qh_precision ("concave ridge");
  	  if (qh ferr){
	      ivp_message( "qhull precision error: f%d is concave to f%d.  Centrum of f%d is %6.4g above f%d\n",
		facet->id, neighbor->id, facet->id, dist, neighbor->id);
	  }
          errfacet1= facet;
          errfacet2= neighbor;
          waserror= True;
        }else if (dist >= 0.0) {   /* if arithmetic always rounds the same,
				     can test against centrum radius instead */
          zzinc_(Zcoplanarridges);
          qh_precision ("coplanar ridge");
  	  if (qh ferr){
	    ivp_message( "qhull precision error: f%d is coplanar or concave to f%d.  Centrum of f%d is %6.4g above f%d\n",
		facet->id, neighbor->id, facet->id, dist, neighbor->id);
	  }
	  errfacet1= facet;
	  errfacet2= neighbor;
	  waserror= True;
        }
      }
      if (tempcentrum)
        qh_memfree(centrum, qh normal_size);
    }
  }
  if (waserror && !qh FORCEoutput)
    qh_errexit2 (qh_ERRprec, errfacet1, errfacet2);
} /* checkconvex */


/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="checkfacet">-</a>
  
  qh_checkfacet( facet, newmerge, waserror )
    checks for consistency errors in facet
    newmerge set if from merge.c

  returns:
    sets waserror if any error occurs

  checks:
    vertex ids are inverse sorted
    unless newmerge, at least hull_dim neighbors and vertices (exactly if simplicial)
    if non-simplicial, at least as many ridges as neighbors
    neighbors are not duplicated
    ridges are not duplicated
    in 3-d, ridges=verticies
    (qh.hull_dim-1) ridge vertices
    neighbors are reciprocated
    ridge neighbors are facet neighbors and a ridge for every neighbor
    simplicial neighbors match facetintersect
    vertex intersection matches vertices of common ridges 
    vertex neighbors and facet vertices agree
    all ridges have distinct vertex sets

  notes:  
    uses neighbor->seen

  design:
    check sets
    check vertices
    check sizes of neighbors and vertices
    check for qh_MERGEridge and qh_DUPLICATEridge flags
    check neighbor set
    check ridge set
    check ridges, neighbors, and vertices
*/
void qh_checkfacet(facetT *facet, boolT newmerge, boolT *waserrorp) {
  facetT *neighbor, **neighborp, *errother=NULL;
  ridgeT *ridge, **ridgep, *errridge= NULL, *ridge2;
  vertexT *vertex, **vertexp;
  unsigned previousid= INT_MAX;
  int numneighbors, numvertices, numridges=0, numRvertices=0;
  boolT waserror= False;
  int skipA, skipB, ridge_i, ridge_n, i;
  setT *intersection;

  if (facet->visible) {
    ivp_message( "qhull internal error (qh_checkfacet): facet f%d is on the visible_list\n",
      facet->id);
    qh_errexit (qh_ERRqhull, facet, NULL);
  }
  if (!facet->normal) {
      if (qh ferr){
	ivp_message( "qhull internal error (qh_checkfacet): facet f%d does not have  a normal\n",
	  facet->id);
      }
    waserror= True;
  }
  qh_setcheck (facet->vertices, "vertices for f", facet->id);
  qh_setcheck (facet->ridges, "ridges for f", facet->id);
  qh_setcheck (facet->outsideset, "outsideset for f", facet->id);
  qh_setcheck (facet->coplanarset, "coplanarset for f", facet->id);
  qh_setcheck (facet->neighbors, "neighbors for f", facet->id);
  FOREACHvertex_(facet->vertices) {
    if (vertex->deleted) {
      ivp_message( "qhull internal error (qh_checkfacet): deleted vertex v%d in f%d\n", vertex->id, facet->id);
      qh_errprint ("ERRONEOUS", NULL, NULL, NULL, vertex);
      waserror= True;
    }
    if (vertex->id >= previousid) {
      ivp_message( "qhull internal error (qh_checkfacet): vertices of f%d are not in descending id order at v%d\n", facet->id, vertex->id);
      waserror= True;
      break;
    }
    previousid= vertex->id;
  }
  numneighbors= qh_setsize(facet->neighbors);
  numvertices= qh_setsize(facet->vertices);
  numridges= qh_setsize(facet->ridges);
  if (facet->simplicial) {
    if (numvertices+numneighbors != 2*qh hull_dim 
    && !facet->degenerate && !facet->redundant) {
      ivp_message( "qhull internal error (qh_checkfacet): for simplicial facet f%d, #vertices %d + #neighbors %d != 2*qh hull_dim\n", 
                facet->id, numvertices, numneighbors);
      qh_setprint (qh ferr, "", facet->neighbors);
      waserror= True;
    }
  }else { /* non-simplicial */
    if (!newmerge 
    &&(numvertices < qh hull_dim || numneighbors < qh hull_dim)
    && !facet->degenerate && !facet->redundant) {
      ivp_message( "qhull internal error (qh_checkfacet): for facet f%d, #vertices %d or #neighbors %d < qh hull_dim\n",
         facet->id, numvertices, numneighbors);
       waserror= True;
    }
    if (numridges < numneighbors
    ||(qh hull_dim == 3 && numvertices != numridges && !qh NEWfacets)
    ||(qh hull_dim == 2 && numridges + numvertices + numneighbors != 6)) {
      if (!facet->degenerate && !facet->redundant) {
	ivp_message( "qhull internal error (qh_checkfacet): for facet f%d, #ridges %d < #neighbors %d or (3-d) != #vertices %d or (2-d) not all 2\n",
	    facet->id, numridges, numneighbors, numvertices);
	waserror= True;
      }
    }
  }
  FOREACHneighbor_(facet) {
    if (neighbor == qh_MERGEridge || neighbor == qh_DUPLICATEridge) {
      ivp_message( "qhull internal error (qh_checkfacet): facet f%d still has a MERGE or DUP neighbor\n", facet->id);
      qh_errexit (qh_ERRqhull, facet, NULL);
    }
    neighbor->seen= True;
  }
  FOREACHneighbor_(facet) {
    if (!qh_setin(neighbor->neighbors, facet)) {
      ivp_message( "qhull internal error (qh_checkfacet): facet f%d has neighbor f%d, but f%d does not have neighbor f%d\n",
	      facet->id, neighbor->id, neighbor->id, facet->id);
      errother= neighbor;
      waserror= True;
    }
    if (!neighbor->seen) {
      ivp_message( "qhull internal error (qh_checkfacet): facet f%d has a duplicate neighbor f%d\n",
	      facet->id, neighbor->id);
      errother= neighbor;
      waserror= True;
    }    
    neighbor->seen= False;
  }
  FOREACHridge_(facet->ridges) {
    qh_setcheck (ridge->vertices, "vertices for r", ridge->id);
    ridge->seen= False;
  }
  FOREACHridge_(facet->ridges) {
    if (ridge->seen) {
      ivp_message( "qhull internal error (qh_checkfacet): facet f%d has a duplicate ridge r%d\n",
	      facet->id, ridge->id);
      errridge= ridge;
      waserror= True;
    }    
    ridge->seen= True;
    numRvertices= qh_setsize(ridge->vertices);
    if (numRvertices != qh hull_dim - 1) {
      ivp_message( "qhull internal error (qh_checkfacet): ridge between f%d and f%d has %d vertices\n", 
                ridge->top->id, ridge->bottom->id, numRvertices);
      errridge= ridge;
      waserror= True;
    }
    neighbor= otherfacet_(ridge, facet);
    neighbor->seen= True;
    if (!qh_setin(facet->neighbors, neighbor)) {
      ivp_message( "qhull internal error (qh_checkfacet): for facet f%d, neighbor f%d of ridge r%d not in facet\n",
           facet->id, neighbor->id, ridge->id);
      errridge= ridge;
      waserror= True;
    }
  }
  if (!facet->simplicial) {
    FOREACHneighbor_(facet) {
      if (!neighbor->seen) {
        ivp_message( "qhull internal error (qh_checkfacet): facet f%d does not have a ridge for neighbor f%d\n",
	      facet->id, neighbor->id);
	errother= neighbor;
        waserror= True;
      }
      intersection= qh_vertexintersect_new(facet->vertices, neighbor->vertices);
      qh_settemppush (intersection);
      FOREACHvertex_(facet->vertices) {
	vertex->seen= False;
	vertex->seen2= False;
      }
      FOREACHvertex_(intersection)
	vertex->seen= True;
      FOREACHridge_(facet->ridges) {
	if (neighbor != otherfacet_(ridge, facet))
	    continue;
	FOREACHvertex_(ridge->vertices) {
	  if (!vertex->seen) {
	    ivp_message( "qhull internal error (qh_checkfacet): vertex v%d in r%d not in f%d intersect f%d\n",
  	          vertex->id, ridge->id, facet->id, neighbor->id);
	    qh_errexit (qh_ERRqhull, facet, ridge);
	  }
	  vertex->seen2= True;
	}
      }
      if (!newmerge) {
	FOREACHvertex_(intersection) {
	  if (!vertex->seen2) {
	    if (qh IStracing >=3 || !qh MERGING) {
	      ivp_message( "qhull precision error (qh_checkfacet): vertex v%d in f%d intersect f%d but\n\
 not in a ridge.  This is ok under merging.  Last point was p%d\n",
		     vertex->id, facet->id, neighbor->id, qh furthest_id);
	      if (!qh FORCEoutput && !qh MERGING) {
		qh_errprint ("ERRONEOUS", facet, neighbor, NULL, vertex);
		if (!qh MERGING)
		  qh_errexit (qh_ERRqhull, NULL, NULL);
	      }
	    }
	  }
	}
      }      
      qh_settempfree (&intersection);
    }
  }else { /* simplicial */
    FOREACHneighbor_(facet) {
      if (neighbor->simplicial) {    
	skipA= SETindex_(facet->neighbors, neighbor);
	skipB= qh_setindex (neighbor->neighbors, facet);
	if (!qh_setequal_skip (facet->vertices, skipA, neighbor->vertices, skipB)) {
	  ivp_message( "qhull internal error (qh_checkfacet): facet f%d skip %d and neighbor f%d skip %d do not match \n",
		   facet->id, skipA, neighbor->id, skipB);
	  errother= neighbor;
	  waserror= True;
	}
      }
    }
  }
  if (qh hull_dim < 5 && (qh IStracing > 2 || qh CHECKfrequently)) {
    FOREACHridge_i_(facet->ridges) {           /* expensive */
      for (i= ridge_i+1; i < ridge_n; i++) {
	ridge2= SETelemt_(facet->ridges, i, ridgeT);
	if (qh_setequal (ridge->vertices, ridge2->vertices)) {
	  ivp_message( "qh_checkfacet: ridges r%d and r%d have the same vertices\n",
		  ridge->id, ridge2->id);
	  errridge= ridge;
	  waserror= True;
	}
      }
    }
  }
  if (waserror) {
    qh_errprint("ERRONEOUS", facet, errother, errridge, NULL);
    *waserrorp= True;
  }
} /* checkfacet */


/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="checkflipped_all">-</a>
  
  qh_checkflipped_all( facetlist )
    checks orientation of facets in list against interior point
*/
void qh_checkflipped_all (facetT *facetlist) {
  facetT *facet;
  boolT waserror= False;
  realT dist;

  if (facetlist == qh facet_list)
    zzval_(Zflippedfacets)= 0;
  FORALLfacet_(facetlist) {
    if (facet->normal && !qh_checkflipped (facet, &dist, !qh_ALL)) {
      ivp_message( "qhull precision error: facet f%d is flipped, distance= %6.12g\n",
	      facet->id, dist);
      if (!qh FORCEoutput) {
	qh_errprint("ERRONEOUS", facet, NULL, NULL, NULL);
	waserror= True;
      }
    }
  }
  if (waserror) {
    ivp_message( "\n\
A flipped facet occurs when its distance to the interior point is\n\
greater than %2.2g, the maximum roundoff error.\n", -qh DISTround);
    qh_errexit(qh_ERRprec, NULL, NULL);
  }
} /* checkflipped_all */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="checkpolygon">-</a>
  
  qh_checkpolygon( facetlist )
    checks the correctness of the structure

  notes:
    call with either qh.facet_list or qh.newfacet_list
    checks num_facets and num_vertices if qh.facet_list

  design:
    for each facet
      checks facet and outside set
    initializes vertexlist
    for each facet
      checks vertex set
    if checking all facets (qh.facetlist)
      check facet count
      if qh.VERTEXneighbors
        check vertex neighbors and count
      check vertex count
*/
void qh_checkpolygon(facetT *facetlist) {
  facetT *facet;
  vertexT *vertex, **vertexp, *vertexlist;
  int numfacets= 0, numvertices= 0, numridges= 0;
  int totvneighbors= 0, totvertices= 0;
  boolT waserror= False, nextseen= False, visibleseen= False;
  
  trace1((qh ferr, "qh_checkpolygon: check all facets from f%d\n", facetlist->id));
  if (facetlist != qh facet_list || qh ONLYgood)
    nextseen= True;
  FORALLfacet_(facetlist) {
    if (facet == qh visible_list)
      visibleseen= True;
    if (!facet->visible) {
      if (!nextseen) {
	if (facet == qh facet_next)
	  nextseen= True;
	else if (qh_setsize (facet->outsideset)
        && !(qh NARROWhull && qh CHECKfrequently)) {
	  ivp_message( "qhull internal error (qh_checkpolygon): f%d has outside set before qh facet_next\n",
		   facet->id);
	  qh_errexit (qh_ERRqhull, facet, NULL);
	}
      }
      numfacets++;
      qh_checkfacet(facet, False, &waserror);
    }
  }
  if (qh visible_list && !visibleseen && facetlist == qh facet_list) {
    ivp_message( "qhull internal error (qh_checkpolygon): visible list f%d no longer on facet list\n", qh visible_list->id);
    qh_printlists();
    qh_errexit (qh_ERRqhull, qh visible_list, NULL);
  }
  if (facetlist == qh facet_list)
    vertexlist= qh vertex_list;
  else if (facetlist == qh newfacet_list)
    vertexlist= qh newvertex_list;
  else
    vertexlist= NULL;
  FORALLvertex_(vertexlist) {
    vertex->seen= False;
    vertex->visitid= 0;
  }  
  FORALLfacet_(facetlist) {
    if (facet->visible)
      continue;
    if (facet->simplicial)
      numridges += qh hull_dim;
    else
      numridges += qh_setsize (facet->ridges);
    FOREACHvertex_(facet->vertices) {
      vertex->visitid++;
      if (!vertex->seen) {
	vertex->seen= True;
	numvertices++;
	if (qh_pointid (vertex->point) == -1) {
	  ivp_message( "qhull internal error (qh_checkpolygon): unknown point %p for vertex v%d first_point %p\n",
		   vertex->point, vertex->id, qh first_point);
	  waserror= True;
	}
      }
    }
  }
  qh vertex_visit += numfacets;
  if (facetlist == qh facet_list) {
    if (numfacets != qh num_facets - qh num_visible) {
      ivp_message( "qhull internal error (qh_checkpolygon): actual number of facets is %d, cumulative facet count is %d\n",
	      numfacets, qh num_facets- qh num_visible);
      waserror= True;
    }
    qh vertex_visit++;
    if (qh VERTEXneighbors) {
      FORALLvertices {
	qh_setcheck (vertex->neighbors, "neighbors for v", vertex->id);
	if (vertex->deleted)
	  continue;
	totvneighbors += qh_setsize (vertex->neighbors);
      }
      FORALLfacet_(facetlist)
	totvertices += qh_setsize (facet->vertices);
      if (totvneighbors != totvertices) {
	ivp_message( "qhull internal error (qh_checkpolygon): vertex neighbors inconsistent.  Totvneighbors %d, totvertices %d\n",
		totvneighbors, totvertices);
	waserror= True;
      }
    }
    if (numvertices != qh num_vertices - qh_setsize(qh del_vertices)) {
      ivp_message( "qhull internal error (qh_checkpolygon): actual number of vertices is %d, cumulative vertex count is %d\n",
	      numvertices, qh num_vertices - qh_setsize(qh del_vertices));
      waserror= True;
    }
    if (qh hull_dim == 2 && numvertices != numfacets) {
      ivp_message( "qhull internal error (qh_checkpolygon): #vertices %d != #facets %d\n",
        numvertices, numfacets);
      waserror= True;
    }
    if (qh hull_dim == 3 && numvertices + numfacets - numridges/2 != 2) {
      ivp_message( "qhull internal error (qh_checkpolygon): #vertices %d + #facets %d - #edges %d != 2\n",
        numvertices, numfacets, numridges/2);
      waserror= True;
    }
  }
  if (waserror) 
    qh_errexit(qh_ERRqhull, NULL, NULL);
} /* checkpolygon */


/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="checkvertex">-</a>
  
  qh_checkvertex( vertex )
    check vertex for consistency
    checks vertex->neighbors

  notes:
    neighbors checked efficiently in checkpolygon
*/
void qh_checkvertex (vertexT *vertex) {
  boolT waserror= False;
  facetT *neighbor, **neighborp, *errfacet=NULL;

  if (qh_pointid (vertex->point) == -1) {
    ivp_message( "qhull internal error (qh_checkvertex): unknown point id %p\n", vertex->point);
    waserror= True;
  }
  if (vertex->id >= qh vertex_id) {
    ivp_message( "qhull internal error (qh_checkvertex): unknown vertex id %d\n", vertex->id);
    waserror= True;
  }
  if (!waserror && !vertex->deleted) {
    if (qh_setsize (vertex->neighbors)) {
      FOREACHneighbor_(vertex) {
        if (!qh_setin (neighbor->vertices, vertex)) {
          ivp_message( "qhull internal error (qh_checkvertex): neighbor f%d does not contain v%d\n", neighbor->id, vertex->id);
	  errfacet= neighbor;
	  waserror= True;
	}
      }
    }
  }
  if (waserror) {
    qh_errprint ("ERRONEOUS", NULL, NULL, NULL, vertex);
    qh_errexit (qh_ERRqhull, errfacet, NULL);
  }
} /* checkvertex */
  
/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="clearcenters">-</a>
  
  qh_clearcenters( type )
    clear old data from facet->center

  notes:
    sets new centertype
    nop if CENTERtype is the same
*/
void qh_clearcenters (qh_CENTER type) {
  facetT *facet;
  
  if (qh CENTERtype != type) {
    FORALLfacets {
      if (qh CENTERtype == qh_ASvoronoi){
        if (facet->center) {
          qh_memfree (facet->center, qh center_size);
          facet->center= NULL;
        }
      }else /* qh CENTERtype == qh_AScentrum */ {
        if (facet->center) {
          qh_memfree (facet->center, qh normal_size);
	  facet->center= NULL;
        }
      }
    }
    qh CENTERtype= type;
  }
  trace2((qh ferr, "qh_clearcenters: switched to center type %d\n", type));
} /* clearcenters */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="createsimplex">-</a>
  
  qh_createsimplex( vertices )
    creates a simplex from a set of vertices

  returns:
    initializes qh.facet_list to the simplex
    initializes qh.newfacet_list, .facet_tail
    initializes qh.vertex_list, .newvertex_list, .vertex_tail

  design:
    initializes lists
    for each vertex
      create a new facet
    for each new facet
      create its neighbor set
*/
void qh_createsimplex(setT *vertices) {
  facetT *facet= NULL, *newfacet;
  boolT toporient= True;
  int vertex_i, vertex_n, nth;
  setT *newfacets= qh_settemp (qh hull_dim+1);
  vertexT *vertex;
  
  qh facet_list= qh newfacet_list= qh facet_tail= qh_newfacet();
  qh num_facets= qh num_vertices= 0;
  qh vertex_list= qh newvertex_list= qh vertex_tail= qh_newvertex(NULL);
  FOREACHvertex_i_(vertices) {
    newfacet= qh_newfacet();
    newfacet->vertices= qh_setnew_delnthsorted (vertices, vertex_n,
						vertex_i, 0);
    newfacet->toporient= toporient;
    qh_appendfacet(newfacet);
    newfacet->newfacet= True;
    qh_appendvertex (vertex);
    qh_setappend (&newfacets, newfacet);
    toporient ^= True;
  }
  FORALLnew_facets {
    nth= 0;
    FORALLfacet_(qh newfacet_list) {
      if (facet != newfacet) 
        SETelem_(newfacet->neighbors, nth++)= facet;
    }
    qh_settruncate (newfacet->neighbors, qh hull_dim);
  }
  qh_settempfree (&newfacets);
  trace1((qh ferr, "qh_createsimplex: created simplex\n"));
} /* createsimplex */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="delridge">-</a>
  
  qh_delridge( ridge )
    deletes ridge from data structures it belongs to
    frees up its memory

  notes:
    in merge.c, caller sets vertex->delridge for each vertex
    ridges also freed in qh_freeqhull
*/
void qh_delridge(ridgeT *ridge) {
  void **freelistp; /* used !qh_NOmem */
  
  qh_setdel(ridge->top->ridges, ridge);
  qh_setdel(ridge->bottom->ridges, ridge);
  qh_setfree(&(ridge->vertices));
  qh_memfree_(ridge, sizeof(ridgeT), freelistp);
} /* delridge */


/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="delvertex">-</a>
  
  qh_delvertex( vertex )
    deletes a vertex and frees its memory

  notes:
    assumes vertex->adjacencies have been updated if needed
    unlinks from vertex_list
*/
void qh_delvertex (vertexT *vertex) {

  if (vertex == qh tracevertex)
    qh tracevertex= NULL;
  qh_removevertex (vertex);
  qh_setfree (&vertex->neighbors);
  qh_memfree(vertex, sizeof(vertexT));
} /* delvertex */


/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="facet3vertex">-</a>
  
  qh_facet3vertex(  )
    return temporary set of 3-d vertices in qh_ORIENTclock order

  design:
    if simplicial facet
      build set from facet->vertices with facet->toporient
    else
      for each ridge in order
        build set from ridge's vertices
*/
setT *qh_facet3vertex (facetT *facet) {
  ridgeT *ridge, *firstridge;
  vertexT *vertex;
  int cntvertices, cntprojected=0;
  setT *vertices;

  cntvertices= qh_setsize(facet->vertices);
  vertices= qh_settemp (cntvertices);
  if (facet->simplicial) {
    if (cntvertices != 3) {
      ivp_message( "qhull internal error (qh_facet3vertex): only %d vertices for simplicial facet f%d\n", 
                  cntvertices, facet->id);
      qh_errexit(qh_ERRqhull, facet, NULL);
    }
    qh_setappend (&vertices, SETfirst_(facet->vertices));
    if (facet->toporient ^ qh_ORIENTclock)
      qh_setappend (&vertices, SETsecond_(facet->vertices));
    else
      qh_setaddnth (&vertices, 0, SETsecond_(facet->vertices));
    qh_setappend (&vertices, SETelem_(facet->vertices, 2));
  }else {
    ridge= firstridge= SETfirstt_(facet->ridges, ridgeT);   /* no infinite */
    while ((ridge= qh_nextridge3d (ridge, facet, &vertex))) {
      qh_setappend (&vertices, vertex);
      if (++cntprojected > cntvertices || ridge == firstridge)
        break;
    }
    if (!ridge || cntprojected != cntvertices) {
      ivp_message( "qhull internal error (qh_facet3vertex): ridges for facet %d don't match up.  got at least %d\n", 
                  facet->id, cntprojected);
      qh_errexit(qh_ERRqhull, facet, ridge);
    }
  }
  return vertices;
} /* facet3vertex */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="findbestfacet">-</a>
  
  qh_findbestfacet( point, bestoutside, bestdist, isoutside )
    find facet that is furthest below a point 

    for Delaunay triangulations, 
      Use qh_setdelaunay() to lift point to paraboloid and scale by 'Qbb' if needed
      Do not use options 'Qbk', 'QBk', or 'QbB' since they scale the coordinates. 

  returns:
    if bestoutside is set (e.g., qh_ALL)
      returns best facet that is not upperdelaunay
      if Delaunay and inside, point is outside circumsphere of bestfacet
    else
      returns first facet below point
      if point is inside, returns nearest, !upperdelaunay facet
    distance to facet
    isoutside set if outside of facet
    
  notes:
    this works for all distributions
    if inside, qh_findbestfacet performed an exhaustive search
    qh_findbestfacet is not used by qhull.
    uses qh.visit_id, qh.searchset
    
  see:
    <a href="geom.c#findbest">qh_findbest</a>
*/
facetT *qh_findbestfacet (pointT *point, boolT bestoutside,
           realT *bestdist, boolT *isoutside) {
  facetT *bestfacet= NULL;
  int numpart, totpart= 0;
  
  bestfacet= qh_findbest (point, qh facet_list, 
			    bestoutside, False, bestoutside,
			    bestdist, isoutside, &totpart);
  if (!bestfacet) {
    ivp_message( "qh_findbestfacet: all facets are flipped or upper Delaunay\n");
    qh_errexit (qh_ERRqhull, NULL, NULL);
  }
  if (*bestdist < -qh DISTround) {
    bestfacet= qh_findfacet_all (point, bestdist, isoutside, &numpart);
    totpart += numpart;
    if ((isoutside && bestoutside)
    || (!isoutside && bestfacet->upperdelaunay)) {
      bestfacet= qh_findbest (point, bestfacet, 
			    bestoutside, False, bestoutside,
			    bestdist, isoutside, &totpart);
      totpart += numpart;
    }
  }
  trace3((qh ferr, "qh_findbestfacet: f%d dist %2.2g isoutside %d totpart %d\n",
	  bestfacet->id, *bestdist, *isoutside, totpart));
  return bestfacet;
} /* findbestfacet */ 
 
/*-<a                             href="qh-c.htm"
  >-------------------------------</a><a name="findfacet_all">-</a>
  
  qh_findfacet_all( point, bestdist, isoutside, numpart )
    exhaustive search for facet below a point 

    for Delaunay triangulations, 
      Use qh_setdelaunay() to lift point to paraboloid and scale by 'Qbb' if needed
      Do not use options 'Qbk', 'QBk', or 'QbB' since they scale the coordinates. 

  returns:
    returns first facet below point
    if point is inside, 
      returns nearest facet
    distance to facet
    isoutside if point is outside of the hull
    number of distance tests
*/
facetT *qh_findfacet_all (pointT *point, realT *bestdist, boolT *isoutside,
			  int *numpart) {
  facetT *bestfacet= NULL, *facet;
  realT dist;
  int totpart= 0;
  
  *bestdist= REALmin;
  *isoutside= False;
  FORALLfacets {
    if (facet->flipped || !facet->normal)
      continue;
    totpart++;
    qh_distplane (point, facet, &dist);
    if (dist > *bestdist) {
      *bestdist= dist;
      bestfacet= facet;
      if (dist > qh MINoutside) {
        *isoutside= True;
        break;
      }
    }
  }
  *numpart= totpart;
  trace3((qh ferr, "qh_findfacet_all: f%d dist %2.2g isoutside %d totpart %d\n",
	  getid_(bestfacet), *bestdist, *isoutside, totpart));
  return bestfacet;
} /* findfacet_all */ 
 
/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="findgood">-</a>
  
  qh_findgood( facetlist, goodhorizon )
    identify good facets for qh.PRINTgood
    if qh.GOODvertex>0
      facet includes point as vertex
      if !match, returns goodhorizon
      inactive if qh.MERGING
    if qh.GOODpoint
      facet is visible or coplanar (>0) or not visible (<0) 
    if qh.GOODthreshold
      facet->normal matches threshold
    if !goodhorizon and !match, 
      selects facet with closest angle
      sets GOODclosest
      
  returns:
    number of new, good facets found
    determines facet->good
    may update qh.GOODclosest
    
  notes:
    qh_findgood_all further reduces the good region

  design:
    count good facets
    mark good facets for qh.GOODpoint  
    mark good facets for qh.GOODthreshold
    if necessary
      update qh.GOODclosest  
*/
int qh_findgood (facetT *facetlist, int goodhorizon) {
  facetT *facet, *bestfacet= NULL;
  realT angle, bestangle= REALmax, dist;
  int  numgood=0;

  FORALLfacet_(facetlist) {
    if (facet->good)
      numgood++;
  }
  if (qh GOODvertex>0 && !qh MERGING) {
    FORALLfacet_(facetlist) {
      if (!qh_isvertex (qh GOODvertexp, facet->vertices)) {
        facet->good= False;
        numgood--;
      }
    }
  }
  if (qh GOODpoint && numgood) {
    FORALLfacet_(facetlist) {
      if (facet->good && facet->normal) {
        zinc_(Zdistgood);
        qh_distplane (qh GOODpointp, facet, &dist);
        if ((qh GOODpoint > 0) ^ (dist > 0.0)) {
          facet->good= False;
          numgood--;
        }
      }
    }
  }
  if (qh GOODthreshold && (numgood || goodhorizon || qh GOODclosest)) {
    FORALLfacet_(facetlist) {
      if (facet->good && facet->normal) {
        if (!qh_inthresholds (facet->normal, &angle)) {
          facet->good= False;
          numgood--;
          if (angle < bestangle) {
            bestangle= angle;
            bestfacet= facet;
          }
        }
      }
    }
    if (!numgood && (!goodhorizon || qh GOODclosest)) {
      if (qh GOODclosest) {
	if (qh GOODclosest->visible)
	  qh GOODclosest= NULL;
	else {
	  qh_inthresholds (qh GOODclosest->normal, &angle);
	  if (angle < bestangle)
	    bestfacet= qh GOODclosest;
	}
      }
      if (bestfacet && bestfacet != qh GOODclosest) {
	if (qh GOODclosest)
	  qh GOODclosest->good= False;
	qh GOODclosest= bestfacet;
	bestfacet->good= True;
	numgood++;
	trace2((qh ferr, "qh_findgood: f%d is closest (%2.2g) to thresholds\n", 
           bestfacet->id, bestangle));
	return numgood;
      }
    }else if (qh GOODclosest) { /* numgood > 0 */
      qh GOODclosest->good= False;
      qh GOODclosest= NULL;
    }
  }
  zadd_(Zgoodfacet, numgood);
  trace2((qh ferr, "qh_findgood: found %d good facets with %d good horizon\n",
               numgood, goodhorizon));
  if (!numgood && qh GOODvertex>0 && !qh MERGING) 
    return goodhorizon;
  return numgood;
} /* findgood */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="findgood_all">-</a>
  
  qh_findgood_all( facetlist )
    apply other constraints for good facets (used by qh.PRINTgood)
    if qh.GOODvertex 
      facet includes (>0) or doesn't include (<0) point as vertex
      if last good facet, prints warning and continues
    if qh.SPLITthresholds
      facet->normal matches threshold, or if none, the closest one
    calls qh_findgood
    nop if good not used

  returns:
    clears facet->good if not good
    sets qh.num_good

  notes:
    this is like qh_findgood but more restrictive

  design:
    uses qh_findgood to mark good facets
    marks facets for qh.GOODvertex
    marks facets for qh.SPLITthreholds  
*/
void qh_findgood_all (facetT *facetlist) {
  facetT *facet, *bestfacet=NULL;
  realT angle, bestangle= REALmax;
  int  numgood=0, startgood;

  if (!qh GOODvertex && !qh GOODthreshold && !qh GOODpoint 
  && !qh SPLITthresholds)
    return;
  if (!qh ONLYgood)
    qh_findgood (qh facet_list, 0);
  FORALLfacet_(facetlist) {
    if (facet->good)
      numgood++;
  }
  if (qh GOODvertex <0 || (qh GOODvertex > 0 && qh MERGING)) {
    FORALLfacet_(facetlist) {
      if (facet->good && ((qh GOODvertex > 0) ^ !!qh_isvertex (qh GOODvertexp, facet->vertices))) {
        if (!--numgood) {
          ivp_message( "qhull warning: good vertex p%d does not match last good facet f%d.  Ignored.\n",
             qh_pointid(qh GOODvertexp), facet->id);
          return;
        }
        facet->good= False;
      }
    }
  }
  startgood= numgood;
  if (qh SPLITthresholds) {
    FORALLfacet_(facetlist) {
      if (facet->good) {
        if (!qh_inthresholds (facet->normal, &angle)) {
          facet->good= False;
          numgood--;
          if (angle < bestangle) {
            bestangle= angle;
            bestfacet= facet;
          }
        }
      }
    }
    if (!numgood && bestfacet) {
      bestfacet->good= True;
      numgood++;
      trace0((qh ferr, "qh_findgood_all: f%d is closest (%2.2g) to thresholds\n", 
           bestfacet->id, bestangle));
      return;
    }
  }
  qh num_good= numgood;
  trace0((qh ferr, "qh_findgood_all: %d good facets remain out of %d facets\n",
        numgood, startgood));
} /* findgood_all */

/*-<a                             href="qh-c.htm"
  >-------------------------------</a><a name="furthestnext">-</a>
  
  qh_furthestnext()
    set qh.facet_next to facet with furthest of all furthest points
    searches all facets on qh.facet_list

  notes:
    this may help avoid precision problems
*/
void qh_furthestnext (void /* qh facet_list */) {
  facetT *facet, *bestfacet= NULL;
  realT dist, bestdist= -REALmax;

  FORALLfacets {
    if (facet->outsideset) {
#if qh_COMPUTEfurthest
      pointT *furthest;
      furthest= (pointT*)qh_setlast (facet->outsideset);
      zinc_(Zcomputefurthest);
      qh_distplane (furthest, facet, &dist);
#else
      dist= facet->furthestdist;
#endif
      if (dist > bestdist) {
	bestfacet= facet;
	bestdist= dist;
      }
    }
  }
  if (bestfacet) {
    qh_removefacet (bestfacet);
    qh_prependfacet (bestfacet, &qh facet_next);
    trace1((qh ferr, "qh_furthestnext: made f%d next facet (dist %.2g)\n",
	    bestfacet->id, bestdist));
  }
} /* furthestnext */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="furthestout">-</a>
  
  qh_furthestout( facet )
    make furthest outside point the last point of outsideset

  returns:
    updates facet->outsideset
    clears facet->notfurthest
    sets facet->furthestdist

  design:
    determine best point of outsideset
    make it the last point of outsideset
*/
void qh_furthestout (facetT *facet) {
  pointT *point, **pointp, *bestpoint= NULL;
  realT dist, bestdist= -REALmax;

  FOREACHpoint_(facet->outsideset) {
    qh_distplane (point, facet, &dist);
    zinc_(Zcomputefurthest);
    if (dist > bestdist) {
      bestpoint= point;
      bestdist= dist;
    }
  }
  if (bestpoint) {
    qh_setdel (facet->outsideset, point);
    qh_setappend (&facet->outsideset, point);
#if !qh_COMPUTEfurthest
    facet->furthestdist= bestdist;
#endif
  }
  facet->notfurthest= False;
  trace3((qh ferr, "qh_furthestout: p%d is furthest outside point of f%d\n",
	  qh_pointid (point), facet->id));
} /* furthestout */


/*-<a                             href="qh-c.htm#qhull"
  >-------------------------------</a><a name="infiniteloop">-</a>
  
  qh_infiniteloop( facet )
    report infinite loop error due to facet
*/
void qh_infiniteloop (facetT *facet) {

  ivp_message( "qhull internal error (qh_infiniteloop): potential infinite loop detected\n");
  qh_errexit (qh_ERRqhull, facet, NULL);
} /* qh_infiniteloop */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="initbuild">-</a>
  
  qh_initbuild()
    initialize hull and outside sets with point array
    qh.FIRSTpoint/qh.NUMpoints is point array
    if qh.GOODpoint
      adds qh.GOODpoint to initial hull

  returns:
    qh_facetlist with initial hull
    points partioned into outside sets, coplanar sets, or inside
    initializes qh.GOODpointp, qh.GOODvertexp,

  design:
    initialize global variables used during qh_buildhull
    determine precision constants and points with max/min coordinate values
      if qh.SCALElast, scale last coordinate (for 'd')
    build initial simplex
    partition input points into facets of initial simplex
    set up lists
    if qh.ONLYgood
      check consistency  
      add qh.GOODvertex if defined
*/
void qh_initbuild( void) {
  setT *maxpoints, *vertices;
  facetT *facet;
  int i, numpart;
  realT dist;
  boolT isoutside;

  qh furthest_id= -1;
  qh lastreport= 0;
  qh facet_id= qh vertex_id= qh ridge_id= 0;
  qh visit_id= qh vertex_visit= 0;
  qh maxoutdone= False;

  if (qh GOODpoint > 0) 
    qh GOODpointp= qh_point (qh GOODpoint-1);
  else if (qh GOODpoint < 0) 
    qh GOODpointp= qh_point (-qh GOODpoint-1);
  if (qh GOODvertex > 0)
    qh GOODvertexp= qh_point (qh GOODvertex-1);
  else if (qh GOODvertex < 0) 
    qh GOODvertexp= qh_point (-qh GOODvertex-1);
  if ((qh GOODpoint  
       && (qh GOODpointp < qh first_point  /* also catches !GOODpointp */
	   || qh GOODpointp > qh_point (qh num_points-1)))
    || (qh GOODvertex
	&& (qh GOODvertexp < qh first_point  /* also catches !GOODvertexp */
	    || qh GOODvertexp > qh_point (qh num_points-1)))) {
    ivp_message( "qhull input error: either QGn or QVn point is > p%d\n",
	     qh num_points-1);
    qh_errexit (qh_ERRinput, NULL, NULL);
  }
  maxpoints= qh_maxmin(qh first_point, qh num_points, qh hull_dim);
  if (qh SCALElast)
    qh_scalelast (qh first_point, qh num_points, qh hull_dim,
               qh MINlastcoord, qh MAXlastcoord, qh MAXwidth);
  qh_detroundoff();
  if (qh DELAUNAY && qh upper_threshold[qh hull_dim-1] > REALmax/2
                  && qh lower_threshold[qh hull_dim-1] < -REALmax/2) {
    for (i= qh_PRINTEND; i--; ) {
      if (qh PRINTout[i] == qh_PRINTgeom && qh DROPdim < 0 
 	  && !qh GOODthreshold && !qh SPLITthresholds)
	break;  /* in this case, don't set upper_threshold */
    }
    if (i < 0) {
      if (qh UPPERdelaunay) { /* matches qh.upperdelaunay in qh_setfacetplane */
	qh lower_threshold[qh hull_dim-1]= qh ANGLEround * qh_ZEROdelaunay;
	qh GOODthreshold= True;
      }else { 
	qh upper_threshold[qh hull_dim-1]= -qh ANGLEround * qh_ZEROdelaunay;
        if (!qh GOODthreshold) 
	  qh SPLITthresholds= True; /* build upper-convex hull even if Qg */
          /* qh_initqhull_globals errors if Qg without Pdk/etc. */
      }
    }
  }
  vertices= qh_initialvertices(qh hull_dim, maxpoints, qh first_point, qh num_points); 
  qh_initialhull (vertices);  /* initial qh facet_list */
  qh_partitionall (vertices, qh first_point, qh num_points);
  if (qh PRINToptions1st || qh TRACElevel || qh IStracing) {
    if (qh TRACElevel || qh IStracing)
      ivp_message( "\nTrace level %d for %s | %s\n", 
         qh IStracing ? qh IStracing : qh TRACElevel, qh rbox_command, qh qhull_command);
    ivp_message( "Options selected for qhull %s:\n%s\n", qh_version, qh qhull_options);
  }
  qh_resetlists (False /*qh visible_list newvertex_list newfacet_list */);
  qh facet_next= qh facet_list;
  qh_furthestnext (/* qh facet_list */);
  if (qh PREmerge) {
    qh cos_max= qh premerge_cos;
    qh centrum_radius= qh premerge_centrum;
  }
  if (qh ONLYgood) {
    if (qh GOODvertex > 0 && qh MERGING) {
      ivp_message( "qhull input error: 'Qg QVn' (only good vertex) does not work with merging.\nUse 'QJ' to joggle the input or 'Q0' to turn off merging.\n");
      qh_errexit (qh_ERRinput, NULL, NULL);
    }
    if (!(qh GOODthreshold || qh GOODpoint
         || (!qh MERGEexact && !qh PREmerge && qh GOODvertexp))) {
      ivp_message( "qhull input error: 'Qg' (ONLYgood) needs a good threshold ('Pd0D0'), a\n\
good point (QGn or QG-n), or a good vertex with 'QJ' or 'Q0' (QVn).\n");
      qh_errexit (qh_ERRinput, NULL, NULL);
    }
    if (qh GOODvertex > 0  && !qh MERGING  /* matches qh_partitionall */
	&& !qh_isvertex (qh GOODvertexp, vertices)) {
      facet= qh_findbestnew (qh GOODvertexp, qh facet_list, 
			  &dist, &isoutside, &numpart);
      zadd_(Zdistgood, numpart);
      if (!isoutside) {
        ivp_message( "qhull input error: point for QV%d is inside initial simplex.  It can not be made a vertex.\n",
	       qh_pointid(qh GOODvertexp));
        qh_errexit (qh_ERRinput, NULL, NULL);
      }
      if (!qh_addpoint (qh GOODvertexp, facet, False)) {
	qh_settempfree(&vertices);
	qh_settempfree(&maxpoints);
	return;
      }
    }
    qh_findgood (qh facet_list, 0);
  }
  qh_settempfree(&vertices);
  qh_settempfree(&maxpoints);
  trace1((qh ferr, "qh_initbuild: initial hull created and points partitioned\n"));
} /* initbuild */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="initialhull">-</a>
  
  qh_initialhull( vertices )
    constructs the initial hull as a DIM3 simplex of vertices

  design:
    creates a simplex (initializes lists)
    determines orientation of simplex
    sets hyperplanes for facets
    doubles checks orientation (in case of axis-parallel facets with Gaussian elimination)
    checks for flipped facets and qh.NARROWhull
    checks the result   
*/
void qh_initialhull(setT *vertices) {
  facetT *facet, *firstfacet, *neighbor, **neighborp;
  realT dist, angle, minangle= REALmax;
#ifndef qh_NOtrace
  int k;
#endif

  qh_createsimplex(vertices);  /* qh facet_list */
  qh_resetlists (False);
  qh facet_next= qh facet_list;      /* advance facet when processed */
  qh interior_point= qh_getcenter(vertices);
  firstfacet= qh facet_list;
  qh_setfacetplane(firstfacet);
  zinc_(Znumvisibility); /* needs to be in printsummary */
  qh_distplane(qh interior_point, firstfacet, &dist);
  if (dist > 0) {  
    FORALLfacets
      facet->toporient ^= True;
  }
  FORALLfacets
    qh_setfacetplane(facet);
  FORALLfacets {
    if (!qh_checkflipped (facet, NULL, qh_ALL)) {/* due to axis-parallel facet */
      trace1((qh ferr, "qh_initialhull: initial orientation incorrect.  Correct all facets\n"));
      facet->flipped= False;
      FORALLfacets {
	facet->toporient ^= True;
	qh_orientoutside (facet);
      }
      break;
    }
  }
  FORALLfacets {
    if (!qh_checkflipped (facet, NULL, !qh_ALL)) {  /* can happen with 'R0.1' */
      qh_precision ("initial facet is coplanar with interior point");
      if (qh ferr) {ivp_message( "qhull precision error: initial facet %d is coplanar with the interior point\n",
                   facet->id);
	  }
      qh_errexit (qh_ERRsingular, facet, NULL);
    }
    FOREACHneighbor_(facet) {
      angle= qh_getangle (facet->normal, neighbor->normal);
      minimize_( minangle, angle);
    }
  }
  if (minangle < qh_MAXnarrow) {
    realT diff= 1.0 + minangle;

    qh NARROWhull= True;
    qh_option ("_narrow-hull", NULL, &diff);
    if (minangle < qh_WARNnarrow && !qh RERUN && qh PRINTprecision)
      ivp_message( "qhull precision warning: \n\
The initial hull is narrow (the cosine of the minimum angle is %.9g).\n\
A coplanar point may lead to a wide facet.  Options 'Qs' (search for best\n\
initial hull), 'QbB' (scale to unit box), or 'Qbb' (scale last coordinate)\n\
may remove this warning.  Use 'Pp' to ignore this warning.\n\
See 'Limitations' in qh-impre.htm.\n",
          minangle);
  }
  zzval_(Zprocessed)= qh hull_dim+1;
  qh_checkpolygon (qh facet_list);
  qh_checkconvex(qh facet_list,   qh_DATAfault);
#ifndef qh_NOtrace
  if (qh IStracing >= 1) {
    ivp_message( "qh_initialhull: simplex constructed, interior point:");
    for (k=0; k < qh hull_dim; k++) 
      ivp_message( " %6.4g", qh interior_point[k]);
    ivp_message( "\n");
  }
#endif
} /* initialhull */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="initialvertices">-</a>
  
  qh_initialvertices( dim, maxpoints, points, numpoints )
    determines a non-singular set of initial vertices
    maxpoints may include duplicate points

  returns:
    temporary set of dim+1 vertices in descending order by vertex id
    if qh.RANDOMoutside && !qh.ALLpoints
      picks random points
    if dim >= qh_INITIALmax, 
      uses min/max x and max points with non-zero determinants

  notes:
    unless qh.ALLpoints, 
      uses maxpoints as long as determinate is non-zero
*/
setT *qh_initialvertices(int dim, setT *maxpoints, pointT *points, int numpoints) {
  pointT *point, **pointp;
  setT *vertices, *simplex, *tested;
  realT randr;
  int index, point_i, point_n, k;
  boolT nearzero= False;
  
  vertices= qh_settemp (dim + 1);
  simplex= qh_settemp (dim+1);
  if (qh ALLpoints) 
    qh_maxsimplex (dim, NULL, points, numpoints, &simplex);
  else if (qh RANDOMoutside) {
    while (qh_setsize (simplex) != dim+1) {
      randr= qh_RANDOMint;
      randr= randr/(qh_RANDOMmax+1);
      index= (int)floor(qh num_points * randr);
      while (qh_setin (simplex, qh_point (index))) {
	index++; /* in case qh_RANDOMint always returns the same value */
        index= index < qh num_points ? index : 0;
      }
      qh_setappend (&simplex, qh_point (index));
    }
  }else if (qh hull_dim >= qh_INITIALmax) {
    tested= qh_settemp (dim+1);
    qh_setappend (&simplex, SETfirst_(maxpoints));   /* max and min X coord */
    qh_setappend (&simplex, SETsecond_(maxpoints));
    qh_maxsimplex (fmin_(qh_INITIALsearch, dim), maxpoints, points, numpoints, &simplex);
    k= qh_setsize (simplex);
    FOREACHpoint_i_(maxpoints) { 
      if (point_i & 0x1) {     /* first pick up max. coord. points */
      	if (!qh_setin (simplex, point) && !qh_setin (tested, point)){
	  qh_detsimplex(point, simplex, k, &nearzero);
          if (nearzero)
            qh_setappend (&tested, point);
          else {
            qh_setappend (&simplex, point);
            if (++k == dim)  /* use search for last point */
	      break;
	  }
	}
      }
    }
    while (k != dim && (point= (pointT*)qh_setdellast (maxpoints))) {
      if (!qh_setin (simplex, point) && !qh_setin (tested, point)){
        qh_detsimplex (point, simplex, k, &nearzero);
        if (nearzero)
          qh_setappend (&tested, point);
        else {
          qh_setappend (&simplex, point);
          k++;
	}
      }
    }
    index= 0;
    while (k != dim && (point= qh_point (index++))) {
      if (!qh_setin (simplex, point) && !qh_setin (tested, point)){
        qh_detsimplex (point, simplex, k, &nearzero);
        if (!nearzero){
          qh_setappend (&simplex, point);
          k++;
	}
      }
    }
    qh_settempfree (&tested);
    qh_maxsimplex (dim, maxpoints, points, numpoints, &simplex);
  }else
    qh_maxsimplex (dim, maxpoints, points, numpoints, &simplex);
  FOREACHpoint_(simplex) 
    qh_setaddnth (&vertices, 0, qh_newvertex(point)); /* descending order */
  qh_settempfree (&simplex);
  return vertices;
} /* initialvertices */


/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="isvertex">-</a>
  
  qh_isvertex(  )
    returns vertex if point is in vertex set, else returns NULL

  notes:
    for qh.GOODvertex
*/
vertexT *qh_isvertex (pointT *point, setT *vertices) {
  vertexT *vertex, **vertexp;

  FOREACHvertex_(vertices) {
    if (vertex->point == point)
      return vertex;
  }
  return NULL;
} /* isvertex */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="makenewfacets">-</a>
  
  qh_makenewfacets( point )
    make new facets from point and qh.visible_list

  returns:
    qh.newfacet_list= list of new facets with hyperplanes and ->newfacet
    qh.newvertex_list= list of vertices in new facets with ->newlist set
    
    if (qh.ONLYgood)
      newfacets reference horizon facets, but not vice versa
      ridges reference non-simplicial horizon ridges, but not vice versa
      does not change existing facets
    else
      sets qh.NEWfacets
      new facets attached to horizon facets and ridges
      for visible facets, 
        visible->r.replace is corresponding new facet

  design:
    for each visible facet
      make new facets to its horizon facets
      update its f.replace 
      clear its neighbor set
*/
vertexT *qh_makenewfacets (pointT *point /*visible_list*/) {
  facetT *visible, *newfacet= NULL, *newfacet2= NULL, *neighbor, **neighborp;
  vertexT *apex;
  int numnew=0;

  qh newfacet_list= qh facet_tail;
  qh newvertex_list= qh vertex_tail;
  apex= qh_newvertex(point);
  qh_appendvertex (apex);  
  qh visit_id++;
  if (!qh ONLYgood)
    qh NEWfacets= True;
  FORALLvisible_facets {
    FOREACHneighbor_(visible) 
      neighbor->seen= False;
    if (visible->ridges) {
      visible->visitid= qh visit_id;
      newfacet2= qh_makenew_nonsimplicial (visible, apex, &numnew);
    }
    if (visible->simplicial)
      newfacet= qh_makenew_simplicial (visible, apex, &numnew);
    if (!qh ONLYgood) {
      if (newfacet2)  /* newfacet is null if all ridges defined */
        newfacet= newfacet2;
      if (newfacet)
      	visible->f.replace= newfacet;
      else
        zinc_(Zinsidevisible);
      SETfirst_(visible->neighbors)= NULL;
    }
  }
  trace1((qh ferr, "qh_makenewfacets: created %d new facets from point p%d to horizon\n",
	  numnew, qh_pointid(point)));
  if (qh IStracing >= 4)
    qh_printfacetlist (qh newfacet_list, NULL, qh_ALL);
  return apex;
} /* makenewfacets */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="matchduplicates">-</a>
  
  qh_matchduplicates( atfacet, atskip, hashsize, hashcount )
    match duplicate ridges in qh.hash_table for atfacet/atskip
    duplicates marked with ->dupridge and qh_DUPLICATEridge

  returns:
    picks match with worst merge (min distance apart)
    updates hashcount
  
  see also:
    qh_matchneighbor

  design:
    compute hash value for atfacet and atskip
    repeat twice -- once to make best matches, once to match the rest
      for each possible facet in qh.hash_table
        if it is a matching facet and pass 2
          make match marked for merging (qh_MERGEridge)
        if it is a matching facet and pass 1
          test if this is a better match
      if pass 1,
        make best match (it will not be merged)
*/
#ifndef qh_NOmerge
void qh_matchduplicates (facetT *atfacet, int atskip, int hashsize, int *hashcount) {
  boolT same, ismatch;
  int hash, scan;
  facetT *facet, *newfacet, *maxmatch= NULL, *maxmatch2= NULL, *nextfacet;
  int skip, newskip, nextskip= 0, maxskip= 0, maxskip2= 0, makematch;
  realT maxdist= -REALmax, mindist, dist2, low, high;

  hash= (int)qh_gethash (hashsize, atfacet->vertices, qh hull_dim, 1, 
                     SETelem_(atfacet->vertices, atskip));
  trace2((qh ferr, "qh_matchduplicates: find duplicate matches for f%d skip %d hash %d hashcount %d\n",
	  atfacet->id, atskip, hash, *hashcount));
  for (makematch= 0; makematch < 2; makematch++) {
    qh visit_id++;
    for (newfacet= atfacet, newskip= atskip; newfacet; newfacet= nextfacet, newskip= nextskip) {
      zinc_(Zhashlookup);
      nextfacet= NULL;
      newfacet->visitid= qh visit_id;
      for (scan= hash; (facet= SETelemt_(qh hash_table, scan, facetT)); 
	   scan= (++scan >= hashsize ? 0 : scan)) {
	if (!facet->dupridge || facet->visitid == qh visit_id)
	  continue;
	zinc_(Zhashtests);
	if (qh_matchvertices (1, newfacet->vertices, newskip, facet->vertices, &skip, &same)) {
	  ismatch= (same == (newfacet->toporient ^ facet->toporient));
	  if (SETelemt_(facet->neighbors, skip, facetT) != qh_DUPLICATEridge) {
	    if (!makematch) {
	      ivp_message( "qhull internal error (qh_matchduplicates): missing dupridge at f%d skip %d for new f%d skip %d hash %d\n",
		     facet->id, skip, newfacet->id, newskip, hash);
	      qh_errexit2 (qh_ERRqhull, facet, newfacet);
	    }
	  }else if (ismatch && makematch) {
	    if (SETelemt_(newfacet->neighbors, newskip, facetT) == qh_DUPLICATEridge) {
	      SETelem_(facet->neighbors, skip)= newfacet;
	      SETelem_(newfacet->neighbors, newskip)= qh_MERGEridge;
	      *hashcount -= 2; /* removed two unmatched facets */
	      trace4((qh ferr, "qh_matchduplicates: duplicate f%d skip %d matched with new f%d skip %d merge\n",
		    facet->id, skip, newfacet->id, newskip));
	    }
	  }else if (ismatch) {
	    mindist= qh_getdistance (facet, newfacet, &low, &high);
	    dist2= qh_getdistance (newfacet, facet, &low, &high);
	    minimize_(mindist, dist2);
	    if (mindist > maxdist) {
	      maxdist= mindist;
	      maxmatch= facet;
	      maxskip= skip;
	      maxmatch2= newfacet;
	      maxskip2= newskip;
	    }
	    trace3((qh ferr, "qh_matchduplicates: duplicate f%d skip %d new f%d skip %d at dist %2.2g, max is now f%d f%d\n",
		    facet->id, skip, newfacet->id, newskip, mindist, 
		    maxmatch->id, maxmatch2->id));
	  }else { /* !ismatch */
	    nextfacet= facet;
	    nextskip= skip;
	  }
	}
	if (makematch && !facet 
        && SETelemt_(facet->neighbors, skip, facetT) == qh_DUPLICATEridge) {
	  ivp_message( "qhull internal error (qh_matchduplicates): no MERGEridge match for duplicate f%d skip %d at hash %d\n",
		     newfacet->id, newskip, hash);
	  qh_errexit (qh_ERRqhull, newfacet, NULL);
	}
      }
    } /* end of for each new facet at hash */
    if (!makematch) {
      if (!maxmatch) {
	ivp_message( "qhull internal error (qh_matchduplicates): no maximum match at duplicate f%d skip %d at hash %d\n",
		     atfacet->id, atskip, hash);
	qh_errexit (qh_ERRqhull, atfacet, NULL);
      }
      SETelem_(maxmatch->neighbors, maxskip)= maxmatch2;
      SETelem_(maxmatch2->neighbors, maxskip2)= maxmatch;
      *hashcount -= 2; /* removed two unmatched facets */
      zzinc_(Zmultiridge);
      trace0((qh ferr, "qh_matchduplicates: duplicate f%d skip %d matched with new f%d skip %d keep\n",
	      maxmatch->id, maxskip, maxmatch2->id, maxskip2));
      qh_precision ("ridge with multiple neighbors");
      if (qh IStracing >= 4)
	qh_errprint ("DUPLICATED/MATCH", maxmatch, maxmatch2, NULL, NULL);
    }
  }
} /* matchduplicates */

/*-<a                             href="qh-c.htm#nearcoplanar"
  >-------------------------------</a><a name="nearcoplanar">-</a>
  
  qh_nearcoplanar()
    for all facets, remove near-inside points from facet->coplanarset</li>
    coplanar points defined by innerplane from qh_outerinner()

  returns:
    if qh KEEPcoplanar && !qh KEEPinside
      facet->coplanarset only contains coplanar points
    if qh.JOGGLEmax
      drops inner plane by another qh.JOGGLEmax diagonal since a
        vertex could shift out while a coplanar point shifts in
  
  notes:
    used for qh.PREmerge and qh.JOGGLEmax
    must agree with computation of qh.NEARcoplanar in qh_detroundoff()
  design:
    if not keeping coplanar or inside points
      free all coplanar sets
    else if not keeping both coplanar and inside points
      remove !coplanar or !inside points from coplanar sets
*/
void qh_nearcoplanar ( void /* qh.facet_list */) {
  facetT *facet;
  pointT *point, **pointp;
  int numpart;
  realT dist, innerplane;

  if (!qh KEEPcoplanar && !qh KEEPinside) {
    FORALLfacets {
      if (facet->coplanarset) 
        qh_setfree( &facet->coplanarset);
    }
  }else if (!qh KEEPcoplanar || !qh KEEPinside) {
    qh_outerinner (NULL, NULL, &innerplane);
    if (qh JOGGLEmax < REALmax/2)
      innerplane -= qh JOGGLEmax * sqrt (qh hull_dim);
    numpart= 0;
    FORALLfacets { 
      if (facet->coplanarset) {
        FOREACHpoint_(facet->coplanarset) {
          numpart++;
	  qh_distplane (point, facet, &dist); 
  	  if (dist < innerplane) {
	    if (!qh KEEPinside)
              SETref_(point)= NULL;
          }else if (!qh KEEPcoplanar)
            SETref_(point)= NULL;
        }
	qh_setcompact (facet->coplanarset);
      }
    }
    zadd_(Zcheckpart, numpart);
  }
} /* nearcoplanar */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="nearvertex">-</a>
  
  qh_nearvertex( facet, point, bestdist )
    return nearest vertex in facet to point

  returns:
    vertex and its distance
    
  notes:
    if qh.DELAUNAY
      distance is measured in the input set
*/
vertexT *qh_nearvertex (facetT *facet, pointT *point, realT *bestdistp) {
  realT bestdist= REALmax, dist;
  vertexT *bestvertex= NULL, *vertex, **vertexp;
  int dim= qh hull_dim;

  if (qh DELAUNAY)
    dim--;
  FOREACHvertex_(facet->vertices) {
    dist= qh_pointdist (vertex->point, point, -dim);
    if (dist < bestdist) {
      bestdist= dist;
      bestvertex= vertex;
    }
  }
  *bestdistp= sqrt (bestdist);
  return bestvertex;
} /* nearvertex */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="newhashtable">-</a>
  
  qh_newhashtable( newsize )
    returns size of qh.hash_table of at least newsize slots

  notes:
    assumes qh.hash_table is NULL
    qh_HASHfactor determines the number of extra slots
    size is not divisible by 2, 3, or 5
*/
int qh_newhashtable(int newsize) {
  int size;

  size= ((newsize+1)*qh_HASHfactor) | 0x1;  /* odd number */
  while (True) { 
    if ((size%3) && (size%5))
      break;
    size += 2;
    /* loop terminates because there is an infinite number of primes */
  }
  qh hash_table= qh_setnew (size);
  qh_setzero (qh hash_table, 0, size);
  return size;
} /* newhashtable */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="newvertex">-</a>
  
  qh_newvertex( point )
    returns a new vertex for point
*/
vertexT *qh_newvertex(pointT *point) {
  vertexT *vertex;

  zinc_(Ztotvertices);
  vertex= (vertexT *)qh_memalloc(sizeof(vertexT));
  memset ((char *) vertex, 0, sizeof (vertexT));
  if (qh vertex_id == 0xFFFFFF) {
    ivp_message( "qhull input error: more than %d vertices.  Id field overflows and two vertices\n\
may have the same identifier.  Vertices not sorted correctly.\n", 0xFFFFFF);
    qh_errexit(qh_ERRinput, NULL, NULL);
  }
  if (qh vertex_id == qh tracevertex_id)
    qh tracevertex= vertex;
  vertex->id= qh vertex_id++;
  vertex->point= point;
  trace4((qh ferr, "qh_newvertex: vertex p%d (v%d) created\n", qh_pointid(vertex->point), 
	  vertex->id));
  return (vertex);
} /* newvertex */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="nextridge3d">-</a>
  
  qh_nextridge3d( atridge, facet, vertex )
    return next ridge and vertex for a 3d facet

  notes:
    in qh_ORIENTclock order
    this is a O(n^2) implementation to trace all ridges
    be sure to stop on any 2nd visit
  
  design:
    for each ridge
      exit if it is the ridge after atridge
*/
ridgeT *qh_nextridge3d (ridgeT *atridge, facetT *facet, vertexT **vertexp) {
  vertexT *atvertex, *vertex, *othervertex;
  ridgeT *ridge, **ridgep;

  if ((atridge->top == facet) ^ qh_ORIENTclock)
    atvertex= SETsecondt_(atridge->vertices, vertexT);
  else
    atvertex= SETfirstt_(atridge->vertices, vertexT);
  FOREACHridge_(facet->ridges) {
    if (ridge == atridge)
      continue;
    if ((ridge->top == facet) ^ qh_ORIENTclock) {
      othervertex= SETsecondt_(ridge->vertices, vertexT);
      vertex= SETfirstt_(ridge->vertices, vertexT);
    }else {
      vertex= SETsecondt_(ridge->vertices, vertexT);
      othervertex= SETfirstt_(ridge->vertices, vertexT);
    }
    if (vertex == atvertex) {
      if (vertexp)
        *vertexp= othervertex;
      return ridge;
    }
  }
  return NULL;
} /* nextridge3d */
#else /* qh_NOmerge */
void qh_matchduplicates (facetT *atfacet, int atskip, int hashsize, int *hashcount) {
}
ridgeT *qh_nextridge3d (ridgeT *atridge, facetT *facet, vertexT **vertexp) {

  return NULL;
}
#endif /* qh_NOmerge */
  
/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="outcoplanar">-</a>
  
  qh_outcoplanar()
    move points from all facets' outsidesets to their coplanarsets

  notes:
    for post-processing under qh.NARROWhull

  design:
    for each facet
      for each outside point for facet
        partition point into coplanar set
*/
void qh_outcoplanar (void /* facet_list */) {
  pointT *point, **pointp;
  facetT *facet;
  realT dist;

  trace1((qh ferr, "qh_outcoplanar: move outsideset to coplanarset for qh NARROWhull\n"));
  FORALLfacets {
    FOREACHpoint_(facet->outsideset) {
      qh num_outside--;
      if (qh KEEPcoplanar || qh KEEPnearinside) {
	qh_distplane (point, facet, &dist);
        zinc_(Zpartition);
	qh_partitioncoplanar (point, facet, &dist);
      }
    }
    qh_setfree (&facet->outsideset);
  }
} /* outcoplanar */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="point">-</a>
  
  qh_point( id )
    return point for a point id, or NULL if unknown

  alternative code:
    return ((pointT *)((unsigned   long)qh.first_point
           + (unsigned long)((id)*qh.normal_size)));
*/
pointT *qh_point (int id) {

  if (id < 0)
    return NULL;
  if (id < qh num_points)
    return qh first_point + id * qh hull_dim;
  id -= qh num_points;
  if (id < qh_setsize (qh other_points))
    return SETelemt_(qh other_points, id, pointT);
  return NULL;
} /* point */
  
/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="point_add">-</a>
  
  qh_point_add( set, point, elem )
    stores elem at set[point.id]
  
  returns:
    access function for qh_pointfacet and qh_pointvertex

  notes:
    checks point.id
*/
void qh_point_add (setT *set, pointT *point, void *elem) {
  int id, size;

  SETreturnsize_(set, size);
  if ((id= qh_pointid(point)) < 0)
    ivp_message( "qhull internal warning (point_add): unknown point %p id %d\n", 
      point, id);
  else if (id >= size) {
    ivp_message( "qhull internal errror (point_add): point p%d is out of bounds (%d)\n",
	     id, size);
    qh_errexit (qh_ERRqhull, NULL, NULL);
  }else
    SETelem_(set, id)= elem;
} /* point_add */


/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="pointfacet">-</a>
  
  qh_pointfacet()
    return temporary set of facet for each point
    the set is indexed by point id

  notes:
    vertices assigned to one of the facets
    coplanarset assigned to the facet
    outside set assigned to the facet
    NULL if no facet for point (inside)
      includes qh.GOODpointp

  access:
    FOREACHfacet_i_(facets) { ... }
    SETelem_(facets, i)
  
  design:
    for each facet
      add each vertex
      add each coplanar point
      add each outside point
*/
setT *qh_pointfacet (void /*qh facet_list*/) {
  int numpoints= qh num_points + qh_setsize (qh other_points);
  setT *facets;
  facetT *facet;
  vertexT *vertex, **vertexp;
  pointT *point, **pointp;
  
  facets= qh_settemp (numpoints);
  qh_setzero (facets, 0, numpoints);
  qh vertex_visit++;
  FORALLfacets {
    FOREACHvertex_(facet->vertices) {
      if (vertex->visitid != qh vertex_visit) {
        vertex->visitid= qh vertex_visit;
        qh_point_add (facets, vertex->point, facet);
      }
    }
    FOREACHpoint_(facet->coplanarset) 
      qh_point_add (facets, point, facet);
    FOREACHpoint_(facet->outsideset) 
      qh_point_add (facets, point, facet);
  }
  return facets;
} /* pointfacet */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="pointvertex">-</a>
  
  qh_pointvertex(  )
    return temporary set of vertices indexed by point id
    entry is NULL if no vertex for a point
      this will include qh.GOODpointp

  access:
    FOREACHvertex_i_(vertices) { ... }
    SETelem_(vertices, i)
*/
setT *qh_pointvertex (void /*qh facet_list*/) {
  int numpoints= qh num_points + qh_setsize (qh other_points);
  setT *vertices;
  vertexT *vertex;
  
  vertices= qh_settemp (numpoints);
  qh_setzero (vertices, 0, numpoints);
  FORALLvertices 
    qh_point_add (vertices, vertex->point, vertex);
  return vertices;
} /* pointvertex */


/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="prependfacet">-</a>
  
  qh_prependfacet( facet, facetlist )
    prepend facet to the start of a facetlist

  returns:
    increments qh.numfacets
    updates facetlist, qh.facet_list, facet_next
  
  notes:
    be careful of prepending since it can lose a pointer.
      e.g., can lose _next by deleting and then prepending before _next
*/
void qh_prependfacet(facetT *facet, facetT **facetlist) {
  facetT *prevfacet, *list= *facetlist;
  

  trace4((qh ferr, "qh_prependfacet: prepend f%d before f%d\n",
	  facet->id, list->id));
  prevfacet= list->previous;
  facet->previous= prevfacet;
  if (prevfacet)
    prevfacet->next= facet;
  list->previous= facet;
  facet->next= *facetlist;
  if (qh facet_list == list)  /* this may change *facetlist */
    qh facet_list= facet;
  if (qh facet_next == list)
    qh facet_next= facet;
  *facetlist= facet;
  qh num_facets++;
} /* prependfacet */


/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="printhashtable">-</a>
  
  qh_printhashtable( fp )
    print hash table to fp

  notes:
    not in I/O to avoid bringing io.c in
  
  design:
    for each hash entry
      if defined
        if unmatched or will merge (NULL, qh_MERGEridge, qh_DUPLICATEridge)
          print entry and neighbors
*/
void qh_printhashtable(FILE *fp) {
  facetT *facet, *neighbor;
  int id, facet_i, facet_n, neighbor_i= 0, neighbor_n= 0;
  vertexT *vertex, **vertexp;

  FOREACHfacet_i_(qh hash_table) {
    if (facet) {
      FOREACHneighbor_i_(facet) {
        if (!neighbor || neighbor == qh_MERGEridge || neighbor == qh_DUPLICATEridge) 
          break;
      }
      if (neighbor_i == neighbor_n)
        continue;
      fprintf (fp, "hash %d f%d ", facet_i, facet->id);
      FOREACHvertex_(facet->vertices)
        fprintf (fp, "v%d ", vertex->id);
      fprintf (fp, "\n neighbors:");
      FOREACHneighbor_i_(facet) {
	if (neighbor == qh_MERGEridge)
	  id= -3;
	else if (neighbor == qh_DUPLICATEridge)
	  id= -2;
	else
	  id= getid_(neighbor);
        fprintf (fp, " %d", id);
      }
      fprintf (fp, "\n");
    }
  }
} /* printhashtable */
     

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="printlists">-</a>
  
  qh_printlists( fp )
    print out facet and vertex list for debugging (without 'f/v' tags)
*/
void qh_printlists (void) {
  facetT *facet;
  vertexT *vertex;
  
  ivp_message( "qh_printlists: facets:");
  FORALLfacets 
    ivp_message( " %d", facet->id);
  ivp_message( "\n  new facets %d visible facets %d next facet for addpoint %d\n  vertices (new %d):",
     getid_(qh newfacet_list), getid_(qh visible_list), getid_(qh facet_next),
     getid_(qh newvertex_list));
  FORALLvertices
    ivp_message( " %d", vertex->id);
  ivp_message( "\n");
} /* printlists */
  
/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="resetlists">-</a>
  
  qh_resetlists( stats )
    reset newvertex_list, newfacet_list, visible_list
    if stats, 
      maintains statistics

  returns:
    clears num_visible
    visible_list is empty if qh_deletevisible was called
*/
void qh_resetlists (boolT stats /*qh newvertex_list newfacet_list visible_list*/) {
  vertexT *vertex;
  facetT *newfacet, *visible;
  int totnew=0, totver=0;
  
  if (stats) {
    FORALLvertex_(qh newvertex_list)
      totver++;
    FORALLnew_facets 
      totnew++;
    zadd_(Zvisvertextot, totver);
    zmax_(Zvisvertexmax, totver);
    zadd_(Znewfacettot, totnew);
    zmax_(Znewfacetmax, totnew);
  }
  FORALLvertex_(qh newvertex_list)
    vertex->newlist= False;
  qh newvertex_list= NULL;
  FORALLnew_facets
    newfacet->newfacet= False;
  qh newfacet_list= NULL;
  FORALLvisible_facets {
    visible->f.replace= NULL;
    visible->visible= False;
  }
  qh visible_list= NULL;
  qh num_visible= 0;
  qh NEWfacets= False;
} /* resetlists */

/*-<a                             href="qh-c.htm"
  >-------------------------------</a><a name="setvoronoi_all">-</a>
  
  qh_setvoronoi_all()
    compute Voronoi centers for all facets
    includes upperDelaunay facets if qh.UPPERdelaunay ('Qu')

  returns:
    facet->center is the Voronoi center
    
  notes:
    this is unused/untested code
      please email bradb@shore.net if this works ok for you
  
  use:
    FORALLvertices {...} to locate the vertex for a point.  
    FOREACHneighbor_(vertex) {...} to visit the Voronoi centers for a Voronoi cell.
*/
void qh_setvoronoi_all (void) {
  facetT *facet;

  qh_clearcenters (qh_ASvoronoi);
  qh_vertexneighbors();
  
  FORALLfacets {
    if (!facet->normal || !facet->upperdelaunay || qh UPPERdelaunay) {
      if (!facet->center)
        facet->center= qh_facetcenter (facet->vertices);
    }
  }
} /* setvoronoi_all */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="vertexintersect">-</a>
  
  qh_vertexintersect( vertexsetA, vertexsetB )
    intersects two vertex sets (inverse id ordered)
    vertexsetA is a temporary set at the top of qhmem.tempstack

  returns:
    replaces vertexsetA with the intersection
  
  notes:
    could overwrite vertexsetA if currently too slow
*/
void qh_vertexintersect(setT **vertexsetA,setT *vertexsetB) {
  setT *intersection;

  intersection= qh_vertexintersect_new (*vertexsetA, vertexsetB);
  qh_settempfree (vertexsetA);
  *vertexsetA= intersection;
  qh_settemppush (intersection);
} /* vertexintersect */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="vertexintersect_new">-</a>
  
  qh_vertexintersect_new(  )
    intersects two vertex sets (inverse id ordered)

  returns:
    a new set
*/
setT *qh_vertexintersect_new (setT *vertexsetA,setT *vertexsetB) {
  setT *intersection= qh_setnew (qh hull_dim - 1);
  vertexT **vertexA= SETaddr_(vertexsetA, vertexT); 
  vertexT **vertexB= SETaddr_(vertexsetB, vertexT); 

  while (*vertexA && *vertexB) {
    if (*vertexA  == *vertexB) {
      qh_setappend(&intersection, *vertexA);
      vertexA++; vertexB++;
    }else {
      if ((*vertexA)->id > (*vertexB)->id)
        vertexA++;
      else
        vertexB++;
    }
  }
  return intersection;
} /* vertexintersect_new */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="vertexneighhbors">-</a>
  
  qh_vertexneighhbors()
    for each vertex in qh.facet_list, 
      determine its neighboring facets 

  returns:
    sets qh.VERTEXneighbors
      nop if qh.VERTEXneighbors already set
      qh_addpoint() will maintain them

  notes:
    assumes all vertex->neighbors are NULL

  design:
    for each facet
      for each vertex
        append facet to vertex->neighbors
*/
void qh_vertexneighbors (void /*qh facet_list*/) {
  facetT *facet;
  vertexT *vertex, **vertexp;

  if (qh VERTEXneighbors)
    return;
  trace1((qh ferr, "qh_vertexneighbors: determing neighboring facets for each vertex\n"));
  qh vertex_visit++;
  FORALLfacets {
    if (facet->visible)
      continue;
    FOREACHvertex_(facet->vertices) {
      if (vertex->visitid != qh vertex_visit) {
        vertex->visitid= qh vertex_visit;
        vertex->neighbors= qh_setnew (qh hull_dim);
      }
      qh_setappend (&vertex->neighbors, facet);
    }
  }
  qh VERTEXneighbors= True;
} /* vertexneighbors */

/*-<a                             href="qh-c.htm#poly"
  >-------------------------------</a><a name="vertexsubset">-</a>
  
  qh_vertexsubset( vertexsetA, vertexsetB )
    returns True if vertexsetA is a subset of vertexsetB
    assumes vertexsets are sorted

  note:    
    empty set is a subset of any other set
*/
boolT qh_vertexsubset(setT *vertexsetA, setT *vertexsetB) {
  vertexT **vertexA= (vertexT **) SETaddr_(vertexsetA, vertexT);
  vertexT **vertexB= (vertexT **) SETaddr_(vertexsetB, vertexT);

  while (True) {
    if (!*vertexA)
      return True;
    if (!*vertexB)
      return False;
    if ((*vertexA)->id > (*vertexB)->id)
      return False;
    if (*vertexA  == *vertexB)
      vertexA++;
    vertexB++; 
  }
  return False; /* avoid warnings */
} /* vertexsubset */

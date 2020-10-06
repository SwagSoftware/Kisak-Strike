/* insed3.f -- translated by f2c (version 19990311).
*/

#include <ivp_physics.hxx>
#include <geompack.hxx>


void IVP_Geompack::insed3_(
			   int	*a,
			   int	*b,
			   int	*hfl
			  ) {

    int	   *polyhedronfaceindices     = this->g_polyhedronfaceindices;
    int    *facesdata                 = this->g_facesdata;
    int    *facestype                 = this->g_facestype;
    double *normals                   = this->g_normals;
    int    *faceverticeslist          = this->g_faceverticeslist;
    double *edge_angles               = this->g_edge_angles;

    /* Local variables */
    int f, g, i__, j, k, sp, sq;


/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Insert an edge on a face of polyhedral decomposition data */
/*        structure. It is assumed that edge is entirely inside face. */

/*     Input parameters: */
/*        A,B - indices of FACEVERTICESLIST for nonadjacent vertices on same face */
/*        NFACE - number of faces or positions used in FACESDATA array */
/*        NVERT - number of positions used in FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        ADDR_OF_N_POLYHEDRONFACES - number of positions used in POLYHEDRONFACEINDICES array */
/*        ADDR_OF_SIZE_FACEARRAYS - maximum size available for FACESDATA, FACESTYPE, NORMALS arrays */
/*        ADDR_OF_SIZE_FACEVERTEXARRAYS - maximum size available for FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        ADDR_OF_SIZE_POLYHEDRONFACEINDICES - maximum size available for POLYHEDRONFACEINDICES array */
/*        FACESDATA(1:3,1:NFACE) - face pointer list */
/*        FACTYP(1:NFACE) - face types */
/*        NORMALS(1:3,1:NFACE) - unit normal vectors for faces; outward */
/*              normal corresponds to CCW traversal of face from polyh */
/*              with index FACESDATA(2,F) */
/*        FACEVERTICESLIST(1:6,1:NVERT) - face vertex list */
/*        EDGE_ANGLES(1:NVERT) - edge angles */
/*        HFL(1:*) - head pointer to face indices in POLYHEDRONFACEINDICES for each polyh */
/*        POLYHEDRONFACEINDICES(1:2,1:ADDR_OF_N_POLYHEDRONFACES) - list of signed face indices for each polyh */

/*     Updated parameters: */
/*        A - index in FACEVERTICESLIST of new edge; LOC field of output A same as */
/*              input A */
/*        NFACE - increased by 1 */
/*        NVERT - increased by 2 */
/*        ADDR_OF_N_POLYHEDRONFACES - increased by 1 or 2 */
/*        FACESDATA,FACTYP,NORMALS,FACEVERTICESLIST,EDGE_ANGLES,POLYHEDRONFACEINDICES - updated by new edge */
/*        NORMALS - one column added at end */

/*     Abnormal return: */
/*        IERR is set to 15, 16, or 17 */




    /* Parameter adjustments */
    normals -= 4;
    --facestype;
    facesdata -= 4;
    --edge_angles;
    faceverticeslist -= 7;
    polyhedronfaceindices -= 3;
    --hfl;

    /* Function Body */
    f = faceverticeslist[*a * 6 + 2];
    i__ = this->n_work_vertices + 1;
    j = this->n_work_vertices + 2;
    this->n_work_vertices = j;

recheck_size1:
    if ( (this->n_work_vertices+2) > this->size_facevertexarrays) {
	int res = 0;
	res = res | increase_memory((void **)&this->g_faceverticeslist, &this->size_facevertexarrays, 6*sizeof(int));
	res = res | increase_memory((void **)&this->g_edge_angles     , &this->size_facevertexarrays,   sizeof(double));
	if ( res == 0 ) {
	    this->ierr = 500;
	    return;
	}
//	*addr_of_size_facevertexarrays *= 2;
	this->size_facevertexarrays += 1024;
	faceverticeslist = this->g_faceverticeslist;
	faceverticeslist -= 7;
	edge_angles = this->g_edge_angles;
	--edge_angles;
	goto recheck_size1;
    }

    faceverticeslist[i__ * 6 + 1] = faceverticeslist[*a * 6 + 1];
    faceverticeslist[i__ * 6 + 3] = *b;
    k = faceverticeslist[*a * 6 + 4];
    faceverticeslist[i__ * 6 + 4] = k;
    faceverticeslist[k * 6 + 3] = i__;
    faceverticeslist[i__ * 6 + 5] = j;
    faceverticeslist[i__ * 6 + 6] = j;
    faceverticeslist[j * 6 + 1] = faceverticeslist[*b * 6 + 1];
    faceverticeslist[j * 6 + 2] = f;
    faceverticeslist[j * 6 + 3] = *a;
    k = faceverticeslist[*b * 6 + 4];
    faceverticeslist[j * 6 + 4] = k;
    faceverticeslist[k * 6 + 3] = j;
    faceverticeslist[j * 6 + 5] = i__;
    faceverticeslist[j * 6 + 6] = i__;
    faceverticeslist[*a * 6 + 4] = j;
    faceverticeslist[*b * 6 + 4] = i__;
    edge_angles[i__] = IVP_PI;
    edge_angles[j] = IVP_PI;
    this->nface++;

recheck_size2:
    if ( (this->nface+2) > this->size_facearrays ) {
	int res = 0;
	res = res | increase_memory((void **)&this->g_normals  , &this->size_facearrays, 3*sizeof(double));
	res = res | increase_memory((void **)&this->g_facesdata, &this->size_facearrays, 3*sizeof(int));
	res = res | increase_memory((void **)&this->g_facestype, &this->size_facearrays,   sizeof(int));
	if ( res == 0 ) {
	    this->ierr = 500;
	    return;
	}
//	*addr_of_size_facearrays *= 2;
	this->size_facearrays += 1024;
	facesdata = this->g_facesdata;
	facesdata -= 4;
	facestype = this->g_facestype;
	--facestype;
	normals = this->g_normals;
	normals -= 4;
	goto recheck_size2;
    }

    facesdata[f * 3 + 1] = *a;
    sp = facesdata[f * 3 + 2];
    sq = facesdata[f * 3 + 3];
    facesdata[this->nface * 3 + 1] = *b;
    facesdata[this->nface * 3 + 2] = sp;
    facesdata[this->nface * 3 + 3] = sq;
    facestype[this->nface] = facestype[f];
    normals[this->nface * 3 + 1] = normals[f * 3 + 1];
    normals[this->nface * 3 + 2] = normals[f * 3 + 2];
    normals[this->nface * 3 + 3] = normals[f * 3 + 3];
    k = *b;
L10:
    faceverticeslist[k * 6 + 2] = this->nface;
    k = faceverticeslist[k * 6 + 3];
    if (k != *b) {
	goto L10;
    }
    g = hfl[abs(sp)];
    this->n_polyhedronfaces++;

recheck_size_3:
    if ( (2+this->n_polyhedronfaces) > this->size_polyhedronfaceindices ) {
	int res = increase_memory((void **)&this->g_polyhedronfaceindices, &this->size_polyhedronfaceindices, 2*sizeof(int));
	if ( res == 0 ) {
	    this->ierr = 500;
	    return;
	}
//	*addr_of_size_polyhedronfaceindices *= 2;
	this->size_polyhedronfaceindices += 1024;
	polyhedronfaceindices = this->g_polyhedronfaceindices;
	polyhedronfaceindices -= 3;
	goto recheck_size_3;
    }

    polyhedronfaceindices[(this->n_polyhedronfaces << 1) + 1] = i_sign(this->nface, sp);
    polyhedronfaceindices[(this->n_polyhedronfaces << 1) + 2] = polyhedronfaceindices[(g << 1) + 2];
    polyhedronfaceindices[(g << 1) + 2] = this->n_polyhedronfaces;
    if (sq != 0) {
	g = hfl[abs(sq)];
	this->n_polyhedronfaces++;

recheck_size_4:
	if ( (2+this->n_polyhedronfaces) > this->size_polyhedronfaceindices ) {
	    int res = increase_memory((void **)&this->g_polyhedronfaceindices, &this->size_polyhedronfaceindices, 2*sizeof(int));
	    if ( res == 0 ) {
		this->ierr = 500;
		return;
	    }
//	    *addr_of_size_polyhedronfaceindices *= 2;
	    this->size_polyhedronfaceindices += 1024;
	    polyhedronfaceindices = this->g_polyhedronfaceindices;
	    polyhedronfaceindices -= 3;
	    goto recheck_size_4;
	}

	polyhedronfaceindices[(this->n_polyhedronfaces << 1) + 1] = i_sign(this->nface, sq);
	polyhedronfaceindices[(this->n_polyhedronfaces << 1) + 2] = polyhedronfaceindices[(g << 1) + 2];
	polyhedronfaceindices[(g << 1) + 2] = this->n_polyhedronfaces;
    } else {
	if ((faceverticeslist[*b * 6 + 1] - faceverticeslist[*a * 6 + 1]) * sp > 0) {
	    faceverticeslist[i__ * 6 + 5] = 0;
	    faceverticeslist[j * 6 + 6] = 0;
	    edge_angles[j] = -1.;
	} else {
	    faceverticeslist[j * 6 + 5] = 0;
	    faceverticeslist[i__ * 6 + 6] = 0;
	    edge_angles[i__] = -1.;
	}
    }
    *a = i__;

    return;
}

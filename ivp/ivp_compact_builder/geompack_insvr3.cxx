/* insvr3.f -- translated by f2c (version 19990311).
*/

#include <ivp_physics.hxx>

#include <geompack.hxx>


void IVP_Geompack::insvr3_(int a) {

    int    *faceverticeslist = this->g_faceverticeslist;
    double *edge_angles      = this->g_edge_angles;

    int lnxt, b, c__, d__, i__, j, k, l;
    long int bflag;
    int n, la, lb, li, lj;
    double angnxt, ang;


/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Insert a vertex on an edge of polyhedral decomposition */
/*        data structure. */

/*     Input parameters: */
/*        A - index of FACEVERTICESLIST specifying edge containing inserted vertex */
/*        ADDR_OF_N_ORIGINAL_VERTICES - number of vertex coordinates */
/*        ADDR_OF_N_WORK_VERTICES - number of positions used in FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        ADDR_OF_SIZE_FACEVERTEXARRAYS - maximum size available for FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        VCL(1:3,1:ADDR_OF_N_ORIGINAL_VERTICES+1) - vertex coordinate list; VCL(*,ADDR_OF_N_ORIGINAL_VERTICES+1) are */
/*              coordinates of vertex to be inserted */
/*        FACEVERTICESLIST(1:6,1:ADDR_OF_N_WORK_VERTICES) - face vertex list */
/*        EDGE_ANGLES(1:ADDR_OF_N_WORK_VERTICES) - edge angles */

/*     Updated parameters: */
/*        ADDR_OF_N_ORIGINAL_VERTICES,ADDR_OF_N_WORK_VERTICES,FACEVERTICESLIST,EDGE_ANGLES */

/*     Abnormal return: */
/*        IERR is set to 15 */




    /* Parameter adjustments */
    --edge_angles;
    faceverticeslist -= 7;

    /* Function Body */
    this->n_original_vertices++;
    b = faceverticeslist[a * 6 + 3];
    la = faceverticeslist[a * 6 + 1];
    lb = faceverticeslist[b * 6 + 1];
    i__ = a;

/*     Find start edge of FACEVERTICESLIST if AB lies on boundary of decomposition. */

    c__ = i__;
L10:
    d__ = faceverticeslist[c__ * 6 + 5];
    if (d__ != 0 && d__ != i__) {
	c__ = d__;
	goto L10;
    }
    bflag = d__ == 0;

/*     Insert new entry of FACEVERTICESLIST for each face containing edge AB. */

    i__ = c__;
    k = this->n_work_vertices;
L20:
    j = faceverticeslist[i__ * 6 + 3];
    k++;

recheck_size:
    if ( (k+2) > this->size_facevertexarrays) {
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
	goto recheck_size;
    }

    faceverticeslist[k * 6 + 1] = this->n_original_vertices;
    faceverticeslist[k * 6 + 2] = faceverticeslist[i__ * 6 + 2];
    faceverticeslist[k * 6 + 3] = j;
    faceverticeslist[k * 6 + 4] = i__;
    faceverticeslist[k * 6 + 5] = 0;
    faceverticeslist[k * 6 + 6] = 0;
    faceverticeslist[i__ * 6 + 3] = k;
    faceverticeslist[j * 6 + 4] = k;
    edge_angles[k] = -1.;
    i__ = faceverticeslist[i__ * 6 + 6];
    if (i__ != 0 && i__ != c__) {
	goto L20;
    }
    this->n_work_vertices = k;

/*     Set EDGA, EDGC, and EDGE_ANGLES fields. */

    i__ = c__;
    l = faceverticeslist[i__ * 6 + 6];
    ang = edge_angles[i__];
L30:
    k = faceverticeslist[i__ * 6 + 3];
    j = faceverticeslist[k * 6 + 3];
    li = faceverticeslist[i__ * 6 + 1];
    lj = faceverticeslist[j * 6 + 1];
    n = faceverticeslist[l * 6 + 3];
    lnxt = faceverticeslist[l * 6 + 6];
    angnxt = edge_angles[l];
    if (li < lj) {
	if (faceverticeslist[l * 6 + 1] == li) {
	    faceverticeslist[k * 6 + 5] = n;
	    faceverticeslist[n * 6 + 6] = k;
	    edge_angles[n] = ang;
	} else {
	    faceverticeslist[i__ * 6 + 6] = n;
	    faceverticeslist[n * 6 + 5] = i__;
	    faceverticeslist[k * 6 + 5] = l;
	    faceverticeslist[l * 6 + 6] = k;
	    edge_angles[l] = ang;
	    if (lnxt == 0) {
		faceverticeslist[l * 6 + 5] = 0;
	    }
	}
    } else {
	if (faceverticeslist[l * 6 + 1] == li) {
	    faceverticeslist[k * 6 + 6] = n;
	    faceverticeslist[n * 6 + 5] = k;
	    edge_angles[k] = ang;
	    faceverticeslist[i__ * 6 + 5] = l;
	    faceverticeslist[l * 6 + 6] = i__;
	    edge_angles[l] = ang;
	    if (lnxt == 0) {
		faceverticeslist[l * 6 + 5] = 0;
	    }
	} else {
	    faceverticeslist[k * 6 + 6] = l;
	    faceverticeslist[l * 6 + 5] = k;
	    edge_angles[k] = ang;
	    faceverticeslist[i__ * 6 + 5] = n;
	    faceverticeslist[n * 6 + 6] = i__;
	    edge_angles[n] = ang;
	}
	if (bflag && i__ == c__) {
	    faceverticeslist[i__ * 6 + 6] = 0;
	    edge_angles[i__] = -1.;
	}
    }
    i__ = l;
    l = lnxt;
    ang = angnxt;
    if (l != 0 && i__ != c__) {
	goto L30;
    }

    return;
}

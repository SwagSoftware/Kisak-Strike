/* resedg.f -- translated by f2c (version 19990311).
*/


#include "ivp_physics.hxx"
#include <ivp_betterdebugmanager.hxx>
#include <geompack.hxx>


void IVP_Geompack::resedg_(
			   int		u_in,
			   long int	*rflag
			  ) {

    int *   intworkarray	      = this->g_intworkarray;
    double *doubleworkarray           = this->g_doubleworkarray;
    double *vcl                       = this->g_vcl;
    int	   *polyhedronfirstfaceoffset = this->g_polyhedronfirstfaceoffset;
    int	   *polyhedronfaceindices     = this->g_polyhedronfaceindices;
    int    *facesdata                 = this->g_facesdata;
    double *normals                   = this->g_normals;
    int    *faceverticeslist          = this->g_faceverticeslist;
    double *edge_angles               = this->g_edge_angles;


    /* System generated locals */
    int i__1, i__2;
    double d__1, d__2, d__3;

    /* Local variables */
    int n_edges, nang;
    double leng, dtol, dotp;
    int a, b, c__;
    double e[3];
    int f, i__, j, k, nedgc, p;
    double t;
    int v;
    double anghi, anglo, nrmlc[4];
    long int first;
    int ca, cb;
    double da, db, ce;
    int la, lb, lc;
    double cn;
    int fl;
    double ep[3];
    int fr, lu, lv, sp;
    double mindis;
    int nce;
    double ang[5], enc[3];
    int ccw;
    double enl[3], enr[3];
    int luo, lvo;
    double sum;
    int ptr;



/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Attempt to resolve reflex edge by a cut polygon which */
/*        does not create small dihedral angles and does not pass near */
/*        any vertices not on cut plane. */

/*     Input parameters: */
/*        U - index in FACEVERTICESLIST of reflex edge */
/*        ANGACC - min acceptable dihedral angle in radians produced by */
/*              a cut face */
/*        RDACC - min acceptable relative distance between a cut plane */
/*              and vertices not on plane */
/*        ADDR_OF_N_ORIGINAL_VERTICES - number of vertex coordinates or positions used in VCL */
/*        NFACE - number of faces or positions used in FACESDATA array */
/*        ADDR_OF_N_WORK_VERTICES - number of positions used in FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        NPOLH - number of polyhedra or positions used in POLYHEDRONFIRSTFACEOFFSET array */
/*        ADDR_OF_N_POLYHEDRONFACES - number of positions used in POLYHEDRONFACEINDICES array */
/*        ADDR_OF_SIZE_VCL - maximum size available for VCL array */
/*        ADDR_OF_SIZE_FACEARRAYS - maximum size available for FACESDATA, FACESTYPE, NORMALS arrays */
/*        ADDR_OF_SIZE_FACEVERTEXARRAYS - maximum size available for FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        ADDR_OF_SIZE_POLYHEDRONFIRSTFACEOFFSET - maximum size available for POLYHEDRONFIRSTFACEOFFSET array */
/*        ADDR_OF_SIZE_POLYHEDRONFACEINDICES - maximum size available for POLYHEDRONFACEINDICES array */
/*        ADDR_OF_SIZE_intWORKARRAY - maximum size available for intWORKARRAY array; should be about */
/*              5/3*NEDG where NEDG is number of edges in polyhedron */
/*              containing reflex edge */
/*        ADDR_OF_SIZE_DOUBLEWORKARRAY - maximum size available for DOUBLEWORKARRAY array; should be >= NEDG */
/*        VCL(1:3,1:ADDR_OF_N_ORIGINAL_VERTICES) - vertex coordinate list */
/*        FACESDATA(1:3,1:NFACE) - face pointer list */
/*        FACESTYPE(1:NFACE) - face types */
/*        NORMALS(1:3,1:NFACE) - unit normal vectors for faces */
/*        FACEVERTICESLIST(1:6,1:ADDR_OF_N_WORK_VERTICES) - face vertex list */
/*        EDGE_ANGLES(1:ADDR_OF_N_WORK_VERTICES) - edge angles */
/*        POLYHEDRONFIRSTFACEOFFSET(1:NPOLH) - head ptr to face indices in POLYHEDRONFACEINDICES for each polyh */
/*        POLYHEDRONFACEINDICES(1:2,1:ADDR_OF_N_POLYHEDRONFACES) - list of signed face indices for each polyh */

/*     Updated parameters: */
/*        ADDR_OF_N_ORIGINAL_VERTICES,NFACE,ADDR_OF_N_WORK_VERTICES,NPOLH,ADDR_OF_N_POLYHEDRONFACES - sizes updated due to cut face */
/*        VCL,FACESDATA,FACESTYPE,NORMALS,FACEVERTICESLIST,EDGE_ANGLES,POLYHEDRONFIRSTFACEOFFSET,POLYHEDRONFACEINDICES - updated by cut face */

/*     Output parameters: */
/*        RFLAG - .TRUE. iff reflex edge is resolved */

/*     Working parameters: */
/*        intWORKARRAY(1:3,1:*ADDR_OF_SIZE_intWORKARRAY) - int work array */
/*        DOUBLEWORKARRAY(1:ADDR_OF_SIZE_DOUBLEWORKARRAY) - double precision work array */

/*     Routines called: */
/*        CUTFAC, INSFAC */

/*     Abnormal return: */
/*        IERR is set to 6, 7, 14, 15, 16, 17, 18, 325, 326, or 328 */




/*     Find faces FL, FR and polyhedron P containing reflex edge UV. */
/*     Do not resolve UV if FL or FR is a double-occurring face. */

    /* Parameter adjustments */
    vcl -= 4;
    normals -= 4;
    facesdata -= 4;
    --edge_angles;
    faceverticeslist -= 7;
    --polyhedronfirstfaceoffset;
    polyhedronfaceindices -= 3;
    intworkarray -= 4; // vorerst mal drinlassen!!!

    /* Function Body */
    *rflag = 0;
    v = faceverticeslist[u_in * 6 + 3];
    lu = faceverticeslist[u_in * 6 + 1];
    lv = faceverticeslist[v * 6 + 1];
    fl = faceverticeslist[u_in * 6 + 2];
    fr = faceverticeslist[faceverticeslist[u_in * 6 + 6] * 6 + 2];
    if ((i__1 = facesdata[fl * 3 + 2], abs(i__1)) == (i__2 = facesdata[fl * 3 + 3], 
	    abs(i__2))) {
	return;
    }
    if ((i__1 = facesdata[fr * 3 + 2], abs(i__1)) == (i__2 = facesdata[fr * 3 + 3], 
	    abs(i__2))) {
	return;
    }
    if (lu < lv) {
	if (facesdata[fl * 3 + 2] > 0) {
	    p = facesdata[fl * 3 + 2];
	} else {
	    p = facesdata[fl * 3 + 3];
	}
	luo = lu;
	lvo = lv;
    } else {
	if (facesdata[fl * 3 + 2] < 0) {
	    p = -facesdata[fl * 3 + 2];
	} else {
	    p = -facesdata[fl * 3 + 3];
	}
	luo = lv;
	lvo = lu;
    }
    IVP_IF(1) {
	IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL3, "RESEDG: U,V,LU,LV,FL,FR,P = %d %d %d %d %d %d %d\n", u_in, v, lu, lv, fl, fr, p);
	}
    }

/*     Determine edges in polyhedron P and average edge length. */

    n_edges = 0;
    sum = 0.;
    ptr = polyhedronfirstfaceoffset[p];
L10:
    f = polyhedronfaceindices[(ptr << 1) + 1];
    if (f > 0) {
	ccw = 3;
    } else {
	ccw = 4;
	f = -f;
    }
    a = facesdata[f * 3 + 1];
    lb = faceverticeslist[a * 6 + 1];
L20:
    la = lb;
    b = faceverticeslist[ccw + a * 6];
    lb = faceverticeslist[b * 6 + 1];
    if (la < lb) {
/* Computing 2nd power */
	d__1 = vcl[la * 3 + 1] - vcl[lb * 3 + 1];
/* Computing 2nd power */
	d__2 = vcl[la * 3 + 2] - vcl[lb * 3 + 2];
/* Computing 2nd power */
	d__3 = vcl[la * 3 + 3] - vcl[lb * 3 + 3];
	leng = sqrt(d__1 * d__1 + d__2 * d__2 + d__3 * d__3);
	sum += leng;
	if (la != luo || lb != lvo) {
	    n_edges++;
recheck_size_1:
	    if ( (2+n_edges)*3 > (this->size_intworkarray) ) {
		int res = increase_memory((void **)&this->g_intworkarray, &this->size_intworkarray, sizeof(int));
		if ( res == 0 ) {
		    this->ierr = 500;
		    return;
		}
//		*addr_of_size_intworkarray *= 2;
		this->size_intworkarray += 1024;
		intworkarray = this->g_intworkarray;
		intworkarray -= 4; // vorerst mal drinlassen!!!
		goto recheck_size_1;
	    }
	    if (ccw == 3) {
		intworkarray[n_edges * 3 + 1] = a;
		intworkarray[n_edges * 3 + 2] = faceverticeslist[a * 6 + 6];
	    } else {
		intworkarray[n_edges * 3 + 1] = b;
		intworkarray[n_edges * 3 + 2] = faceverticeslist[b * 6 + 6];
	    }
	}
    }
    a = b;
    if (a != facesdata[f * 3 + 1]) {
	goto L20;
    }
    ptr = polyhedronfaceindices[(ptr << 1) + 2];
    if (ptr != polyhedronfirstfaceoffset[p]) {
	goto L10;
    }
    sum /= (double) (n_edges + 1);
    mindis = this->rdacc * sum;
    dtol = this->tolerance * sum;
recheck_size_2:
    if ( ((2+n_edges)*5) > (this->size_intworkarray) ) {
	int res = increase_memory((void **)&this->g_intworkarray, &this->size_intworkarray, sizeof(int));
	if ( res == 0 ) {
	    this->ierr = 500;
	    return;
	}
//	*addr_of_size_intworkarray *= 2;
	this->size_intworkarray += 1024;
	intworkarray = this->g_intworkarray;
	intworkarray -= 4; // vorerst mal drinlassen!!!
	goto recheck_size_2;
    }
    else if ( (2+n_edges) > (this->size_doubleworkarray) ) {
	int res = increase_memory((void **)&this->g_doubleworkarray, &this->size_doubleworkarray, sizeof(double));
	if ( res == 0 ) {
	    this->ierr = 500;
	    return;
	}
//	*addr_of_size_doubleworkarray *= 2;
	this->size_doubleworkarray += 1024;
	doubleworkarray = this->g_doubleworkarray;
	goto recheck_size_2;
    }

/*     Compute unit vectors E, ENL, ENR which are directed edge UV and */
/*     edge normals to UV on faces FL and FR. E is unit normal vector */
/*     of plane containing ENL, ENR (it is normalization of ENL x ENR). */

    e[0] = vcl[lvo * 3 + 1] - vcl[luo * 3 + 1];
    e[1] = vcl[lvo * 3 + 2] - vcl[luo * 3 + 2];
    e[2] = vcl[lvo * 3 + 3] - vcl[luo * 3 + 3];
/* Computing 2nd power */
    d__1 = e[0];
/* Computing 2nd power */
    d__2 = e[1];
/* Computing 2nd power */
    d__3 = e[2];
    leng = sqrt(d__1 * d__1 + d__2 * d__2 + d__3 * d__3);
    e[0] /= leng;
    e[1] /= leng;
    e[2] /= leng;
    enl[0] = normals[fl * 3 + 2] * e[2] - normals[fl * 3 + 3] * e[1];
    enl[1] = normals[fl * 3 + 3] * e[0] - normals[fl * 3 + 1] * e[2];
    enl[2] = normals[fl * 3 + 1] * e[1] - normals[fl * 3 + 2] * e[0];
    enr[0] = e[1] * normals[fr * 3 + 3] - e[2] * normals[fr * 3 + 2];
    enr[1] = e[2] * normals[fr * 3 + 1] - e[0] * normals[fr * 3 + 3];
    enr[2] = e[0] * normals[fr * 3 + 2] - e[1] * normals[fr * 3 + 1];
    if ((i__1 = facesdata[fl * 3 + 2], abs(i__1)) != p) {
	enl[0] = -enl[0];
	enl[1] = -enl[1];
	enl[2] = -enl[2];
    }
    if ((i__1 = facesdata[fr * 3 + 2], abs(i__1)) != p) {
	enr[0] = -enr[0];
	enr[1] = -enr[1];
	enr[2] = -enr[2];
    }
    k = 1;
    if (abs(e[1]) > abs(e[0])) {
	k = 2;
    }
    if (abs(e[2]) > (d__1 = e[k - 1], abs(d__1))) {
	k = 3;
    }

/*     Find range of angles [ANGLO, ANGHI] that will resolve reflex edge, */
/*     and an ordered list of at most NANGMX angles determined by planes */
/*     containing reflex edge plus an adjacent edge (the bisection and */
/*     other angles may also be added to list). The list is obtained */
/*     by cycling through edges incident on U and V. CCW = 1 (-1) if */
/*     going CCW (CW) around LA (when viewed from outside polyhedron P). */

    ang[0] = edge_angles[u_in] - IVP_PI;
    anglo = max(this->angacc,ang[0]);
/* Computing MIN */
    d__1 = IVP_PI, d__2 = edge_angles[u_in] - this->angacc;
    anghi = min(d__1,d__2);
    if (anglo > ang[0]) {
	nang = 0;
    } else {
	nang = 2;
	ang[1] = IVP_PI;
    }
    for (i__ = 1; i__ <= 2; ++i__) {
	ccw = 1;
	if (i__ == 1) {
	    la = lu;
	    b = faceverticeslist[u_in * 6 + 4];
	    if (lu > lv) {
		ccw = -1;
	    }
	} else {
	    la = lv;
	    b = v;
	    if (lu < lv) {
		ccw = -1;
	    }
	}
	first = 1;
	f = fl;
L30:
	if (! first) {
	    if (faceverticeslist[b * 6 + 1] == la) {
		b = faceverticeslist[b * 6 + 4];
	    } else {
		b = faceverticeslist[b * 6 + 3];
	    }
	}
	c__ = faceverticeslist[b * 6 + 3];
	if ((i__1 = facesdata[f * 3 + 2], abs(i__1)) == (i__2 = facesdata[f * 3 + 3], 
		abs(i__2))) {
	    if (faceverticeslist[b * 6 + 1] == la) {
		sp = -ccw * p;
	    } else {
		sp = ccw * p;
	    }
	} else if ((i__1 = facesdata[f * 3 + 2], abs(i__1)) == p) {
	    sp = facesdata[f * 3 + 2];
	} else {
	    sp = facesdata[f * 3 + 3];
	}
	if ((faceverticeslist[c__ * 6 + 1] - faceverticeslist[b * 6 + 1]) * sp > 0) {
	    b = faceverticeslist[b * 6 + 6];
	} else {
	    b = faceverticeslist[b * 6 + 5];
	}
	f = faceverticeslist[b * 6 + 2];
	if (f == fr) {
	    goto L50;
	}
	if (first) {
	    first = 0;
	} else {
	    if (faceverticeslist[b * 6 + 1] == la) {
		c__ = faceverticeslist[b * 6 + 3];
	    } else {
		c__ = b;
	    }
	    lc = faceverticeslist[c__ * 6 + 1];
	    ep[0] = vcl[lc * 3 + 1] - vcl[la * 3 + 1];
	    ep[1] = vcl[lc * 3 + 2] - vcl[la * 3 + 2];
	    ep[2] = vcl[lc * 3 + 3] - vcl[la * 3 + 3];
	    nrmlc[0] = ep[1] * e[2] - ep[2] * e[1];
	    nrmlc[1] = ep[2] * e[0] - ep[0] * e[2];
	    nrmlc[2] = ep[0] * e[1] - ep[1] * e[0];
/* Computing 2nd power */
	    d__1 = nrmlc[0];
/* Computing 2nd power */
	    d__2 = nrmlc[1];
/* Computing 2nd power */
	    d__3 = nrmlc[2];
	    leng = sqrt(d__1 * d__1 + d__2 * d__2 + d__3 * d__3);
/* Computing MAX */
	    d__1 = abs(ep[0]), d__2 = abs(ep[1]), d__1 = max(d__1,d__2), d__2 
		    = abs(ep[2]);
	    if (leng <= this->tolerance * max(d__1,d__2)) {
		goto L30;
	    }
	    nrmlc[0] /= leng;
	    nrmlc[1] /= leng;
	    nrmlc[2] /= leng;
	    enc[0] = e[1] * nrmlc[2] - e[2] * nrmlc[1];
	    enc[1] = e[2] * nrmlc[0] - e[0] * nrmlc[2];
	    enc[2] = e[0] * nrmlc[1] - e[1] * nrmlc[0];
	    nrmlc[0] = enr[1] * enc[2] - enr[2] * enc[1];
	    nrmlc[1] = enr[2] * enc[0] - enr[0] * enc[2];
	    nrmlc[2] = enr[0] * enc[1] - enr[1] * enc[0];
/* Computing 2nd power */
	    d__1 = nrmlc[0];
/* Computing 2nd power */
	    d__2 = nrmlc[1];
/* Computing 2nd power */
	    d__3 = nrmlc[2];
	    leng = sqrt(d__1 * d__1 + d__2 * d__2 + d__3 * d__3);
	    if (leng <= this->tolerance) {
		goto L30;
	    }
	    t = nrmlc[k - 1];
	    nrmlc[0] = enc[1] * enl[2] - enc[2] * enl[1];
	    nrmlc[1] = enc[2] * enl[0] - enc[0] * enl[2];
	    nrmlc[2] = enc[0] * enl[1] - enc[1] * enl[0];
/* Computing 2nd power */
	    d__1 = nrmlc[0];
/* Computing 2nd power */
	    d__2 = nrmlc[1];
/* Computing 2nd power */
	    d__3 = nrmlc[2];
	    leng = sqrt(d__1 * d__1 + d__2 * d__2 + d__3 * d__3);
	    if (leng <= this->tolerance) {
		goto L30;
	    }
	    if (nrmlc[k - 1] * t < 0.) {
		goto L30;
	    }
	    dotp = enr[0] * enc[0] + enr[1] * enc[1] + enr[2] * enc[2];
	    if (e[k - 1] * t < 0.) {
		dotp = -dotp;
	    }
	    if (abs(dotp) > 1. - this->tolerance) {
		dotp = d_sign(1.0, dotp);
	    }
	    t = acos(dotp);
	    if (t < anglo || t > anghi) {
		goto L30;
	    }
	    i__1 = nang;
	    for (j = 1; j <= i__1; ++j) {
		if ((d__1 = ang[j - 1] - t, abs(d__1)) <= this->tolerance) {
		    goto L30;
		}
/* L40: */
	    }
	    ++nang;
	    ang[nang - 1] = t;
	    if (nang >= 5) {
		goto L80;
	    }
	}
	goto L30;
L50:
	;
    }
    t = edge_angles[u_in] * .5;
    i__1 = nang;
    for (j = 1; j <= i__1; ++j) {
	if ((d__1 = ang[j - 1] - t, abs(d__1)) <= this->tolerance) {
	    goto L70;
	}
/* L60: */
    }
    ++nang;
    ang[nang - 1] = t;
L70:
    if (nang <= 3 && edge_angles[u_in] <= IVP_PI * 1.33) {
	nang += 2;
	ang[nang - 2] = edge_angles[u_in] * .25;
	ang[nang - 1] = edge_angles[u_in] * .75;
	if (IVP_PI - ang[nang - 1] <= .1) {
	    ang[nang - 2] = edge_angles[u_in] * .375;
	    ang[nang - 1] = edge_angles[u_in] * .625;
	}
	if (ang[nang - 2] < anglo || ang[nang - 2] > anghi) {
	    ang[nang - 2] = ang[nang - 1];
	    --nang;
	}
	if (ang[nang - 1] < anglo || ang[nang - 1] > anghi) {
	    --nang;
	}
    } else if (nang < 5 && edge_angles[u_in] <= IVP_PI * 1.4) {
	t = edge_angles[u_in] * 0.375;
	if (t >= anglo && t <= anghi) {
	    ++nang;
	    ang[nang - 1] = t;
	}
    }
L80:
    IVP_IF(1) {
	IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL3, "Candidate angles (dihedral angle = %f ", edge_angles[u_in] * 180.0 / IVP_PI);
	    for (i__ = 1; i__ <= nang; ++i__) {
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL3, "%f ", ang[i__ - 1] * 180.0 / IVP_PI);
	    }
	}
    }

/*     For each angle in ANG array, try to resolve reflex edge as */
/*     follows. Compute unit normal NRMLC and equation of cut plane; this */
/*     normal is outward wrt subpolyhedron containing face FL; NRMLC(4) */
/*     is right hand side constant term of plane equation. */
/*     Then determine which edges of polyhedron intersect cut plane; */
/*     reject plane if there is a vertex within distance MINDIS of plane */
/*     which does not lie on plane. */

    i__1 = nang;
    for (i__ = 1; i__ <= i__1; ++i__) {
	cn = cos(ang[i__ - 1]);
	if ((i__2 = facesdata[fr * 3 + 2], abs(i__2)) != p) {
	    cn = -cn;
	}
	ce = sin(ang[i__ - 1]);
	nrmlc[0] = cn * normals[fr * 3 + 1] + ce * enr[0];
	nrmlc[1] = cn * normals[fr * 3 + 2] + ce * enr[1];
	nrmlc[2] = cn * normals[fr * 3 + 3] + ce * enr[2];
	nrmlc[3] = nrmlc[0] * vcl[lu * 3 + 1] + nrmlc[1] * vcl[lu * 3 + 2] + 
		nrmlc[2] * vcl[lu * 3 + 3];
	nedgc = n_edges;
	j = 1;
L90:
	la = faceverticeslist[intworkarray[j * 3 + 1] * 6 + 1];
	da = nrmlc[0] * vcl[la * 3 + 1] + nrmlc[1] * vcl[la * 3 + 2] + nrmlc[
		2] * vcl[la * 3 + 3] - nrmlc[3];
	if (abs(da) <= dtol) {
	    ca = 0;
	} else if (abs(da) <= mindis) {
	    goto L110;
	} else if (da < 0.) {
	    ca = -1;
	} else {
	    ca = 1;
	}
	b = faceverticeslist[intworkarray[j * 3 + 1] * 6 + 3];
	lb = faceverticeslist[b * 6 + 1];
	db = nrmlc[0] * vcl[lb * 3 + 1] + nrmlc[1] * vcl[lb * 3 + 2] + nrmlc[
		2] * vcl[lb * 3 + 3] - nrmlc[3];
	if (abs(db) <= dtol) {
	    cb = 0;
	} else if (abs(db) <= mindis) {
	    goto L110;
	} else if (db < 0.) {
	    cb = -1;
	} else {
	    cb = 1;
	}
	if (ca * cb > 0) {
	    for (k = 1; k <= 2; ++k) {
		c__ = intworkarray[k + j * 3];
		intworkarray[k + j * 3] = intworkarray[k + nedgc * 3];
		intworkarray[k + nedgc * 3] = c__;
/* L100: */
	    }
	    --nedgc;
	} else {
	    intworkarray[j * 3 + 3] = (ca + 2) * 10 + (cb + 2);
	    ++j;
	}
	if (j <= nedgc) {
	    goto L90;
	}
	intworkarray[(n_edges + 1) * 3 + 1] = lvo;
	intworkarray[(n_edges + 1) * 3 + 3] = luo;
	intworkarray[(n_edges + 2) * 3 + 1] = u_in;
	doubleworkarray[1] = ang[i__ - 1];
	IVP_IF(1) {
	    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL3, "Trying angle %d   NEDGC = %d\n", i__, nedgc);
	    }
	}

	// *******************************************************************
	cutfac_(&p,
		nrmlc,
		&dtol,
		&nedgc,
		intworkarray,
		&nce,
		&intworkarray[((n_edges+1)*3)],
		&doubleworkarray[1],
		rflag
		);

	vcl = this->g_vcl;
	vcl -= 4;

	if (this->ierr != 0) {
	    return;
	}
	if (*rflag) {

	    // ***************************************************************
	    insfac_(&p,
		    nrmlc,
		    &nce,
		    &intworkarray[(n_edges+1)*3],
		    &doubleworkarray[1]
		    );

	    polyhedronfirstfaceoffset = this->g_polyhedronfirstfaceoffset;
	    --polyhedronfirstfaceoffset;

	    polyhedronfaceindices = this->g_polyhedronfaceindices;
	    polyhedronfaceindices -= 3;

	    facesdata = this->g_facesdata;
	    facesdata -= 4;

	    normals = this->g_normals;
	    normals -= 4;

	    faceverticeslist = this->g_faceverticeslist;
	    faceverticeslist -= 7;

	    edge_angles = this->g_edge_angles;
	    --edge_angles;

	    return;
	}
L110:
	;
    }

    return;
}

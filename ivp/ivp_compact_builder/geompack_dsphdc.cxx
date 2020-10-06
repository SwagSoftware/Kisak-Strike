/* dsphdc.f -- translated by f2c (version 19990311).
*/


#include <ivp_physics.hxx>
#include <ivp_betterdebugmanager.hxx>
#include <geompack.hxx>


void IVP_Geompack::dsphdc_() {

    double *vcl			      = this->g_vcl;
    int    *facesdata                 = this->g_facesdata;
    int    *faceverticeslist          = this->g_faceverticeslist;
    double *normals                   = this->g_normals;
    double *edge_angles               = this->g_edge_angles;
    int	   *polyhedronfirstfaceoffset = this->g_polyhedronfirstfaceoffset;
    int	   *polyhedronfaceindices     = this->g_polyhedronfaceindices;

    /* System generated locals */
    int i__1, i__2, i__3, i__4;
    double d__1, d__2, d__3;


    /* Local variables */
    double leng;
    int last;
    double dotp;
    int f, g, i__, j, k, l;
    long int fflag;
    int p;
    long int gflag;
    double ab[3], ac[3];
    int la, lb, lc;
    double en[3];
    int sf, hdfree;
    double pi2, ang;
    int ccw, nht;


/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Initialize the polyhedral decomposition data structure */
/*        where there are no holes on faces and no interior holes. It is */
/*        assumed head vertex of each face is a strictly convex vertex. */

/*     Input parameters: */
/*        NVC - number of vertex coordinates */
/*        NFACE - number of faces in polyhedral decomposition */
/*        NPOLH - number of polyhedra in decomposition */
/*        VCL(1:3,1:NVC) - vertex coordinate list */
/*        FACESDATA(1,1:NFACE+1) - head pointer to vertex indices in FACEVERTICESLIST for */
/*              each face; 1 = FACESDATA(1,1) < ... < FACESDATA(1,NFACE+1) */
/*        FACEVERTICESLIST(1,1:*) - vertex indices; those for Ith face are in FACEVERTICESLIST(1,J) */
/*              for J = FACESDATA(1,I),...,FACESDATA(1,I+1)-1 */
/*        POLYHEDRONFIRSTFACEOFFSET(1:NPOLH+1) - head pointer to face indices in POLYHEDRONFACEINDICES for each */
/*              polyhedron; 1 = POLYHEDRONFIRSTFACEOFFSET(1) < POLYHEDRONFIRSTFACEOFFSET(2) < ... < POLYHEDRONFIRSTFACEOFFSET(NPOLH+1) */
/*        POLYHEDRONFACEINDICES(1,1:*) - signed face indices; those for Ith polyh are in */
/*              POLYHEDRONFACEINDICES(1,J) for J = POLYHEDRONFIRSTFACEOFFSET(I),...,POLYHEDRONFIRSTFACEOFFSET(I+1)-1; the face index */
/*              must be negated if the ordering of vertices for the face */
/*              in FACEVERTICESLIST is in CW order when viewed from outside Ith polyh */
/*        HASHTABLE_SIZE - size of hash table HT; should be a prime number which */
/*              is >= NVC+2 */
/*        MAXEDG - maximum size available for EDGE array; should be at */
/*              least max number of edges in a polyh of decomposition */

/*     Output parameters: */
/*        FACESDATA(1:3,1:NFACE) - FACESDATA(1,F) same as input; FACESDATA(2,F) and */
/*              FACESDATA(3,F) are signed indices of 2 polyhedra sharing face */
/*              F; if F is boundary face then FACESDATA(3,F) = 0; the sign of */
/*              the polyh index indicates whether face is oriented CCW */
/*              (positive) or CW (negative) in FACEVERTICESLIST when viewed from */
/*              outside polyh; if interior face, 2 signs are different */
/*        NORMALS(1:3,1:NFACE) - normals at faces; NORMALS(*,F) is unit outward */
/*              normal of face F with its vertices oriented CCW when */
/*              viewed from outside polyhedron |FACESDATA(2,F)| */
/*        FACEVERTICESLIST(1:6,1:NVERT) - face vertex list where NVERT = FACESDATA(1, */
/*              NFACE+1)-1; 6 rows are for LOC, FACN, SUCC, PRED, EDGA, */
/*              EDGC; first 4 fields are the same as that used for the */
/*              convex polyhedron data structure (see routine DSCPH). */
/*              EDGA and EDGC give information about the edge UV where */
/*              V = FACEVERTICESLIST(SUCC,U). Let LU = FACEVERTICESLIST(LOC,U), LV = FACEVERTICESLIST(LOC,V), */
/*              and SF = +1 (-1) if face containing UV in polyh P is */
/*              oriented CCW (CW) when viewed from outside P. Let WX be */
/*              edge corresponding to UV in the adjacent face of P, where */
/*              X = FACEVERTICESLIST(SUCC,W). If (LV-LU)*SF > 0, then FACEVERTICESLIST(EDGC,U) = W, */
/*              FACEVERTICESLIST(EDGA,W) = U, and EDGE_ANGLES(U) is angle at UV between the */
/*              2 faces inside P; else FACEVERTICESLIST(EDGA,U) = W, FACEVERTICESLIST(EDGC,W) = U, */
/*              and EDGE_ANGLES(W) is the edge angle. In other words, if P is */
/*              viewed from outside with edge UV directed upwards from */
/*              vertex with smaller LOC value to other vertex, then there */
/*              is a CCW or CW rotation in P from face containing UV to */
/*              other face as indicated by EDGA or EDGC, respectively (A */
/*              for AntiCW, C for CW). If the CCW or CW rotation between */
/*              2 faces is exterior to the region, then the EDGA or EDGC */
/*              value is 0 and EDGE_ANGLES value is -1. */
/*        EDGE_ANGLES(1:NVERT) - angles at edges common to 2 faces in a polyh; */
/*              EDGE_ANGLES(J) corresponds to FACEVERTICESLIST(*,J) and is determined by */
/*              EDGC field */
/*        POLYHEDRONFACEINDICES(1:2,1:NPF) - row 1 same as input and row 2 used for link, */
/*              where NPF = POLYHEDRONFIRSTFACEOFFSET(NPOLH+1)-1 */

/*     Working parameters: */
/*        HT(0:HASHTABLE_SIZE-1),EDGE(1:4,1:MAXEDG) - hash table and edge records */
/*              used to determine matching occurrences of polyhedron */
/*              edges by calling routine EDGHT */

/*     Abnormal return: */
/*        IERR is set to 1, 321, or 322 */

/*     Routines called: */
/*        EDGHT */



    /* Parameter adjustments */
    vcl -= 4;
    normals -= 4;
    facesdata -= 4;
    --polyhedronfirstfaceoffset;
    faceverticeslist -= 7;
    --edge_angles;
    polyhedronfaceindices -= 3;

    /* Function Body */
    pi2 = IVP_PI * 2.;
    hdfree = 0;
    last = 0;
    nht = 0;
    i__1 = this->hashtable_size - 1;
    for (i__ = 0; i__ <= i__1; ++i__) {
	this->g_hashtable[i__] = 0;
/* L10: */
    }
    i__1 = this->nface;
    for (i__ = 1; i__ <= i__1; ++i__) {
	facesdata[i__ * 3 + 2] = 0;
	facesdata[i__ * 3 + 3] = 0;
	k = facesdata[i__ * 3 + 1];
	l = facesdata[(i__ + 1) * 3 + 1] - 1;
	i__2 = l;
	for (j = k; j <= i__2; ++j) {
	    faceverticeslist[j * 6 + 2] = i__;
	    faceverticeslist[j * 6 + 3] = j + 1;
	    faceverticeslist[j * 6 + 4] = j - 1;
	    faceverticeslist[j * 6 + 5] = 0;
	    faceverticeslist[j * 6 + 6] = 0;
	    edge_angles[j] = -1.;
/* L20: */
	}
	faceverticeslist[l * 6 + 3] = k;
	faceverticeslist[k * 6 + 4] = l;
/* L30: */
    }
    i__1 = this->npolh;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = polyhedronfirstfaceoffset[i__];
	l = polyhedronfirstfaceoffset[i__ + 1] - 1;
	i__2 = l;
	for (j = k; j <= i__2; ++j) {
	    polyhedronfaceindices[(j << 1) + 2] = j + 1;
	    f = polyhedronfaceindices[(j << 1) + 1];
	    p = i_sign(i__, f);
	    f = abs(f);
	    if (facesdata[f * 3 + 2] == 0) {
		facesdata[f * 3 + 2] = p;
	    } else {
		facesdata[f * 3 + 3] = p;
	    }
/* L40: */
	}
	polyhedronfaceindices[(l << 1) + 2] = k;
/* L50: */
    }
    i__1 = this->nface;
    for (f = 1; f <= i__1; ++f) {
	if (facesdata[f * 3 + 2] * facesdata[f * 3 + 3] > 0) {
	    this->ierr = 321;
	    IVP_IF(1) {
		IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
		    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "*** GEOMPACK: face oriented same way twice in routine DSPHDC\n");
		}
	    }
	    return;
	}
/* L60: */
    }

/*     Compute normals for each face from orientation in FACESDATA(2,*). */

    i__1 = this->nface;
    for (f = 1; f <= i__1; ++f) {
	if (facesdata[f * 3 + 2] > 0) {
	    ccw = 3;
	} else {
	    ccw = 4;
	}
	j = facesdata[f * 3 + 1];
	lb = faceverticeslist[j * 6 + 1];
	lc = faceverticeslist[faceverticeslist[ccw + j * 6] * 6 + 1];
	la = faceverticeslist[faceverticeslist[7 - ccw + j * 6] * 6 + 1];
	for (i__ = 1; i__ <= 3; ++i__) {
	    ab[i__ - 1] = vcl[i__ + lb * 3] - vcl[i__ + la * 3];
	    ac[i__ - 1] = vcl[i__ + lc * 3] - vcl[i__ + la * 3];
/* L70: */
	}
	normals[f * 3 + 1] = ab[1] * ac[2] - ab[2] * ac[1];
	normals[f * 3 + 2] = ab[2] * ac[0] - ab[0] * ac[2];
	normals[f * 3 + 3] = ab[0] * ac[1] - ab[1] * ac[0];
/* Computing 2nd power */
	d__1 = normals[f * 3 + 1];
/* Computing 2nd power */
	d__2 = normals[f * 3 + 2];
/* Computing 2nd power */
	d__3 = normals[f * 3 + 3];
	leng = sqrt(d__1 * d__1 + d__2 * d__2 + d__3 * d__3);
	if (leng > 0.) {
	    normals[f * 3 + 1] /= leng;
	    normals[f * 3 + 2] /= leng;
	    normals[f * 3 + 3] /= leng;
	}
/* L80: */
    }

/*     Determine EDGA, EDGC fields and compute EDGE_ANGLES values. */

    i__1 = this->npolh;
    for (p = 1; p <= i__1; ++p) {
	nht = 0;
	i__2 = polyhedronfirstfaceoffset[p + 1] - 1;
	for (i__ = polyhedronfirstfaceoffset[p]; i__ <= i__2; ++i__) {
	    sf = polyhedronfaceindices[(i__ << 1) + 1];
	    f = abs(sf);
	    i__3 = facesdata[(f + 1) * 3 + 1] - 1;
	    for (j = facesdata[f * 3 + 1]; j <= i__3; ++j) {
		la = faceverticeslist[j * 6 + 1];
		lb = faceverticeslist[faceverticeslist[j * 6 + 3] * 6 + 1];

		// ***********************************************************
		edght_(&la,
		       &lb,
		       &j,
		       &hdfree,
		       &last,
		       &k
		       );

		if (this->ierr != 0) {
		    return;
		}
		if (k <= 0) {
		    ++nht;
		} else {
		    --nht;
		    g = faceverticeslist[k * 6 + 2];
		    dotp = normals[f * 3 + 1] * normals[g * 3 + 1] + normals[f * 3 + 2]
			     * normals[g * 3 + 2] + normals[f * 3 + 3] * normals[g * 3 
			    + 3];
		    if (abs(dotp) > 1. - this->tolerance) {
			dotp = d_sign(1.0, dotp);
		    }
		    fflag = (i__4 = facesdata[f * 3 + 2], abs(i__4)) == p;
		    gflag = (i__4 = facesdata[g * 3 + 2], abs(i__4)) == p;
		    if (fflag != gflag) {
			dotp = -dotp;
		    }
		    ang = IVP_PI - acos(dotp);
/*                 Determine whether edge angle is reflex. */
		    for (l = 1; l <= 3; ++l) {
			ab[l - 1] = vcl[l + lb * 3] - vcl[l + la * 3];
/* L90: */
		    }
		    en[0] = normals[f * 3 + 2] * ab[2] - normals[f * 3 + 3] * ab[1];
		    en[1] = normals[f * 3 + 3] * ab[0] - normals[f * 3 + 1] * ab[2];
		    en[2] = normals[f * 3 + 1] * ab[1] - normals[f * 3 + 2] * ab[0];
		    if (fflag != sf > 0) {
			en[0] = -en[0];
			en[1] = -en[1];
			en[2] = -en[2];
		    }
/*                 AC = (midpoint of A and B) + EN - A */
		    for (l = 1; l <= 3; ++l) {
			ac[l - 1] = (vcl[l + lb * 3] - vcl[l + la * 3]) * .5 
				+ en[l - 1];
/* L100: */
		    }
		    dotp = ac[0] * normals[g * 3 + 1] + ac[1] * normals[g * 3 + 2] 
			    + ac[2] * normals[g * 3 + 3];
		    if (! gflag) {
			dotp = -dotp;
		    }
		    if (dotp > 0.) {
			ang = pi2 - ang;
		    }
		    if ((lb - la) * sf > 0) {
			faceverticeslist[j * 6 + 6] = k;
			faceverticeslist[k * 6 + 5] = j;
			edge_angles[j] = ang;
		    } else {
			faceverticeslist[j * 6 + 5] = k;
			faceverticeslist[k * 6 + 6] = j;
			edge_angles[k] = ang;
		    }
		}
/* L110: */
	    }
/* L120: */
	}
	if (nht != 0) {
	    this->ierr = 322;
	    IVP_IF(1) {
		IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
		    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "*** GEOMPACK: unmatched edge determined by routine DSPHDC\n");
		}
	    }
	    return;
	}
/* L130: */
    }

    return;
}

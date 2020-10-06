/* cutfac.f -- translated by f2c (version 19990311).
*/


#include <ivp_physics.hxx>
#include <ivp_betterdebugmanager.hxx>
#include <geompack.hxx>


void IVP_Geompack::cutfac_(
			   int		*p,
			   double	*nrmlc,
			   double	*dtol,
			   int		*nedgc,
			   int		*pedge,
			   int		*nce,
			   int		*cedge,
			   double	*cdang,
			   long int	*rflag
			  ) {

    double *vcl              = this->g_vcl;
    int    *facesdata        = this->g_facesdata;
    double *normals          = this->g_normals;
    int    *faceverticeslist = this->g_faceverticeslist;
    double *edge_angles      = this->g_edge_angles;

    /* System generated locals */
    int i__1, i__2;
    double d__1, d__2, d__3, d__4;


    /* Local variables */
    double angr, cmax;
    int imin, kmax;
    double dist, nmax, dotp, tmin, ntol;
    int a, e, f, i__, j, k, l, n;
    long int eflag;
    double s, t;
    int w;
    double iamin;
    int ccwfl;
    double dsave[3];
    int isave;
    double dirsq;
    int estop, inout, estrt;
    double dir1sq;
    int ca, cb;
    double de[3];
    int ee, la, lb, fl;
    double cp[3];
    int fr, sf, lv, lw, sp;
    double intang;
    double pi2;
    int lw1;
    double dee[3], ang;
    long int dof;
    double dir[3];
    int nev, sgn;
    double rhs[3], dir1[3];


/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Trace out cut face in polyhedron P given a starting edge, */
/*        e.g. a reflex edge or an edge in the interior of a face. */
/*        Accept cut face if it creates no small angles, it is an outer */
/*        boundary and there are no holes (inner polygons) in it. It is */
/*        assumed starting edge does not lie on a double-occurring face. */

/*     Input parameters: */
/*        P - polyhedron index */
/*        NRMLC(1:4) - unit normal vector of cut plane plus right hand */
/*              side constant term of plane equation */
/*        ANGACC - min acceptable dihedral angle in radians produced by */
/*              a cut face */
/*        DTOL - absolute tolerance to determine if point is on cut plane */
/*        ADDR_OF_N_ORIGINAL_VERTICES - number of vertex coordinates */
/*        SIZE_VCL - maximum size available for VCL array */
/*        VCL(1:3,1:ADDR_OF_N_ORIGINAL_VERTICES) - vertex coordinate list */
/*        FACESDATA(1:3,1:*) - face pointer list */
/*        NRML(1:3,1:*) - unit normal vectors for faces */
/*        FACEVERTICESLIST(1:6,1:*) - face vertex list */
/*        EDGE_ANGLES(1:*) - edge angles */
/*        NEDGC - number of edges which intersect cut plane */
/*        PEDGE(1:3,1:NEDGC) - edges of polyh P that intersect cut plane */
/*              excluding starting edge if it is an edge of P or the edge */
/*              containing CEDGE(1,1) if CEDGE(1,1) > ADDR_OF_N_ORIGINAL_VERTICES; */
/*              PEDGE(1,I), PEDGE(2,I) are indices of FACEVERTICESLIST; if PEDGE(1,I) */
/*              = A and B = FACEVERTICESLIST(SUCC,A) then PEDGE(3,I) = 10*CA+CB where */
/*              CA = 1, 2, or 3 if vertex A in negative half-space, on */
/*              cut plane, or in positive half-space determined by cut */
/*              plane, and similarly for CB */
/*        CEDGE(1:2,0:1) - CEDGE(1,0) = LV, CEDGE(1,1) = LU where LU, LV */
/*              are indices of VCL and are vertices of starting edge; if */
/*              called by RESEDG, LU < LV <= ADDR_OF_N_ORIGINAL_VERTICES are on reflex edge and */
/*              CEDGE(2,1) = U is index of FACEVERTICESLIST indicating reflex edge; */
/*              LV may be ADDR_OF_N_ORIGINAL_VERTICES+1 and LU may be ADDR_OF_N_ORIGINAL_VERTICES+1 or ADDR_OF_N_ORIGINAL_VERTICES+2 to indicate */
/*              a point on interior of an edge; CEDGE(2,1) is specified */
/*              as described for output below; if LV > ADDR_OF_N_ORIGINAL_VERTICES, CEDGE(2,0) */
/*              takes on the value ABS(CEDGE(2,NCE)) described for output */
/*              below, else CEDGE(2,0) is not used */
/*        CDANG(1) - dihedral angle at starting edge determined by cut */
/*              plane in positive half-space */

/*     Updated parameters: */
/*        VCL - some temporary or permanent entries may be added at end */
/*              of array */
/*        NEDGC - may be decreased to indicate number of unchecked edges */
/*        PEDGE(1:3,1:NEDGC) - columns may be permuted (NEDGC=input val) */

/*     Output parameters: */
/*        NCE - number of edges in cut polygon; it is assumed there is */
/*              enough space in the following two arrays */
/*        CEDGE(1:2,0:NCE) - CEDGE(1,I) is an index of VCL, indices > ADDR_OF_N_ORIGINAL_VERTICES */
/*              are new points; CEDGE(2,I) = J indicates that edge of cut */
/*              face ending at CEDGE(1,I) is edge from J to FACEVERTICESLIST(SUCC,J) */
/*              if J > 0; else if J < 0 then edge of cut face ending at */
/*              CEDGE(1,I) is a new edge and CEDGE(1,I) lies on edge from */
/*              -J to FACEVERTICESLIST(SUC,-J) and new edge lies in face FACEVERTICESLIST(FACN,-J); */
/*              CEDGE(2,I) always refers to an edge in the subpolyhedron */
/*              in negative half-space; CEDGE(1,NCE) = CEDGE(1,0) */
/*        CDANG(1:NCE) - dihedral angles created by edges of cut polygon */
/*              in pos. half-space; neg. sign for angle I indicates that */
/*              face containing edge I is oriented CW in polyhedron P */
/*        RFLAG - .TRUE. iff reflex or starting edge is resolved */

/*     Routines called: */
/*        PTPOLG */

/*     Abnormal return: */
/*        IERR is set to 14, 325, 326, or 328 */




    /* Parameter adjustments */
    --nrmlc;
    vcl -= 4;
    facesdata -= 4;
    normals -= 4;
    faceverticeslist -= 7;
    --edge_angles;
    --cdang;

    /* Function Body */
    *rflag = 0;
    pi2 = IVP_PI + IVP_PI;
/* Computing MAX */
    i__1 = max(this->n_original_vertices, cedge[1]);
    n = max(i__1,cedge[3]);
    *nce = 1;
    w = cedge[4];
    lv = cedge[1];
    lw = lv;
    lw1 = cedge[3];
    if (lw1 <= this->n_original_vertices ) {
	nev = 1;
	this->g_ev[nev - 1] = lw1;
    } else {
	nev = 0;
    }
    if (w > 0) {
	fl = faceverticeslist[w * 6 + 2];
	if (lw > lw1) {
	    fr = faceverticeslist[faceverticeslist[w * 6 + 6] * 6 + 2];
	} else {
	    fr = faceverticeslist[faceverticeslist[w * 6 + 5] * 6 + 2];
	}
    } else {
	fl = faceverticeslist[-w * 6 + 2];
	fr = fl;
    }
    dir[0] = vcl[lw1 * 3 + 1] - vcl[lw * 3 + 1];
    dir[1] = vcl[lw1 * 3 + 2] - vcl[lw * 3 + 2];
    dir[2] = vcl[lw1 * 3 + 3] - vcl[lw * 3 + 3];
    kmax = 1;
    if (abs(nrmlc[2]) > abs(nrmlc[1])) {
	kmax = 2;
    }
    if (abs(nrmlc[3]) > (d__1 = nrmlc[kmax], abs(d__1))) {
	kmax = 3;
    }
    if ((i__1 = facesdata[fl * 3 + 2], abs(i__1)) == *p) {
	ccwfl = facesdata[fl * 3 + 2];
    } else {
	ccwfl = facesdata[fl * 3 + 3];
    }
    if (ccwfl < 0) {
	cdang[1] = -cdang[1];
    }

/*     LW, LW1, FL, FR, DIR are updated before each iteration of loop. */
/*     CCWFL = P (-P) if FL is CCW (CW) according to SUCC traversal. */

L10:
    if (lw1 > this->n_original_vertices ) {

/*           LW1 is new vertex on interior of edge E. FL is used for */
/*           previous and next faces containing edges of cut polygon. */

	e = -cedge[(*nce << 1) + 2];
	la = faceverticeslist[e * 6 + 1];
	lb = faceverticeslist[faceverticeslist[e * 6 + 3] * 6 + 1];
	if ((lb - la) * ccwfl > 0) {
	    ee = faceverticeslist[e * 6 + 6];
	} else {
	    ee = faceverticeslist[e * 6 + 5];
	}
	fl = faceverticeslist[ee * 6 + 2];
	dof = (i__1 = facesdata[fl * 3 + 2], abs(i__1)) == (i__2 = facesdata[fl * 3 + 
		3], abs(i__2));
	if (dof) {
	    l = faceverticeslist[ee * 6 + 1];
	    if (l == la) {
		ccwfl = -ccwfl;
	    }
	} else if ((i__1 = facesdata[fl * 3 + 2], abs(i__1)) == *p) {
	    ccwfl = facesdata[fl * 3 + 2];
	} else {
	    ccwfl = facesdata[fl * 3 + 3];
	}
	dir[0] = nrmlc[2] * normals[fl * 3 + 3] - nrmlc[3] * normals[fl * 3 + 2];
	dir[1] = nrmlc[3] * normals[fl * 3 + 1] - nrmlc[1] * normals[fl * 3 + 3];
	dir[2] = nrmlc[1] * normals[fl * 3 + 2] - nrmlc[2] * normals[fl * 3 + 1];
	if ((i__1 = facesdata[fl * 3 + 2], abs(i__1)) != *p || dof && ccwfl < 0) {
	    dir[0] = -dir[0];
	    dir[1] = -dir[1];
	    dir[2] = -dir[2];
	}
	goto L70;
    } else {

/*           LW1 is existing vertex of polyhedron P. FL (FL and FR) is */
/*           previous face if edge ending at LW1 is new (already exists). */
/*           In former case, -CEDGE(2,NCE) is the edge of FL incident on */
/*           LW1 which will lie only in subpolyhedron PL. In latter case, */
/*           CEDGE(2,NCE) is an edge of FL. Cycle thru edges, faces CCW */
/*           (from outside) between edges ESTRT, ESTOP. */
/*           If LW1 lies on a doubly-occurring face, there are 2 cycles */
/*           around LW1 and the correct one is chosen based on CCWFL. */

	iamin = pi2;
	imin = 0;
/* Computing 2nd power */
	d__1 = dir[0];
/* Computing 2nd power */
	d__2 = dir[1];
/* Computing 2nd power */
	d__3 = dir[2];
	dirsq = d__1 * d__1 + d__2 * d__2 + d__3 * d__3;
	eflag = cedge[(*nce << 1) + 2] > 0;
	if (! eflag) {
	    estrt = -cedge[(*nce << 1) + 2];
	    sp = ccwfl;
	    if (ccwfl > 0) {
		estop = faceverticeslist[estrt * 6 + 3];
	    } else {
		estop = faceverticeslist[estrt * 6 + 4];
	    }
	} else {
	    w = cedge[(*nce << 1) + 2];
	    la = faceverticeslist[w * 6 + 1];
	    if (ccwfl > 0) {
		estrt = faceverticeslist[w * 6 + 4];
	    } else {
		estrt = faceverticeslist[w * 6 + 3];
	    }
	    if (la == lw) {
		l = lw1 - lw;
	    } else {
		l = lw - lw1;
	    }
	    if (l * ccwfl > 0) {
		w = faceverticeslist[w * 6 + 6];
	    } else {
		w = faceverticeslist[w * 6 + 5];
	    }
	    if ((i__1 = facesdata[fr * 3 + 2], abs(i__1)) == (i__2 = facesdata[fr * 3 
		    + 3], abs(i__2))) {
		lb = faceverticeslist[w * 6 + 1];
		if (la == lb) {
		    sp = -ccwfl;
		} else {
		    sp = ccwfl;
		}
	    } else if ((i__1 = facesdata[fr * 3 + 2], abs(i__1)) == *p) {
		sp = facesdata[fr * 3 + 2];
	    } else {
		sp = facesdata[fr * 3 + 3];
	    }
	    if (sp > 0) {
		estop = faceverticeslist[w * 6 + 3];
	    } else {
		estop = faceverticeslist[w * 6 + 4];
	    }
	}
	la = faceverticeslist[estop * 6 + 1];
	lb = faceverticeslist[faceverticeslist[estop * 6 + 3] * 6 + 1];
	if ((lb - la) * sp > 0) {
	    estop = faceverticeslist[estop * 6 + 6];
	} else {
	    estop = faceverticeslist[estop * 6 + 5];
	}
	e = estrt;
	sf = ccwfl;
L20:
	if (eflag || e != estrt && e != estop) {
	    if (faceverticeslist[e * 6 + 1] == lw1) {
		l = faceverticeslist[faceverticeslist[e * 6 + 3] * 6 + 1];
	    } else {
		l = faceverticeslist[e * 6 + 1];
	    }
	    dist = nrmlc[1] * vcl[l * 3 + 1] + nrmlc[2] * vcl[l * 3 + 2] + 
		    nrmlc[3] * vcl[l * 3 + 3] - nrmlc[4];
	    if (abs(dist) <= *dtol) {
		dir1[0] = vcl[l * 3 + 1] - vcl[lw1 * 3 + 1];
		dir1[1] = vcl[l * 3 + 2] - vcl[lw1 * 3 + 2];
		dir1[2] = vcl[l * 3 + 3] - vcl[lw1 * 3 + 3];
/* Computing 2nd power */
		d__1 = dir1[0];
/* Computing 2nd power */
		d__2 = dir1[1];
/* Computing 2nd power */
		d__3 = dir1[2];
		dir1sq = d__1 * d__1 + d__2 * d__2 + d__3 * d__3;
		dotp = -(dir[0] * dir1[0] + dir[1] * dir1[1] + dir[2] * dir1[
			2]) / sqrt(dirsq * dir1sq);
		if (abs(dotp) > 1. - this->tolerance) {
		    dotp = d_sign(1.0, dotp);
		}
		if (kmax == 1) {
		    cp[0] = dir[1] * dir1[2] - dir[2] * dir1[1];
		} else if (kmax == 2) {
		    cp[1] = dir[2] * dir1[0] - dir[0] * dir1[2];
		} else {
		    cp[2] = dir[0] * dir1[1] - dir[1] * dir1[0];
		}
/* Computing MAX */
		d__2 = abs(dir[0]), d__3 = abs(dir[1]), d__2 = max(d__2,d__3),
			 d__3 = abs(dir[2]), d__2 = max(d__2,d__3), d__3 = 
			abs(dir1[0]), d__2 = max(d__2,d__3), d__3 = abs(dir1[
			1]), d__2 = max(d__2,d__3), d__3 = abs(dir1[2]);
		if ((d__1 = cp[kmax - 1], abs(d__1)) <= this->tolerance * max(
			d__2,d__3)) {
		    intang = IVP_PI;
		} else if (cp[kmax - 1] * nrmlc[kmax] > 0.) {
		    intang = acos(dotp);
		} else {
		    intang = pi2 - acos(dotp);
		}
		if (intang < iamin) {
		    iamin = intang;
		    imin = e;
		    dsave[0] = dir1[0];
		    dsave[1] = dir1[1];
		    dsave[2] = dir1[2];
		}
	    }
	}

	if (e == estop) {
	    goto L40;
	}
	la = faceverticeslist[e * 6 + 1];
	lb = faceverticeslist[faceverticeslist[e * 6 + 3] * 6 + 1];
	if ((lb - la) * sf > 0) {
	    e = faceverticeslist[e * 6 + 6];
	} else {
	    e = faceverticeslist[e * 6 + 5];
	}
	f = faceverticeslist[e * 6 + 2];
	dof = (i__1 = facesdata[f * 3 + 2], abs(i__1)) == (i__2 = facesdata[f * 3 + 3]
		, abs(i__2));
	if (dof) {
	    l = faceverticeslist[e * 6 + 1];
	    if (l == la) {
		sf = -sf;
	    }
	} else if ((i__1 = facesdata[f * 3 + 2], abs(i__1)) == *p) {
	    sf = facesdata[f * 3 + 2];
	} else {
	    sf = facesdata[f * 3 + 3];
	}
	if (sf > 0) {
	    ee = faceverticeslist[e * 6 + 4];
	    la = faceverticeslist[faceverticeslist[e * 6 + 3] * 6 + 1];
	    lb = faceverticeslist[ee * 6 + 1];
	} else {
	    ee = faceverticeslist[e * 6 + 3];
	    la = faceverticeslist[e * 6 + 1];
	    lb = faceverticeslist[faceverticeslist[ee * 6 + 3] * 6 + 1];
	}
	dir1[0] = nrmlc[2] * normals[f * 3 + 3] - nrmlc[3] * normals[f * 3 + 2];
	dir1[1] = nrmlc[3] * normals[f * 3 + 1] - nrmlc[1] * normals[f * 3 + 3];
	dir1[2] = nrmlc[1] * normals[f * 3 + 2] - nrmlc[2] * normals[f * 3 + 1];
/* Computing MAX */
	d__1 = abs(dir1[0]), d__2 = abs(dir1[1]), d__1 = max(d__1,d__2), d__2 
		= abs(dir1[2]);
	if (max(d__1,d__2) <= this->tolerance) {
	    goto L30;
	}
	sgn = 1;
	if ((i__1 = facesdata[f * 3 + 2], abs(i__1)) != *p || dof && sf < 0) {
	    dir1[0] = -dir1[0];
	    dir1[1] = -dir1[1];
	    dir1[2] = -dir1[2];
	    sgn = -1;
	}
	k = 1;
	if ((d__1 = normals[f * 3 + 2], abs(d__1)) > (d__2 = normals[f * 3 + 1], 
		abs(d__2))) {
	    k = 2;
	}
	if ((d__1 = normals[f * 3 + 3], abs(d__1)) > (d__2 = normals[k + f * 3], 
		abs(d__2))) {
	    k = 3;
	}
	nmax = sgn * normals[k + f * 3];
	de[0] = vcl[la * 3 + 1] - vcl[lw1 * 3 + 1];
	de[1] = vcl[la * 3 + 2] - vcl[lw1 * 3 + 2];
	de[2] = vcl[la * 3 + 3] - vcl[lw1 * 3 + 3];
	dee[0] = vcl[lb * 3 + 1] - vcl[lw1 * 3 + 1];
	dee[1] = vcl[lb * 3 + 2] - vcl[lw1 * 3 + 2];
	dee[2] = vcl[lb * 3 + 3] - vcl[lw1 * 3 + 3];
/* Computing MAX */
	d__1 = abs(de[0]), d__2 = abs(de[1]), d__1 = max(d__1,d__2), d__2 = 
		abs(de[2]), d__1 = max(d__1,d__2), d__2 = abs(dee[0]), d__1 = 
		max(d__1,d__2), d__2 = abs(dee[1]), d__1 = max(d__1,d__2), 
		d__2 = abs(dee[2]);
	ntol = this->tolerance * max(d__1,d__2);
	if (k == 1) {
	    cp[0] = de[1] * dee[2] - de[2] * dee[1];
	} else if (k == 2) {
	    cp[1] = de[2] * dee[0] - de[0] * dee[2];
	} else {
	    cp[2] = de[0] * dee[1] - de[1] * dee[0];
	}
	if ((d__1 = cp[k - 1], abs(d__1)) <= ntol || cp[k - 1] * nmax > 0.) {
	    if (k == 1) {
		cp[0] = de[1] * dir1[2] - de[2] * dir1[1];
	    }
	    if (k == 2) {
		cp[1] = de[2] * dir1[0] - de[0] * dir1[2];
	    }
	    if (k == 3) {
		cp[2] = de[0] * dir1[1] - de[1] * dir1[0];
	    }
	    if ((d__1 = cp[k - 1], abs(d__1)) <= ntol || cp[k - 1] * nmax < 
		    0.) {
		goto L30;
	    }
	    if (k == 1) {
		cp[0] = dir1[1] * dee[2] - dir1[2] * dee[1];
	    }
	    if (k == 2) {
		cp[1] = dir1[2] * dee[0] - dir1[0] * dee[2];
	    }
	    if (k == 3) {
		cp[2] = dir1[0] * dee[1] - dir1[1] * dee[0];
	    }
	    if ((d__1 = cp[k - 1], abs(d__1)) <= ntol || cp[k - 1] * nmax < 
		    0.) {
		goto L30;
	    }
	} else {
	    if (k == 1) {
		cp[0] = dir1[1] * de[2] - dir1[2] * de[1];
	    }
	    if (k == 2) {
		cp[1] = dir1[2] * de[0] - dir1[0] * de[2];
	    }
	    if (k == 3) {
		cp[2] = dir1[0] * de[1] - dir1[1] * de[0];
	    }
	    if ((d__1 = cp[k - 1], abs(d__1)) <= ntol || cp[k - 1] * nmax > 
		    0.) {
		if (k == 1) {
		    cp[0] = dee[1] * dir1[2] - dee[2] * dir1[1];
		}
		if (k == 2) {
		    cp[1] = dee[2] * dir1[0] - dee[0] * dir1[2];
		}
		if (k == 3) {
		    cp[2] = dee[0] * dir1[1] - dee[1] * dir1[0];
		}
		if ((d__1 = cp[k - 1], abs(d__1)) <= ntol || cp[k - 1] * nmax 
			> 0.) {
		    goto L30;
		}
	    }
	}
/* Computing 2nd power */
	d__1 = dir1[0];
/* Computing 2nd power */
	d__2 = dir1[1];
/* Computing 2nd power */
	d__3 = dir1[2];
	dir1sq = d__1 * d__1 + d__2 * d__2 + d__3 * d__3;
	dotp = -(dir[0] * dir1[0] + dir[1] * dir1[1] + dir[2] * dir1[2]) / 
		sqrt(dirsq * dir1sq);
	if (abs(dotp) > 1. - this->tolerance) {
	    dotp = d_sign(1.0, dotp);
	}
	if (kmax == 1) {
	    cp[0] = dir[1] * dir1[2] - dir[2] * dir1[1];
	} else if (kmax == 2) {
	    cp[1] = dir[2] * dir1[0] - dir[0] * dir1[2];
	} else {
	    cp[2] = dir[0] * dir1[1] - dir[1] * dir1[0];
	}
/* Computing MAX */
	d__2 = abs(dir[0]), d__3 = abs(dir[1]), d__2 = max(d__2,d__3), d__3 = 
		abs(dir[2]), d__2 = max(d__2,d__3), d__3 = abs(dir1[0]), d__2 
		= max(d__2,d__3), d__3 = abs(dir1[1]), d__2 = max(d__2,d__3), 
		d__3 = abs(dir1[2]);
	if ((d__1 = cp[kmax - 1], abs(d__1)) <= this->tolerance * max(d__2,d__3))
		 {
	    intang = IVP_PI;
	} else if (cp[kmax - 1] * nrmlc[kmax] > 0.) {
	    intang = acos(dotp);
	} else {
	    intang = pi2 - acos(dotp);
	}
	if (intang < iamin) {
	    iamin = intang;
	    imin = -f;
	    ccwfl = sf;
	    dsave[0] = dir1[0];
	    dsave[1] = dir1[1];
	    dsave[2] = dir1[2];
	}
L30:
	e = ee;
	goto L20;

L40:
	if (imin == 0) {
/* 	       IERR = 325 */
	    return;
	} else if (imin > 0) {
	    dir[0] = dsave[0];
	    dir[1] = dsave[1];
	    dir[2] = dsave[2];
	    lw = lw1;
	    la = faceverticeslist[imin * 6 + 1];
	    lb = faceverticeslist[faceverticeslist[imin * 6 + 3] * 6 + 1];
	    if (la == lw1) {
		lw1 = lb;
	    } else {
		lw1 = la;
	    }
	    ++(*nce);
	    cedge[(*nce << 1) + 1] = lw1;
	    cedge[(*nce << 1) + 2] = imin;
	    fl = faceverticeslist[imin * 6 + 2];
	    dof = (i__1 = facesdata[fl * 3 + 2], abs(i__1)) == (i__2 = facesdata[fl * 
		    3 + 3], abs(i__2));
	    if (dof) {
		if (la == lw) {
		    ccwfl = -(*p);
		} else {
		    ccwfl = *p;
		}
	    } else if ((i__1 = facesdata[fl * 3 + 2], abs(i__1)) == *p) {
		ccwfl = facesdata[fl * 3 + 2];
	    } else {
		ccwfl = facesdata[fl * 3 + 3];
	    }
	    if ((lb - la) * ccwfl > 0) {
		fr = faceverticeslist[faceverticeslist[imin * 6 + 6] * 6 + 2];
	    } else {
		fr = faceverticeslist[faceverticeslist[imin * 6 + 5] * 6 + 2];
	    }
	    k = 1;
L50:
	    if (pedge[k * 3 + 1] == imin || pedge[k * 3 + 2] == imin) {
		for (i__ = 1; i__ <= 3; ++i__) {
		    j = pedge[i__ + k * 3];
		    pedge[i__ + k * 3] = pedge[i__ + *nedgc * 3];
		    pedge[i__ + *nedgc * 3] = j;
/* L60: */
		}
		--(*nedgc);
	    } else {
		++k;
		goto L50;
	    }
	    goto L110;
	} else {
	    dir[0] = dsave[0];
	    dir[1] = dsave[1];
	    dir[2] = dsave[2];
	    fl = -imin;
	    goto L70;
	}
    }

/*        Determine LW1 from direction DIR in interior of face FL. */

L70:
    lw = lw1;
    fr = 0;
    imin = 0;
    tmin = 0.;
    k = 1;
    if (abs(dir[1]) > abs(dir[0])) {
	k = 2;
    }
    if (abs(dir[2]) > (d__1 = dir[k - 1], abs(d__1))) {
	k = 3;
    }
    ntol = this->tolerance * (d__1 = dir[k - 1], abs(d__1));
    i__1 = *nedgc;
    for (i__ = 1; i__ <= i__1; ++i__) {
	e = pedge[i__ * 3 + 1];
	ee = pedge[i__ * 3 + 2];
	if (faceverticeslist[e * 6 + 2] == fl) {
	    a = e;
	} else if (faceverticeslist[ee * 6 + 2] == fl) {
	    a = ee;
	} else {
	    goto L80;
	}
	ca = pedge[i__ * 3 + 3] / 10;
	cb = pedge[i__ * 3 + 3] % 10;
	if (ca == 2) {
	    la = faceverticeslist[a * 6 + 1];
	    if (cb == 2) {
		lb = faceverticeslist[faceverticeslist[a * 6 + 3] * 6 + 1];
		s = (vcl[k + la * 3] - vcl[k + lw * 3]) / dir[k - 1];
		t = (vcl[k + lb * 3] - vcl[k + lw * 3]) / dir[k - 1];
		if (s > 0.) {
		    if (min(s,t) < tmin || imin == 0) {
			if (s < t) {
			    if (ccwfl < 0) {
				imin = a;
				lw1 = la;
				tmin = s;
			    }
			} else {
			    if (ccwfl > 0) {
				imin = a;
				lw1 = lb;
				tmin = t;
			    }
			}
		    }
		}
	    } else {
		l = faceverticeslist[e * 6 + 1];
		if (l == la && ccwfl < 0 || l != la && ccwfl > 0) {
		    t = (vcl[k + l * 3] - vcl[k + lw * 3]) / dir[k - 1];
		    if (t > ntol) {
			if (t < tmin || imin == 0) {
			    lw1 = l;
			    imin = a;
			    tmin = t;
			}
		    }
		}
	    }
	} else if (cb == 2) {
	    la = faceverticeslist[a * 6 + 1];
	    l = faceverticeslist[faceverticeslist[e * 6 + 3] * 6 + 1];
	    if (l == la && ccwfl < 0 || l != la && ccwfl > 0) {
		t = (vcl[k + l * 3] - vcl[k + lw * 3]) / dir[k - 1];
		if (t > ntol) {
		    if (t < tmin || imin == 0) {
			lw1 = l;
			imin = a;
			tmin = t;
		    }
		}
	    }
	} else {
	    la = faceverticeslist[e * 6 + 1];
	    lb = faceverticeslist[faceverticeslist[e * 6 + 3] * 6 + 1];
	    dir1[0] = vcl[la * 3 + 1] - vcl[lb * 3 + 1];
	    dir1[1] = vcl[la * 3 + 2] - vcl[lb * 3 + 2];
	    dir1[2] = vcl[la * 3 + 3] - vcl[lb * 3 + 3];
	    rhs[0] = vcl[la * 3 + 1] - vcl[lw * 3 + 1];
	    rhs[1] = vcl[la * 3 + 2] - vcl[lw * 3 + 2];
	    rhs[2] = vcl[la * 3 + 3] - vcl[lw * 3 + 3];
	    cp[0] = dir[1] * dir1[2] - dir[2] * dir1[1];
	    cp[1] = dir[2] * dir1[0] - dir[0] * dir1[2];
	    cp[2] = dir[0] * dir1[1] - dir[1] * dir1[0];
	    l = 1;
	    if (abs(cp[1]) > abs(cp[0])) {
		l = 2;
	    }
	    if (abs(cp[2]) > (d__1 = cp[l - 1], abs(d__1))) {
		l = 3;
	    }
	    if (l == 1) {
		t = (rhs[1] * dir1[2] - rhs[2] * dir1[1]) / cp[0];
	    } else if (l == 2) {
		t = (rhs[2] * dir1[0] - rhs[0] * dir1[2]) / cp[1];
	    } else {
		t = (rhs[0] * dir1[1] - rhs[1] * dir1[0]) / cp[2];
	    }
	    if (t > ntol) {
		if (t < tmin || imin == 0) {
		    imin = -a;
		    tmin = t;
		    isave = i__;
		}
	    }
	}
L80:
	;
    }
    if (imin == 0) {
/*  	    IERR = 326 */
	return;
    }
    if (imin < 0) {
	if (lv > this->n_original_vertices ) {
	    if (-imin == cedge[2]) {
		lw1 = lv;
		goto L90;
	    }
	}
	n++;
recheck_size_1:
	if ( (2+n)*1 > this->size_vcl ) {
	    int res = increase_memory((void **)&this->g_vcl, &this->size_vcl, 3*sizeof(double));
	    if ( res == 0 ) {
		this->ierr = 500;
		return;
	    }
//	    *size_vcl *= 2;
	    this->size_vcl += 1024;
	    vcl = this->g_vcl;
	    vcl -= 4;
	    goto recheck_size_1;
	}
	lw1 = n;
	vcl[n * 3 + 1] = vcl[lw * 3 + 1] + tmin * dir[0];
	vcl[n * 3 + 2] = vcl[lw * 3 + 2] + tmin * dir[1];
	vcl[n * 3 + 3] = vcl[lw * 3 + 3] + tmin * dir[2];
L90:
	if ((i__1 = facesdata[fl * 3 + 2], abs(i__1)) != (i__2 = facesdata[fl * 3 + 3]
		, abs(i__2))) {
	    for (i__ = 1; i__ <= 3; ++i__) {
		j = pedge[i__ + isave * 3];
		pedge[i__ + isave * 3] = pedge[i__ + *nedgc * 3];
		pedge[i__ + *nedgc * 3] = j;
/* L100: */
	    }
	    --(*nedgc);
	}
    }
    ++(*nce);
    cedge[(*nce << 1) + 1] = lw1;
    cedge[(*nce << 1) + 2] = -abs(imin);

/*        If vertex of cut polygon has appeared before, then cut polygon */
/*        is simply-connected (non-simple), so reject cut plane. */

L110:
    if (lw1 == lv) {
	goto L150;
    }
    if (lw1 <= this->n_original_vertices ) {
	i__1 = nev;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    if (lw1 == this->g_ev[i__ - 1]) {
		IVP_IF(1) {
		    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
			ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL3, "Rejected due to simply-connected polygon: case 1\n");
		    }
		}
		return;
	    }
/* L120: */
	}

recheck_size_2:
	if (nev >= this->size_ev) { // orig. 20
	    int res = increase_memory((void **)&this->g_ev, &this->size_ev, sizeof(int));
	    if ( res == 0 ) {
		this->ierr = 500;
		return;
	    }
//	    *addr_of_size_ev *= 2;
	    this->size_ev += 1024;
	    goto recheck_size_2;
	}

	++nev;
	this->g_ev[nev - 1] = lw1;
    } else {
	i__1 = n - 1;
	for (i__ = this->n_original_vertices + 1; i__ <= i__1; ++i__) {
	    for (j = 1; j <= 3; ++j) {
/* Computing MAX */
		d__3 = (d__1 = vcl[j + i__ * 3], abs(d__1)), d__4 = (d__2 = 
			vcl[j + n * 3], abs(d__2));
		cmax = max(d__3,d__4);
		if ((d__1 = vcl[j + i__ * 3] - vcl[j + n * 3], abs(d__1)) > 
			this->tolerance * cmax && cmax > this->tolerance) {
		    goto L140;
		}
/* L130: */
	    }
	    IVP_IF(1) {
		IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
		    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL3, "Rejected due to simply-connected polygon: case 2\n");
		}
	    }
	    return;
L140:
	    ;
	}
    }

/*        Compute dihedral angles due to cut plane at edge. If any angle */
/*        is too small, reject cut plane. */

L150:
    if (fr == 0) {
	f = fl;
	dof = (i__1 = facesdata[f * 3 + 2], abs(i__1)) == (i__2 = facesdata[f * 3 + 3]
		, abs(i__2));
	eflag = (i__1 = facesdata[f * 3 + 2], abs(i__1)) != *p || dof && ccwfl < 
		0;
    } else {
	f = fr;
	dof = (i__1 = facesdata[f * 3 + 2], abs(i__1)) == (i__2 = facesdata[f * 3 + 3]
		, abs(i__2));
	sf = ccwfl;
	if (dof) {
	    e = cedge[(*nce << 1) + 2];
	    la = faceverticeslist[e * 6 + 1];
	    lb = faceverticeslist[faceverticeslist[e * 6 + 3] * 6 + 1];
	    if ((lb - la) * ccwfl > 0) {
		e = faceverticeslist[e * 6 + 6];
	    } else {
		e = faceverticeslist[e * 6 + 5];
	    }
	    l = faceverticeslist[e * 6 + 1];
	    if (l == la) {
		sf = -ccwfl;
	    }
	}
	eflag = (i__1 = facesdata[f * 3 + 2], abs(i__1)) != *p || dof && sf < 0;
    }
    dotp = -(nrmlc[1] * normals[f * 3 + 1] + nrmlc[2] * normals[f * 3 + 2] + nrmlc[
	    3] * normals[f * 3 + 3]);
    if (abs(dotp) > 1. - this->tolerance) {
	dotp = d_sign(1.0, dotp);
    }
    if (eflag) {
	dotp = -dotp;
    }
    angr = IVP_PI - acos(dotp);
    dir1[0] = nrmlc[2] * dir[2] - nrmlc[3] * dir[1];
    dir1[1] = nrmlc[3] * dir[0] - nrmlc[1] * dir[2];
    dir1[2] = nrmlc[1] * dir[1] - nrmlc[2] * dir[0];
    dotp = dir1[0] * normals[f * 3 + 1] + dir1[1] * normals[f * 3 + 2] + dir1[2] * 
	    normals[f * 3 + 3];
    if (eflag) {
	dotp = -dotp;
    }
    if (dotp > 0.) {
	angr = pi2 - angr;
    }
    if (fr == 0) {
	ang = IVP_PI;
    } else {
	a = cedge[(*nce << 1) + 2];
	la = faceverticeslist[a * 6 + 1];
	lb = faceverticeslist[faceverticeslist[a * 6 + 3] * 6 + 1];
	if ((lb - la) * ccwfl > 0) {
	    ang = edge_angles[a];
	} else {
	    ang = edge_angles[faceverticeslist[a * 6 + 5]];
	}
    }
    if ( (angr < this->angacc) || ang - angr < this->angacc ) {
	IVP_IF(1) {
	    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL3, "Rejected due to small angle\n");
	    }
	}
	return;
    }
    if (ccwfl > 0) {
	cdang[*nce] = angr;
    } else {
	cdang[*nce] = -angr;
    }
    if (lw1 != lv) {
	goto L10;
    }

/*     Determine if cut polygon is outer or inner boundary by summing */
/*     the exterior angles (which lie in range (-PI,PI)). A sum of 2*PI */
/*     (-2*PI) means that polygon is outer (inner). Cut polygon is */
/*     rejected in the latter case. */

    s = 0.;
    la = cedge[(((*nce)-1) << 1) + 1];
    lb = cedge[1];
    dir1[0] = vcl[lb * 3 + 1] - vcl[la * 3 + 1];
    dir1[1] = vcl[lb * 3 + 2] - vcl[la * 3 + 2];
    dir1[2] = vcl[lb * 3 + 3] - vcl[la * 3 + 3];
/* Computing 2nd power */
    d__1 = dir1[0];
/* Computing 2nd power */
    d__2 = dir1[1];
/* Computing 2nd power */
    d__3 = dir1[2];
    dir1sq = d__1 * d__1 + d__2 * d__2 + d__3 * d__3;
    i__1 = *nce - 1;
    for (i__ = 0; i__ <= i__1; ++i__) {
	dir[0] = dir1[0];
	dir[1] = dir1[1];
	dir[2] = dir1[2];
	dirsq = dir1sq;
	la = lb;
	lb = cedge[((i__+1) << 1) + 1];
	dir1[0] = vcl[lb * 3 + 1] - vcl[la * 3 + 1];
	dir1[1] = vcl[lb * 3 + 2] - vcl[la * 3 + 2];
	dir1[2] = vcl[lb * 3 + 3] - vcl[la * 3 + 3];
/* Computing 2nd power */
	d__1 = dir1[0];
/* Computing 2nd power */
	d__2 = dir1[1];
/* Computing 2nd power */
	d__3 = dir1[2];
	dir1sq = d__1 * d__1 + d__2 * d__2 + d__3 * d__3;
	dotp = (dir[0] * dir1[0] + dir[1] * dir1[1] + dir[2] * dir1[2]) / 
		sqrt(dirsq * dir1sq);
	if (abs(dotp) > 1. - this->tolerance) {
	    dotp = d_sign(1.0, dotp);
	}
	ang = acos(dotp);
	if (kmax == 1) {
	    cp[0] = dir[1] * dir1[2] - dir[2] * dir1[1];
	} else if (kmax == 2) {
	    cp[1] = dir[2] * dir1[0] - dir[0] * dir1[2];
	} else {
	    cp[2] = dir[0] * dir1[1] - dir[1] * dir1[0];
	}
	if (cp[kmax - 1] * nrmlc[kmax] < 0.) {
	    ang = -ang;
	}
	s += ang;
/* L160: */
    }
    if (s < 0.) {
	IVP_IF(1) {
	    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL3, "Rejected due to inner boundary\n");
	    }
	}
	return;
    }

/*     Move edges incident on LV (if <= this->N_ORIGINAL_VERTICES), EV(1:NEV) to end of PEDGE. */

    if (lv <= this->n_original_vertices ) {
	l = lv;
	ee = 0;
    } else {
	ee = 1;
    }
    i__1 = nev;
    for (e = ee; e <= i__1; ++e) {
	if (e > 0) {
	    l = this->g_ev[e - 1];
	}
	k = 1;
L170:
	if (k > *nedgc) {
	    goto L190;
	}
	a = pedge[k * 3 + 1];
	la = faceverticeslist[a * 6 + 1];
	lb = faceverticeslist[faceverticeslist[a * 6 + 3] * 6 + 1];
	if (la == l || lb == l) {
	    for (i__ = 1; i__ <= 3; ++i__) {
		j = pedge[i__ + k * 3];
		pedge[i__ + k * 3] = pedge[i__ + *nedgc * 3];
		pedge[i__ + *nedgc * 3] = j;
/* L180: */
	    }
	    --(*nedgc);
	} else {
	    ++k;
	}
	goto L170;
L190:
	;
    }

/*     Determine if cut face contains any inner polygons by checking */
/*     if the remaining edges of PEDGE intersect interior of cut face. */

    i__1 = *nedgc;
    for (i__ = 1; i__ <= i__1; ++i__) {
	a = pedge[i__ * 3 + 1];
	ca = pedge[i__ * 3 + 3] / 10;
	cb = pedge[i__ * 3 + 3] % 10;
	la = faceverticeslist[a * 6 + 1];
	lb = faceverticeslist[faceverticeslist[a * 6 + 3] * 6 + 1];
	if (ca == 2) {
	    cp[0] = vcl[la * 3 + 1];
	    cp[1] = vcl[la * 3 + 2];
	    cp[2] = vcl[la * 3 + 3];
	} else if (cb == 2) {
	    cp[0] = vcl[lb * 3 + 1];
	    cp[1] = vcl[lb * 3 + 2];
	    cp[2] = vcl[lb * 3 + 3];
	} else {
	    dir[0] = vcl[lb * 3 + 1] - vcl[la * 3 + 1];
	    dir[1] = vcl[lb * 3 + 2] - vcl[la * 3 + 2];
	    dir[2] = vcl[lb * 3 + 3] - vcl[la * 3 + 3];
	    t = (nrmlc[4] - nrmlc[1] * vcl[la * 3 + 1] - nrmlc[2] * vcl[la * 
		    3 + 2] - nrmlc[3] * vcl[la * 3 + 3]) / (nrmlc[1] * dir[0] 
		    + nrmlc[2] * dir[1] + nrmlc[3] * dir[2]);
	    cp[0] = vcl[la * 3 + 1] + t * dir[0];
	    cp[1] = vcl[la * 3 + 2] + t * dir[1];
	    cp[2] = vcl[la * 3 + 3] + t * dir[2];
	}

	// *******************************************************************
	ptpolg_(3,
		3,
		nce,
		2,
		&cedge[1],
		&vcl[4],
		cp,
		&nrmlc[1], 
		dtol,
		&inout);

	if (inout == 1) {
	    IVP_IF(1) {
		IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
		    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL3, "Rejected due to hole polygon\n");
		}
	    }
	    return;
	}
/* L200: */
    }
    *rflag = 1;
    IVP_IF(1) {
	IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL3, "CEDGE(1:2), CDANG\n");
	    i__1 = *nce;
	    for (i__ = 1; i__ <= i__1; ++i__) {
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL3, "%d %d %d %f\n", i__, cedge[(i__ << 1) + 1], cedge[(i__ << 1) + 2], cdang[i__] * 180.0 / IVP_PI);
	    }
  	}
    }

    return;
}

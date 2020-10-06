/* ptpolg.f -- translated by f2c (version 19990311).
*/

#include <ivp_physics.hxx>

#include <geompack.hxx>


void IVP_Geompack::ptpolg_(
			   int		dim,
			   int		ldv,
			   int		*nv,
			   int		inc,
			   int		*pgind,
			   double	*vcl,
			   double	*pt,
			   double	*nrml,
			   double	*dtol,
			   int		*inout
			  ) {

    /* System generated locals */
    int vcl_dim1, vcl_offset, i__1, i__2;
    double d__1, d__2, d__3;

    /* Local variables */
    double area, dist;
    int h__, i__, j, k, l, m, s;
    double t, armax, da[3], db[3];
    int la, lb;
    double cp[3];
    int sa;
    double ta;
    int sb;
    double nr[4], dir[3], rhs[3];


/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Determine whether a point lies inside, outside, or on */
/*        boundary of a planar polygon in 2 or 3 dimensional space. */
/*        It is assumed that point lies in plane of polygon. */

/*     Input parameters: */
/*        DIM - dimension of polygon (2 or 3) */
/*        LDV - leading dimension of VCL array in calling routine */
/*        NV - number of vertices in polygon */
/*        INC - increment for PGIND indicating indices of polygon */
/*        PGIND(0:NV*INC) - indices in VCL of polygon vertices are in */
/*              PGIND(0), PGIND(INC), ..., PGIND(NV*INC) with first and */
/*              last vertices identical */
/*        VCL(1:DIM,1:*) - vertex coordinate list */
/*        PT(1:DIM) - point for which in/out test is applied */
/*        NRML(1:3) - unit normal vector of plane containing polygon, */
/*              with vertices oriented CCW wrt normal (used iff DIM = 3); */
/*              normal is assumed to be (0,0,1) if DIM = 2 */
/*        DTOL - absolute tolerance to determine whether a point is on */
/*              a line or plane */

/*     Output parameters: */
/*        INOUT - +1, 0, or -1 depending on whether point PT is inside */
/*              polygon, on boundary of polygon, or outside polygon; */
/*              or -2 if error in input parameters */



    /* Parameter adjustments */
    --pt;
    vcl_dim1 = ldv;
    vcl_offset = 1 + vcl_dim1 * 1;
    vcl -= vcl_offset;
    --nrml;

    /* Function Body */
    *inout = -2;
    if (dim < 2 || dim > 3) {
	return;
    }

/*     Find edge subtending max area with PT as third triangle vertex. */

    armax = 0.;
    h__ = 0;
    lb = pgind[0];
    i__1 = dim;
    for (j = 1; j <= i__1; ++j) {
	db[j - 1] = vcl[j + lb * vcl_dim1] - pt[j];
/* L10: */
    }
    i__1 = *nv;
    for (i__ = 1; i__ <= i__1; ++i__) {
	la = lb;
	lb = pgind[i__ * inc];
	i__2 = dim;
	for (j = 1; j <= i__2; ++j) {
	    da[j - 1] = db[j - 1];
	    db[j - 1] = vcl[j + lb * vcl_dim1] - pt[j];
	    dir[j - 1] = vcl[j + lb * vcl_dim1] - vcl[j + la * vcl_dim1];
/* L20: */
	}
	if (dim == 2) {
	    area = (d__1 = da[0] * db[1] - db[0] * da[1], abs(d__1));
	} else {
/* Computing 2nd power */
	    d__1 = da[1] * db[2] - db[1] * da[2];
/* Computing 2nd power */
	    d__2 = da[2] * db[0] - db[2] * da[0];
/* Computing 2nd power */
	    d__3 = da[0] * db[1] - db[0] * da[1];
	    area = d__1 * d__1 + d__2 * d__2 + d__3 * d__3;
	}
	if (area > armax) {
	    h__ = i__;
	    armax = area;
	}
/* L30: */
    }
    if (dim == 2) {
/* Computing 2nd power */
	d__1 = armax;
	armax = d__1 * d__1;
    }
/* Computing 2nd power */
    d__1 = *dtol;
    if (armax <= d__1 * d__1) {
	return;
    }

/*     Direction of ray is from PT through midpoint of edge subtending */
/*     max area. NR is unit normal of line or plane containing ray, */
/*     which is also orthogonal to NRML in 3-D case. */

    la = pgind[(h__ - 1) * inc];
    lb = pgind[h__ * inc];
    dir[0] = (vcl[la * vcl_dim1 + 1] + vcl[lb * vcl_dim1 + 1]) * .5 - pt[1];
    dir[1] = (vcl[la * vcl_dim1 + 2] + vcl[lb * vcl_dim1 + 2]) * .5 - pt[2];
    if (dim == 2) {
/* Computing 2nd power */
	d__1 = dir[0];
/* Computing 2nd power */
	d__2 = dir[1];
	dist = sqrt(d__1 * d__1 + d__2 * d__2);
	dir[0] /= dist;
	dir[1] /= dist;
	dir[2] = 0.;
	nr[0] = -dir[1];
	nr[1] = dir[0];
	nr[3] = nr[0] * pt[1] + nr[1] * pt[2];
	dist = nr[0] * vcl[lb * vcl_dim1 + 1] + nr[1] * vcl[lb * vcl_dim1 + 2]
		 - nr[3];
    } else {
	dir[2] = (vcl[la * vcl_dim1 + 3] + vcl[lb * vcl_dim1 + 3]) * .5 - pt[
		3];
/* Computing 2nd power */
	d__1 = dir[0];
/* Computing 2nd power */
	d__2 = dir[1];
/* Computing 2nd power */
	d__3 = dir[2];
	dist = sqrt(d__1 * d__1 + d__2 * d__2 + d__3 * d__3);
	dir[0] /= dist;
	dir[1] /= dist;
	dir[2] /= dist;
	nr[0] = nrml[2] * dir[2] - nrml[3] * dir[1];
	nr[1] = nrml[3] * dir[0] - nrml[1] * dir[2];
	nr[2] = nrml[1] * dir[1] - nrml[2] * dir[0];
	nr[3] = nr[0] * pt[1] + nr[1] * pt[2] + nr[2] * pt[3];
	dist = nr[0] * vcl[lb * vcl_dim1 + 1] + nr[1] * vcl[lb * vcl_dim1 + 2]
		 + nr[2] * vcl[lb * vcl_dim1 + 3] - nr[3];
    }
    if (dist > 0.) {
	sb = 1;
    } else {
	sb = -1;
    }
    m = 1;
    if (abs(dir[1]) > abs(dir[0])) {
	m = 2;
    }
    if (abs(dir[2]) > (d__1 = dir[m - 1], abs(d__1))) {
	m = 3;
    }
    k = 1;

/*     For remaining edges of polygon, check whether ray intersects edge. */
/*     Vertices or edges lying on ray are handled by looking at preceding */
/*     and succeeding edges not lying on ray. */

    k = 1;
    i__ = h__ + 1;
    if (i__ > *nv) {
	i__ = 1;
    }
L40:
    la = lb;
    lb = pgind[i__ * inc];
    sa = sb;
    if (dim == 2) {
	dist = nr[0] * vcl[lb * vcl_dim1 + 1] + nr[1] * vcl[lb * vcl_dim1 + 2]
		 - nr[3];
    } else {
	dist = nr[0] * vcl[lb * vcl_dim1 + 1] + nr[1] * vcl[lb * vcl_dim1 + 2]
		 + nr[2] * vcl[lb * vcl_dim1 + 3] - nr[3];
    }
    if (abs(dist) <= *dtol) {
	sb = 0;
    } else if (dist > 0.) {
	sb = 1;
    } else {
	sb = -1;
    }
    s = sa * sb;
    if (s < 0) {
	if (dim == 2) {
	    da[0] = vcl[la * vcl_dim1 + 1] - vcl[lb * vcl_dim1 + 1];
	    da[1] = vcl[la * vcl_dim1 + 2] - vcl[lb * vcl_dim1 + 2];
	    rhs[0] = vcl[la * vcl_dim1 + 1] - pt[1];
	    rhs[1] = vcl[la * vcl_dim1 + 2] - pt[2];
	    t = (rhs[0] * da[1] - rhs[1] * da[0]) / (dir[0] * da[1] - dir[1] *
		     da[0]);
	} else {
	    da[0] = vcl[la * vcl_dim1 + 1] - vcl[lb * vcl_dim1 + 1];
	    da[1] = vcl[la * vcl_dim1 + 2] - vcl[lb * vcl_dim1 + 2];
	    da[2] = vcl[la * vcl_dim1 + 3] - vcl[lb * vcl_dim1 + 3];
	    rhs[0] = vcl[la * vcl_dim1 + 1] - pt[1];
	    rhs[1] = vcl[la * vcl_dim1 + 2] - pt[2];
	    rhs[2] = vcl[la * vcl_dim1 + 3] - pt[3];
	    cp[0] = dir[1] * da[2] - dir[2] * da[1];
	    cp[1] = dir[2] * da[0] - dir[0] * da[2];
	    cp[2] = dir[0] * da[1] - dir[1] * da[0];
	    l = 1;
	    if (abs(cp[1]) > abs(cp[0])) {
		l = 2;
	    }
	    if (abs(cp[2]) > (d__1 = cp[l - 1], abs(d__1))) {
		l = 3;
	    }
	    if (l == 1) {
		t = (rhs[1] * da[2] - rhs[2] * da[1]) / cp[0];
	    } else if (l == 2) {
		t = (rhs[2] * da[0] - rhs[0] * da[2]) / cp[1];
	    } else {
		t = (rhs[0] * da[1] - rhs[1] * da[0]) / cp[2];
	    }
	}
	if (t > *dtol) {
	    ++k;
	} else if (t >= -(*dtol)) {
	    *inout = 0;
	    return;
	}
    } else if (s == 0) {
	l = lb;
L50:
	++i__;
	if (i__ > *nv) {
	    i__ = 1;
	}
	if (i__ == h__) {
	    return;
	}
	la = lb;
	lb = pgind[i__ * inc];
	if (dim == 2) {
	    dist = nr[0] * vcl[lb * vcl_dim1 + 1] + nr[1] * vcl[lb * vcl_dim1 
		    + 2] - nr[3];
	} else {
	    dist = nr[0] * vcl[lb * vcl_dim1 + 1] + nr[1] * vcl[lb * vcl_dim1 
		    + 2] + nr[2] * vcl[lb * vcl_dim1 + 3] - nr[3];
	}
	if (abs(dist) <= *dtol) {
	    goto L50;
	} else if (dist > 0.) {
	    sb = 1;
	} else {
	    sb = -1;
	}
	t = (vcl[m + l * vcl_dim1] - pt[m]) / dir[m - 1];
	if (abs(t) <= *dtol) {
	    *inout = 0;
	    return;
	}
	if (la != l) {
	    ta = (vcl[m + la * vcl_dim1] - pt[m]) / dir[m - 1];
	    if (abs(ta) <= *dtol || t * ta < 0.) {
		*inout = 0;
		return;
	    }
	}
	if (sa * sb < 0 && t > 0.) {
	    ++k;
	}
    }
    ++i__;
    if (i__ > *nv) {
	i__ = 1;
    }
    if (i__ != h__) {
	goto L40;
    }

/*     Point lies inside polygon if number of intersections K is odd. */

    if (k % 2 == 1) {
	*inout = 1;
    } else {
	*inout = -1;
    }

    return;
}

/* insfac.f -- translated by f2c (version 19990311).
*/


#include <ivp_physics.hxx>
#include <ivp_betterdebugmanager.hxx>
#include <geompack.hxx>


void IVP_Geompack::insfac_(
			   int		*p,
			   double	*nrmlc,
			   int		*nce,
			   int		*cedge,
			   double	*cdang
			  ) {

    double *vcl                       = this->g_vcl;
    int	   *polyhedronfirstfaceoffset = this->g_polyhedronfirstfaceoffset;
    int	   *polyhedronfaceindices     = this->g_polyhedronfaceindices;
    int    *facesdata                 = this->g_facesdata;
    int    *facestype                 = this->g_facestype;
    double *normals                   = this->g_normals;
    int    *faceverticeslist          = this->g_faceverticeslist;
    double *edge_angles               = this->g_edge_angles;

    /* System generated locals */
    int i__1, i__2;
    double d__1;

    /* Local variables */
    int head;
    long int docf;
    int pind, a, b, c__, f, g, i__, j, k, tailn, tailp, tailt;
    int la, lb;
    int fp, sf, nv, sp, np1;
    double ang;
    long int dof;
    int ptr;


/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Insert a new face (cut face) in polyhedral decomposition */
/*        data structure. It is assumed that interior of face does not */
/*        intersect any other faces. */

/*     Input parameters: */
/*        P - polyhedron index */
/*        NRMLC(1:3) - unit normal vector of cut plane */
/*        NCE - number of edges in cut face */
/*        CEDGE(1:2,0:NCE) - CEDGE(1,I) is an index of VCL, indices > ADDR_OF_N_ORIGINAL_VERTICES */
/*              are new points; CEDGE(2,I) = J indicates that edge of cut */
/*              face ending at CEDGE(1,I) is edge from J to FACEVERTICESLIST(SUCC,J) */
/*              if J > 0; else if J < 0 then edge of cut face ending at */
/*              CEDGE(1,I) is a new edge and CEDGE(1,I) lies on edge from */
/*              -J to FACEVERTICESLIST(SUC,-J) and new edge lies in face FACEVERTICESLIST(FACN,-J); */
/*              CEDGE(2,I) always refers to an edge in the subpolyhedron */
/*              in negative half-space; CEDGE(1,NCE) = CEDGE(1,0); */
/*              CEDGE(2,0) is not input but is used temporarily */
/*        CDANG(1:NCE) - dihedral angles created by edges of cut polygon */
/*              in pos. half-space; neg. sign for angle I indicates that */
/*              face containing edge I is oriented CW in polyhedron P */
/*        ADDR_OF_N_ORIGINAL_VERTICES - number of vertex coordinates (excluding new ones)) */
/*        NFACE - number of faces or positions used in FACESDATA array */
/*        ADDR_OF_N_WORK_VERTICES - number of positions used in FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        NPOLH - number of polyhedra or positions used in POLYHEDRONFIRSTFACEOFFSET array */
/*        ADDR_OF_N_POLYHEDRONFACES - number of positions used in POLYHEDRONFACEINDICES array */
/*        ADDR_OF_SIZE_FACEARRAYS - maximum size available for FACESDATA, FACESTYPE, NORMALS arrays */
/*        ADDR_OF_SIZE_FACEVERTEXARRAYS - maximum size available for FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        ADDR_OF_SIZE_POLYHEDRONFIRSTFACEOFFSET - maximum size available for POLYHEDRONFIRSTFACEOFFSET array */
/*        ADDR_OF_SIZE_POLYHEDRONFACEINDICES - maximum size available for POLYHEDRONFACEINDICES array */
/*        VCL(1:3,1:ADDR_OF_N_ORIGINAL_VERTICES+?) - vertex coordinate list; the new vertices to */
/*           be inserted as indicated by CEDGE are after column ADDR_OF_N_ORIGINAL_VERTICES */
/*        FACESDATA(1:3,1:NFACE) - face pointer list */
/*        FACESTYPE(1:NFACE) - face types */
/*        NORMALS(1:3,1:NFACE) - unit normal vectors for faces */
/*        FACEVERTICESLIST(1:6,1:ADDR_OF_N_WORK_VERTICES) - face vertex list */
/*        EDGE_ANGLES(1:ADDR_OF_N_WORK_VERTICES) - edge angles */
/*        POLYHEDRONFIRSTFACEOFFSET(1:NPOLH) - head ptr to face indices in POLYHEDRONFACEINDICES for each polyh */
/*        POLYHEDRONFACEINDICES(1:2,1:ADDR_OF_N_POLYHEDRONFACES) - list of signed face indices for each polyh */

/*     Updated parameters: */
/*        CEDGE(1:1,1:NCE) - updated to edges of cut face wrt positive */
/*              half-space */
/*        CEDGE(2:2,1:NCE) - neg. entries updated to index of new edge */
/*        ADDR_OF_N_ORIGINAL_VERTICES,NFACE,ADDR_OF_N_WORK_VERTICES,NPOLH,ADDR_OF_N_POLYHEDRONFACES - sizes updated due to cut face */
/*        FACESDATA,FACESTYPE,NORMALS,FACEVERTICESLIST,EDGE_ANGLES,POLYHEDRONFIRSTFACEOFFSET,POLYHEDRONFACEINDICES - updated by cut face */

/*     Routines called: */
/*        INSVR3, INSED3 */

/*     Abnormal return: */
/*        IERR is set to 15, 16, 17, or 18 */




/*     Insert new vertices and update CEDGE(2,*). */

    /* Parameter adjustments */
    --nrmlc;
    --cdang;
    normals -= 4;
    --facestype;
    facesdata -= 4;
    --edge_angles;
    faceverticeslist -= 7;
    --polyhedronfirstfaceoffset;
    polyhedronfaceindices -= 3;

    /* Function Body */
    i__1 = *nce - 1;
    for (i__ = 0; i__ <= i__1; ++i__) {
	if (cedge[(i__ << 1) + 1] <= this->n_original_vertices ) {
	    goto L10;
	}
	if (i__ == 0) {
	    j = *nce;
	} else {
	    j = i__;
	}
	a = -cedge[(j << 1) + 2];

	// *******************************************************************
	insvr3_(a);

	faceverticeslist = this->g_faceverticeslist;
	faceverticeslist -= 7;

	edge_angles = this->g_edge_angles;
	--edge_angles;


	if (this->ierr != 0) {
	    return;
	}
	if (cdang[j] < 0.) {
	    cedge[(j << 1) + 2] = -faceverticeslist[a * 6 + 3];
	}
L10:
	;
    }

/*     Insert new edges and update CEDGE(2,*). */

    cedge[2] = cedge[(*nce << 1) + 2];
    i__1 = *nce;
    for (i__ = 1; i__ <= i__1; ++i__) {
	b = -cedge[(i__ << 1) + 2];
	if (b < 0) {
	    goto L30;
	}
	f = faceverticeslist[b * 6 + 2];
	la = cedge[((i__-1) << 1) + 1];
	a = cedge[((i__-1) << 1) + 2];
	if (a < 0) {
/*           This can only occur if I = 1. */
	    a = -a;
	    if (faceverticeslist[a * 6 + 1] == la) {
		a = faceverticeslist[a * 6 + 4];
	    } else {
		a = faceverticeslist[a * 6 + 3];
	    }
	}
L20:
	if (faceverticeslist[a * 6 + 1] == la) {
	    a = faceverticeslist[a * 6 + 4];
	    j = la - faceverticeslist[a * 6 + 1];
	    sf = *p;
	} else {
	    a = faceverticeslist[a * 6 + 3];
	    j = faceverticeslist[faceverticeslist[a * 6 + 3] * 6 + 1] - la;
	    sf = -(*p);
	}
	if (j * sf > 0) {
	    a = faceverticeslist[a * 6 + 6];
	} else {
	    a = faceverticeslist[a * 6 + 5];
	}
	fp = faceverticeslist[a * 6 + 2];
	if (fp != f) {
	    goto L20;
	}
	if (faceverticeslist[a * 6 + 1] == la) {
	    j = a;
	    a = faceverticeslist[b * 6 + 3];
	    b = j;
	} else {
	    a = faceverticeslist[a * 6 + 3];
	}

	// *******************************************************************
	insed3_(&a,
		&b,
		&polyhedronfirstfaceoffset[1]
		);

	polyhedronfaceindices = this->g_polyhedronfaceindices;
	polyhedronfaceindices -= 3;

	facesdata = this->g_facesdata;
	facesdata -= 4;

	facestype = this->g_facestype;
	--facestype;

	normals = this->g_normals;
	normals -= 4;

	faceverticeslist = this->g_faceverticeslist;
	faceverticeslist -= 7;

	edge_angles = this->g_edge_angles;
	--edge_angles;

	if (this->ierr != 0) {
	    return;
	}
	cedge[(i__ << 1) + 2] = a;
L30:
	;
    }

/*     Insert cut face into decomposition data structure. Subpolyhedron */
/*     in negative half space is numbered P, other is numbered NPOLH. */

    this->nface++;
    this->npolh++;
    this->n_polyhedronfaces += 2;
recheck_size:
    if ( (this->n_work_vertices + *nce + 2) > this->size_facevertexarrays) {
	int res = 0;
	res = res | increase_memory((void **)&this->g_faceverticeslist, &this->size_facevertexarrays, 6*sizeof(int));
	res = res | increase_memory((void **)&this->g_edge_angles     , &this->size_facevertexarrays,   sizeof(double));
	if ( res == 0 ) {
	    this->ierr = 500;
	    return;
	}
//	*addr_of_size_facevertexarrays *= 2;
	this->size_facevertexarrays +=1024;
	faceverticeslist = this->g_faceverticeslist;
	faceverticeslist -= 7;
	edge_angles = this->g_edge_angles;
	--edge_angles;
	goto recheck_size;
    }

    else if ( (this->nface+2) > this->size_facearrays ) {
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
	goto recheck_size;
    }

    else if ( (2+this->n_polyhedronfaces) > this->size_polyhedronfaceindices ) {
	int res = increase_memory((void **)&this->g_polyhedronfaceindices, &this->size_polyhedronfaceindices, 2*sizeof(int));
	if ( res == 0 ) {
	    this->ierr = 500;
	    return;
	}
//	*addr_of_size_polyhedronfaceindices *= 2;
	this->size_polyhedronfaceindices += 1024;
	polyhedronfaceindices = this->g_polyhedronfaceindices;
	polyhedronfaceindices -= 3;
	goto recheck_size;
    }

    else if ( (1+this->npolh) > this->size_polyhedronfirstfaceoffset ) {
	int res = increase_memory((void **)&this->g_polyhedronfirstfaceoffset, &this->size_polyhedronfirstfaceoffset, sizeof(int));
	if ( res == 0 ) {
	    this->ierr = 500;
	    return;
	}
//	*addr_of_size_polyhedronfirstfaceoffset *= 2;
	this->size_polyhedronfirstfaceoffset += 1024;
	polyhedronfirstfaceoffset = this->g_polyhedronfirstfaceoffset;
	--polyhedronfirstfaceoffset;
	goto recheck_size;
    }

    nv = this->n_work_vertices;
    facesdata[this->nface * 3 + 1] = this->n_work_vertices + 1;
    facesdata[this->nface * 3 + 2] = *p;
    facesdata[this->nface * 3 + 3] = -this->npolh;
    facestype[this->nface] = 0;
    normals[this->nface * 3 + 1] = nrmlc[1];
    normals[this->nface * 3 + 2] = nrmlc[2];
    normals[this->nface * 3 + 3] = nrmlc[3];
    i__1 = *nce - 1;
    for (i__ = 0; i__ <= i__1; ++i__) {
	this->n_work_vertices++;
	faceverticeslist[this->n_work_vertices * 6 + 1] = cedge[(i__ << 1) + 1];
	faceverticeslist[this->n_work_vertices * 6 + 2] = this->nface;
	faceverticeslist[this->n_work_vertices * 6 + 3] = this->n_work_vertices + 1;
	faceverticeslist[this->n_work_vertices * 6 + 4] = this->n_work_vertices - 1;
/* L40: */
    }
    faceverticeslist[this->n_work_vertices * 6 + 3] = facesdata[this->nface * 3 + 1];
    faceverticeslist[facesdata[this->nface * 3 + 1] * 6 + 4] = this->n_work_vertices;

/*     Set CEDGE(1,*) to edges of face in polyh NPOLH (after split). New */
/*     face is CCW from outside new P which contains edges of CEDGE(2,*). */

    i__1 = *nce;
    for (i__ = 1; i__ <= i__1; ++i__) {
	a = cedge[(i__ << 1) + 2];
	la = faceverticeslist[a * 6 + 1];
	lb = faceverticeslist[faceverticeslist[a * 6 + 3] * 6 + 1];
	if ((lb - la) * cdang[i__] > 0.) {
	    cedge[(i__ << 1) + 1] = faceverticeslist[a * 6 + 6];
	} else {
	    cedge[(i__ << 1) + 1] = faceverticeslist[a * 6 + 5];
	}
/* L50: */
    }

/*     Determine which faces of old P belong to new P or other new polyh, */
/*     and update POLYHEDRONFIRSTFACEOFFSET, POLYHEDRONFACEINDICES. NP1 is used as a temporary polyhedron index. */
/*     Faces of old P are put into FACESDATA(2,*) field. */
/*     FACESTYPE(F) is set to -1 for double occurring faces. */

    dof = 0;
    np1 = this->npolh + 1;
    j = polyhedronfirstfaceoffset[*p];
L60:
    f = (i__1 = polyhedronfaceindices[(j << 1) + 1], abs(i__1));
    if ((i__1 = facesdata[f * 3 + 2], abs(i__1)) == (i__2 = facesdata[f * 3 + 3], abs(
	    i__2))) {
	facesdata[f * 3 + 2] = np1;
	facesdata[f * 3 + 3] = -np1;
	facestype[f] = -1;
	dof = 1;
    } else if ((i__1 = facesdata[f * 3 + 2], abs(i__1)) == *p) {
	facesdata[f * 3 + 2] = i_sign(np1, facesdata[f * 3 + 2]);
    } else {
	i__ = facesdata[f * 3 + 2];
	facesdata[f * 3 + 2] = i_sign(np1, facesdata[f * 3 + 3]);
	facesdata[f * 3 + 3] = i__;
	normals[f * 3 + 1] = -normals[f * 3 + 1];
	normals[f * 3 + 2] = -normals[f * 3 + 2];
	normals[f * 3 + 3] = -normals[f * 3 + 3];
    }
    j = polyhedronfaceindices[(j << 1) + 2];
    if (j != polyhedronfirstfaceoffset[*p]) {
	goto L60;
    }
    i__1 = *nce;
    for (i__ = 1; i__ <= i__1; ++i__) {
	j = 2;
	f = faceverticeslist[cedge[(i__ << 1) + 1] * 6 + 2];
	if (facestype[f] == -1) {
	    if (faceverticeslist[cedge[(i__ << 1) + 1] * 6 + 1] != faceverticeslist[(nv + i__) * 6 + 1])
		     {
		j = 3;
	    }
	}
	facesdata[j + f * 3] = i_sign(this->npolh, facesdata[j + f * 3]);
	j = 2;
	f = faceverticeslist[cedge[(i__ << 1) + 2] * 6 + 2];
	if (facestype[f] == -1) {
	    if (cdang[i__] < 0.) {
		j = 3;
	    }
	}
	facesdata[j + f * 3] = i_sign(*p, facesdata[j + f * 3]);
/* L70: */
    }
    polyhedronfaceindices[((this->n_polyhedronfaces-1) << 1) + 1] = this->nface;
    polyhedronfaceindices[(this->n_polyhedronfaces << 1) + 1] = -(this->nface);
    tailp = this->n_polyhedronfaces - 1;
    tailn = this->n_polyhedronfaces;
    tailt = polyhedronfirstfaceoffset[*p];
    ptr = polyhedronfaceindices[(tailt << 1) + 2];
    polyhedronfaceindices[(tailt << 1) + 2] = 0;
    polyhedronfirstfaceoffset[*p] = this->n_polyhedronfaces - 1;
    polyhedronfirstfaceoffset[this->npolh] = this->n_polyhedronfaces;

L80:
    if (ptr == 0) {
	goto L110;
    }
    j = ptr;
    sp = polyhedronfaceindices[(ptr << 1) + 1];
    f = abs(sp);
    ptr = polyhedronfaceindices[(ptr << 1) + 2];
    if (facestype[f] != -1 || sp > 0) {
	k = 2;
    } else {
	k = 3;
    }
    sf = facesdata[k + f * 3];
    if (abs(sf) == *p) {
	polyhedronfaceindices[(tailp << 1) + 2] = j;
	tailp = j;
    } else if (abs(sf) == this->npolh) {
	polyhedronfaceindices[(tailn << 1) + 2] = j;
	tailn = j;
    } else {
	a = facesdata[f * 3 + 1];
	la = faceverticeslist[a * 6 + 1];
L90:
	b = faceverticeslist[a * 6 + 3];
	lb = faceverticeslist[b * 6 + 1];
	if ((lb - la) * sp > 0) {
	    c__ = faceverticeslist[a * 6 + 6];
	} else {
	    c__ = faceverticeslist[a * 6 + 5];
	}
	g = faceverticeslist[c__ * 6 + 2];
	i__ = 2;
	if (facestype[g] == -1) {
	    if (faceverticeslist[c__ * 6 + 1] == la) {
		if (sp > 0) {
		    i__ = 3;
		}
	    } else {
		if (sp < 0) {
		    i__ = 3;
		}
	    }
	}
	if ((i__1 = facesdata[i__ + g * 3], abs(i__1)) == *p) {
	    polyhedronfaceindices[(tailp << 1) + 2] = j;
	    tailp = j;
	    facesdata[k + f * 3] = i_sign(*p, facesdata[k + f * 3]);
	    goto L100;
	} else if ((i__1 = facesdata[i__ + g * 3], abs(i__1)) == this->npolh) {
	    polyhedronfaceindices[(tailn << 1) + 2] = j;
	    tailn = j;
	    facesdata[k + f * 3] = i_sign(this->npolh, facesdata[k + f * 3]);
	    goto L100;
	}
	a = b;
	la = lb;
	if (a != facesdata[f * 3 + 1]) {
	    goto L90;
	}
	polyhedronfaceindices[(tailt << 1) + 2] = j;
	polyhedronfaceindices[(j << 1) + 2] = 0;
	tailt = j;
L100:
	;
    }
    goto L80;
L110:
    polyhedronfaceindices[(tailp << 1) + 2] = this->n_polyhedronfaces - 1;
    polyhedronfaceindices[(tailn << 1) + 2] = this->n_polyhedronfaces;

/*     Check whether cut face occurs twice in same polyhedron. */
/*     Temporarily modify PRED field of edges of cut face. */

    docf = 0;
    i__1 = *nce;
    for (i__ = 1; i__ <= i__1; ++i__) {
	for (j = 1; j <= 2; ++j) {
	    a = cedge[j + (i__ << 1)];
	    faceverticeslist[a * 6 + 4] = -faceverticeslist[a * 6 + 4];
/* L120: */
	}
/* L130: */
    }
    for (i__ = 1; i__ <= 2; ++i__) {
	if (i__ == 1) {
	    head = this->n_polyhedronfaces - 1;
	    pind = *p;
	} else {
	    head = this->n_polyhedronfaces;
	    pind = this->npolh;
	}
	ptr = polyhedronfaceindices[(head << 1) + 2];
L140:
	sf = polyhedronfaceindices[(ptr << 1) + 1];
	f = abs(sf);
	a = facesdata[f * 3 + 1];
	la = faceverticeslist[a * 6 + 1];
L150:
	b = faceverticeslist[a * 6 + 3];
	lb = faceverticeslist[b * 6 + 1];
	if (faceverticeslist[a * 6 + 4] > 0) {
	    if ((lb - la) * sf > 0) {
		c__ = faceverticeslist[a * 6 + 6];
	    } else {
		c__ = faceverticeslist[a * 6 + 5];
	    }
	    g = faceverticeslist[c__ * 6 + 2];
	    k = 2;
	    if (facestype[g] == -1) {
		if (faceverticeslist[c__ * 6 + 1] == la) {
		    if (sf > 0) {
			k = 3;
		    }
		} else {
		    if (sf < 0) {
			k = 3;
		    }
		}
	    }
	    if ((i__1 = facesdata[k + g * 3], abs(i__1)) != pind) {
		docf = 1;
		goto L170;
	    }
	}
	a = b;
	la = lb;
	if (a != facesdata[f * 3 + 1]) {
	    goto L150;
	}
	ptr = polyhedronfaceindices[(ptr << 1) + 2];
	if (ptr != head) {
	    goto L140;
	}
/* L160: */
    }

/*     Reset PRED field of edges of cut face. */

L170:
    i__1 = *nce;
    for (i__ = 1; i__ <= i__1; ++i__) {
	for (j = 1; j <= 2; ++j) {
	    a = cedge[j + (i__ << 1)];
	    faceverticeslist[a * 6 + 4] = -faceverticeslist[a * 6 + 4];
/* L180: */
	}
/* L190: */
    }

/*     Update EDGA, EDGC, and EDGE_ANGLES fields. */

    i__1 = *nce;
    for (i__ = 1; i__ <= i__1; ++i__) {
	a = cedge[(i__ << 1) + 2];
	c__ = cedge[(i__ << 1) + 1];
	la = faceverticeslist[a * 6 + 1];
	lb = faceverticeslist[faceverticeslist[a * 6 + 3] * 6 + 1];
	ang = (d__1 = cdang[i__], abs(d__1));
	if ((lb - la) * cdang[i__] > 0.) {
	    faceverticeslist[a * 6 + 6] = nv + i__;
	    faceverticeslist[c__ * 6 + 5] = nv + i__;
	    faceverticeslist[(nv + i__) * 6 + 6] = c__;
	    faceverticeslist[(nv + i__) * 6 + 5] = a;
	    edge_angles[a] -= ang;
	    edge_angles[nv + i__] = ang;
	} else {
	    faceverticeslist[c__ * 6 + 6] = nv + i__;
	    faceverticeslist[a * 6 + 5] = nv + i__;
	    faceverticeslist[(nv + i__) * 6 + 6] = a;
	    faceverticeslist[(nv + i__) * 6 + 5] = c__;
	    edge_angles[nv + i__] = edge_angles[c__] - ang;
	    edge_angles[c__] = ang;
	}
/* L200: */
    }

/*     If DOF, reset FACESTYPE values of -1 to 0. */

    if (dof) {
	for (i__ = 1; i__ <= 2; ++i__) {
	    if (i__ == 1) {
		head = this->n_polyhedronfaces - 1;
	    } else {
		head = this->n_polyhedronfaces;
	    }
	    ptr = polyhedronfaceindices[(head << 1) + 2];
L210:
	    f = (i__1 = polyhedronfaceindices[(ptr << 1) + 1], abs(i__1));
	    if (facestype[f] == -1) {
		facestype[f] = 0;
	    }
	    ptr = polyhedronfaceindices[(ptr << 1) + 2];
	    if (ptr != head) {
		goto L210;
	    }
/* L220: */
	}
    }

/*     If cut face is double occurring, set all faces to belong to */
/*     polyhedron P. */

    if (! docf) {
	goto L240;
    }
    this->npolh--;
    facesdata[this->nface * 3 + 3] = -(*p);
    tailp = polyhedronfaceindices[((this->n_polyhedronfaces-1) << 1) + 2];
    polyhedronfaceindices[((this->n_polyhedronfaces-1) << 1) + 2] = this->n_polyhedronfaces;
    ptr = this->n_polyhedronfaces;
L230:
    sf = polyhedronfaceindices[(ptr << 1) + 1];
    f = abs(sf);
    if (sf * facesdata[f * 3 + 2] > 0) {
	facesdata[f * 3 + 2] = i_sign(*p, sf);
    } else {
	facesdata[f * 3 + 3] = -(*p);
    }
    if (polyhedronfaceindices[(ptr << 1) + 2] != this->n_polyhedronfaces) {
	ptr = polyhedronfaceindices[(ptr << 1) + 2];
	goto L230;
    } else {
	polyhedronfaceindices[(ptr << 1) + 2] = tailp;
    }

L240:
    IVP_IF(1) {
	IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL2) {
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "Cut face: #edges, polyh(1:2) = %d %d %d\n", (*nce), facesdata[this->nface * 3 + 2], facesdata[this->nface * 3 + 3]);
	    i__1 = *nce;
	    for (i__ = 1; i__ <= i__1; ++i__) {
		la = faceverticeslist[(nv + i__) * 6 + 1];
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, " %d %d ", i__, la);
		for (j = 1; j <= 3; ++j) {
		    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "%f ", vcl[j + la * 3]);
		}
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "\n");

	    }
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "\n");
	}
    }

    return;
}

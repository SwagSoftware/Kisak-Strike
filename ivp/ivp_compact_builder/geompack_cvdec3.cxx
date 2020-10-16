/* cvdec3.f -- translated by f2c (version 19990311).
*/

#include <ivp_physics.hxx>
#include <ivp_betterdebugmanager.hxx>
#include <geompack.hxx>


void IVP_Geompack::cvdec3_() {


    double *edge_angles = this->g_edge_angles;

    /* Local variables */
    int l, n, u;
    long int rflag;
    double piptol;


/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Given one or more polyhedra in polyhedral decomposition */
/*        data structure, decompose the polyhedra into convex parts. It */
/*        is assumed all faces are simple (any faces with holes should be */
/*        decomposed into simple polygons), each face appears at most */
/*        once in a polyhedron (double-occurring faces are not allowed), */
/*        and no interior holes occur in any polyhedra. */

/*     Input parameters: */
/*        ANGACC - min acceptable dihedral angle in radians produced by */
/*              cut faces */
/*        RDACC - minimum acceptable relative distance between cut planes */
/*              and vertices not on plane */
/*        ADDR_OF_N_ORIGINAL_VERTICES - number of vertex coordinates or positions used in VCL */
/*        NFACE - number of faces or positions used in FACESDATA array */
/*        ADDR_OF_N_WORK_VERTICES - number of positions used in FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        NPOLH - number of polyhedra or positions used in POLYHEDRONFIRSTFACEOFFSET array */
/*        ADDR_OF_N_POLYHEDRONFACES - number of positions used in POLYHEDRONFACEINDICES array */
/*        SIZE_VCL - maximum size available for VCL array */
/*        ADDR_OF_SIZE_FACEARRAYS - maximum size available for FACESDATA, FACESTYPE, NORMALS arrays */
/*        ADDR_OF_SIZE_FACEVERTEXARRAYS - maximum size available for FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        ADDR_OF_SIZE_POLYHEDRONFIRSTFACEOFFSET - maximum size available for POLYHEDRONFIRSTFACEOFFSET array */
/*        ADDR_OF_SIZE_POLYHEDRONFACEINDICES - maximum size available for POLYHEDRONFACEINDICES array */
/*        ADDR_OF_intWORKARRAY - maximum size available for intWORKARRAY array; should be about */
/*              5 times number of edges in any polyhedron */
/*        ADDR_OF_SIZE_DOUBLEWORKARRAY - maximum size available for DOUBLEWORKARRAY array; should be about */
/*              number of edges in any polyhedron */
/*        VCL(1:3,1:ADDR_OF_N_ORIGINAL_VERTICES) - vertex coordinate list */
/*        FACESDATA(1:3,1:NFACE) - face pointer list: row 1 is head pointer, */
/*              rows 2 and 3 are signed polyhedron indices */
/*        FACESTYPE(1:NFACE) - face types: useful for specifying types of */
/*              boundary faces; entries must be >= 0; any new interior */
/*              faces (not part of previous face) has face type set to 0 */
/*        NORMALS(1:3,1:NFACE) - unit normal vectors for faces; outward */
/*              normal corresponds to CCW traversal of face from polyh */
/*              with index |FACESDATA(2,F)| */
/*        FACEVERTICESLIST(1:6,1:ADDR_OF_N_WORK_VERTICES) - face vertex list; see routine DSPHDC */
/*        EDGE_ANGLES(1:ADDR_OF_N_WORK_VERTICES) - angles at edges common to 2 faces in a polyh; */
/*              EDGE_ANGLES(J) corresponds to FACEVERTICESLIST(*,J), determined by EDGC field */
/*        POLYHEDRONFIRSTFACEOFFSET(1:NPOLH) - head ptr to face indices in POLYHEDRONFACEINDICES for each polyh */
/*        POLYHEDRONFACEINDICES(1:2,1:ADDR_OF_N_POLYHEDRONFACES) - list of signed face indices for each polyh; */
/*              row 2 used for link */

/*     Updated parameters: */
/*        ADDR_OF_N_ORIGINAL_VERTICES,NFACE,ADDR_OF_N_WORK_VERTICES,NPOLH,ADDR_OF_N_POLYHEDRONFACES - sizes updated due to cut faces */
/*        VCL,FACESDATA,FACESTYPE,NORMALS,FACEVERTICESLIST,EDGE_ANGLES,POLYHEDRONFIRSTFACEOFFSET,POLYHEDRONFACEINDICES - updated by cut faces */
/*              to contain convex decomposition */

/*     Working parameters: */
/*        intWORKARRAY(1:*ADDR_OF_intWORKARRAY) - int work array */
/*        DOUBLEWORKARRAY(1:ADDR_OF_SIZE_DOUBLEWORKARRAY) - double precision work array */

/*     Routines called: */
/*        RESEDG */

/*     Abnormal return: */
/*        IERR is set to 6, 7, 14, 15, 16, 17, 18, 325, 326, 327, or 328 */




    /* Function Body */
    piptol = IVP_PI + this->tolerance;
L10:
    l = 0;
    n = 0;
    u = 1;
L20:
    if (edge_angles[u-1] > piptol) {

	// *******************************************************************
	resedg_(u,
		&rflag
		);

	edge_angles = this->g_edge_angles;

	if (this->ierr != 0) {
	    return;
	}
	if (rflag) {
	    ++n;
	} else {
	    if (l == 0) {
		l = u;
	    }
	}
    }
    ++u;
    if (u <= this->n_work_vertices) {
	goto L20;
    }
    if (l > 0) {
	if (n == 0) {
	    this->ierr = 327;
	    IVP_IF(1) {
		IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
		    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "*** GEOMPACK: at least one reflex edge has not been resolved by routine RESEDG\n");
		}
	    }
	    return;
	} else {
	    goto L10;
	}
    } else {
	if (n > 0) {
	    goto L10;
	}
    }

    return;
}

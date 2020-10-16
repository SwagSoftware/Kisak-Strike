/* drdec3.f -- translated by f2c (version 19990311).
*/

#include <ivp_physics.hxx>

#if !( (defined(__MWERKS__) && defined(__POWERPC__)) || defined(GEKKO) )
#include <malloc.h>
#endif

#include <ivp_betterdebugmanager.hxx>
#include <geompack.hxx>


int IVP_Geompack::i_sign(int a, int b) {
    int x;
    x = (a >= 0 ? a : - a);
    return( b >= 0 ? x : -x);
}

double IVP_Geompack::d_sign(double a, double b) {
    double x;
    x = (a >= 0 ? a : - a);
    return( b >= 0 ? x : -x);
}


int IVP_Geompack::increase_memory(void **mem_block, int *mem_size, int size_of_element) {

////    void *new_mem = p_realloc(*mem_block, (*mem_size) * 2 * size_of_element);
#ifndef GEKKO
    void *new_mem = p_realloc(*mem_block, (*mem_size + 1024) * size_of_element);
#else
	void *new_mem = p_malloc( (*mem_size + 1024) * size_of_element );
#endif
    if ( !new_mem ) {
	return(0);
    }
    
#ifdef GEKKO
    memcpy(new_mem, mem_block, *mem_size);
#endif

    *mem_block = new_mem;
    return(1);
}


void IVP_Geompack::decompose(struct geompack_parameters *params) {

    // Local variables
    int		i;
    int		retry_counter;


    // Initializing variables
    this->size_intworkarray              = 12; // orig: maxiw [5000]. Should be divisible by 3 & 4!;
    this->size_doubleworkarray           = 12; // orig: maxwk [5000]. Should be divisible by 3 & 4!;
    this->size_vcl                       = params->nvc + 2; // number of vertices (NOT bytesize of array!)
    this->size_polyhedronfirstfaceoffset = 2; // Leave this to "2" if there will never be more than one polyhedron to decompose. <orig. maxhf [200]>
    this->size_polyhedronfaceindices     = 1; // number of entries (NOT bytesize of array!) <orig: maxpf [2000]>
    this->size_facearrays                = params->nface + 2; // <orig. maxfp [800]>
    this->size_facevertexarrays          = params->facep[params->nface*3]+2; // <orig. maxfv>
    this->size_ev                        = 200;
    this->hashtable_maxsize              = 307;

    int *	n_original_vertices_out	= params->nvc_out;
    int *	nface_out		= params->nface_out;
    int *	n_work_vertices_out	= params->nvert_out;
    int *	npolh_out		= params->npolh_out;
    double **	vcl_out			= params->vcl_out;
    int **	facep_out		= params->facep_out;
    int **	fvl_out			= params->fvl_out;

    *n_original_vertices_out	= 0;
    *nface_out			= 0;
    *n_work_vertices_out	= 0;
    *npolh_out			= 0;



    // TOLIN  - relative tolerance used to determine TOL
    // TOL    - relative tolerance MAX(TOLIN,100.0D0*EPS) where
    //          EPS is approximation to machine epsilon
    // ANGACC - min acceptable dihedral angle in degrees produced by
    //          a cut face
    // RDACC  - minimum acceptable relative distance between a cut
    //          plane and vertices not on plane
    this->angacc = params->angacc * IVP_PI / 180.0;
    this->rdacc  = params->rdacc;


    // Initialize some basic values
    this->initcb_(params->tolin);


    // allocate necessary dynamic memory
    this->g_hashtable                 = (int *)   p_calloc(this->hashtable_maxsize             ,   sizeof(int));
    this->g_intworkarray              = (int *)   p_calloc(this->size_intworkarray             ,   sizeof(int));    // [5000]
    this->g_doubleworkarray           = (double *)p_calloc(this->size_doubleworkarray          ,   sizeof(double)); // [5000]
    this->g_polyhedronfirstfaceoffset = (int *)   p_calloc(this->size_polyhedronfirstfaceoffset,   sizeof(int));    // [200]
    this->g_polyhedronfaceindices     = (int *)   p_calloc(this->size_polyhedronfaceindices    , 2*sizeof(int));    // [4000]
    this->g_normals                   = (double *)p_calloc(this->size_facearrays               , 3*sizeof(double)); // [2400]
    this->g_facesdata                 = (int *)   p_calloc(this->size_facearrays               , 3*sizeof(int));    // [2400]
    this->g_facestype                 = (int *)   p_calloc(this->size_facearrays               ,   sizeof(int));
    this->g_faceverticeslist          = (int *)   p_calloc(this->size_facevertexarrays         , 6*sizeof(int));    // [21000]
    this->g_edge_angles               = (double *)p_calloc(this->size_facevertexarrays         ,   sizeof(double)); // [3500]
    this->g_ev                        = (int *)   p_calloc(this->size_ev                       ,   sizeof(int));



    if ( !this->g_facesdata || !this->g_facestype || !this->g_hashtable || !this->g_polyhedronfirstfaceoffset || !this->g_polyhedronfaceindices || !this->g_faceverticeslist || !this->g_intworkarray || !this->g_doubleworkarray || !this->g_edge_angles || !this->g_normals || !this->g_ev ) {
	IVP_IF(1) {
	    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "*** GEOMPACK: Out of memory!\n\n");
	    }
	}
	goto out_of_memory;
    }


    // N_ORIGINAL_VERTICES - number of original vertex coordinates (i.e. total number of polyhedron points without any duplicated points for different polygons!)
    // NFACE               - number of faces in decomposition
    // NPOLH               - number of polyhedra in decomposition
    this->n_original_vertices	= params->nvc;
    this->nface		= params->nface;
    this->npolh		= 1; // if changed to anything higher than 1 you will have to adjust the variable 'this->size_polyhedronfirstfaceoffset' accordingly!


    // -----------------------------------------------------------------------
    // Initializing polyhedral decomposition data structure.
    // -----------------------------------------------------------------------

    // init 'vertex coordinate list'
    this->g_vcl = params->vcl;

    // init 'face pointer' list (offsets for each face into faceverticeslist)
    this->g_facesdata = params->facep;

    // set face type for each face to ZERO
    for (i=0; i<this->nface; i++) {
	this->g_facestype[i] = 0;
    }

    this->n_work_vertices = this->g_facesdata[this->nface*3] - 1;

    // face list: offsets of (face defining) points in VCL
    for (i=0; i<this->n_work_vertices; i++) {
	this->g_faceverticeslist[i*6] = params->fvl[i];
    }

    // offsets into face list for each polyhedron (we will only process ONE polyhedron!)
    this->g_polyhedronfirstfaceoffset[0] = 1;
    this->g_polyhedronfirstfaceoffset[1] = this->nface+1;

    this->n_polyhedronfaces = this->g_polyhedronfirstfaceoffset[this->npolh] - 1;

    while ( ((2+this->n_polyhedronfaces)*1) > this->size_polyhedronfaceindices ) { // 2000
	int res = increase_memory((void **)&this->g_polyhedronfaceindices, &this->size_polyhedronfaceindices, 2*sizeof(int));
	if ( res == 0 ) {
	    this->ierr = 500;
	    goto GEOMPACK_abort;
	}
//	size_polyhedronfaceindices *= 2;
	this->size_polyhedronfaceindices += 1024;
    }

    // init 'this->g_polyhedronfaceindices' (?)... we will simply set these values to the number of the
    // corresponding face.
    for (i=1; i<=this->n_polyhedronfaces; i++) {
	this->g_polyhedronfaceindices[(i<<1)-2] = i;
    }


    this->hashtable_size = min(prime_(this->n_original_vertices + 2), this->hashtable_maxsize);

    // ***********************************************************************
    // -----------------------------------------------------------------------
    // Init data structure
    // -----------------------------------------------------------------------
    dsphdc_();

    if (this->ierr != 0) {
	IVP_IF(1) {
	    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
		if ( this->ierr == 500 ) {
		    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "*** GEOMPACK: Out of memory!\n\n");
		}
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "Premature abort due to above error.\n\n");
	    }
	}
	goto GEOMPACK_abort;
    }



    IVP_IF(1) {
	IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
	    int	n_reflex_edges;
	    double	minimum_angle;
	    int mem_iwa;
	    int mem_dwa;
	    int mem_vcl;
	    int mem_pffl;
	    int mem_pfil;
	    int mem_fl;
	    int mem_fvl;
	    int mem_total;

	    // calculate some statistical values
	    n_reflex_edges = 0;
	    minimum_angle = IVP_PI * 2.0f; // set minimum angle to 2*pi
	    for (i=0; i<this->n_work_vertices; i++) {
		if (this->g_edge_angles[i] > IVP_PI + this->tolerance) {
		    n_reflex_edges++;
		}
		if (this->g_edge_angles[i] > -1.0) {
		    // Computing minimum angle in convex decomposition
		    minimum_angle = min(minimum_angle, this->g_edge_angles[i]);
		}
	    }
	    minimum_angle = minimum_angle * 180.0f / IVP_PI;

	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "Statistics after initializing polyhedral decomposition data structure:\n");
	    mem_iwa   = this->size_intworkarray*sizeof(int);
	    mem_dwa   = this->size_doubleworkarray*sizeof(double);
	    mem_vcl   = this->size_vcl*3*sizeof(double);
	    mem_pffl  = this->size_polyhedronfirstfaceoffset*sizeof(int);
	    mem_pfil  = this->size_polyhedronfaceindices*2*sizeof(int);
	    mem_fl    = this->size_facearrays*sizeof(int)+this->size_facearrays*3*sizeof(int)+this->size_facearrays*3*sizeof(double);
	    mem_fvl   = this->size_facevertexarrays*6*sizeof(int)+this->size_facevertexarrays*sizeof(double);
	    mem_total = mem_iwa+ mem_dwa + mem_vcl + mem_pffl + mem_pfil + mem_fl + mem_fvl;
	    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL2) {
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for int WORK ARRAY: %d bytes\n"        , mem_iwa);
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for DOUBLE WORK ARRAY: %d bytes\n"         , mem_dwa);
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for VERTEX COORDINATE LIST: %d bytes\n"    , mem_vcl);
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for POLYHEDRON FIRST FACE LIST: %d bytes\n", mem_pffl);
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for POLYHEDRON FACE INDEX LIST: %d bytes\n", mem_pfil);
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for FACE LISTS: %d bytes\n"                , mem_fl);
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for FACE VERTICES LISTS: %d bytes\n"       , mem_fvl);
	    }
	    else {
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "Total memory allocated: %d bytes\n", mem_total);
	    }
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "N_ORIGINAL_VERTICES = %d\n", this->n_original_vertices);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "NFACE               = %d\n", this->nface);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "N_WORK_VERTICES     = %d\n", this->n_work_vertices);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "NPOLH               = %d\n", this->npolh);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "N_POLYHEDRONFACES   = %d\n", this->n_polyhedronfaces);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "N_REFLEX_EDGES      = %d\n", n_reflex_edges);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "MINIMUM_ANGLE       = %f\n", minimum_angle);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "\n");
	}
    }



    // -----------------------------------------------------------------------
    // Decompose polyhedral region into convex parts.
    // -----------------------------------------------------------------------
    retry_counter = 1;

Retry_convex_decomposition:

    // ***********************************************************************
    this->cvdec3_();

    if ( (this->ierr != 0) && (this->ierr != 327) ) { // abort on error but skip "reflex edge" resolving problems
	IVP_IF(1) {
	    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
		if ( this->ierr == 500 ) {
		    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "*** GEOMPACK: Out of memory!\n\n");
		}
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "Premature abort due to above error.\n\n");
	    }
	}
	goto GEOMPACK_abort;
    }



    IVP_IF(1) {
	IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
	    int	n_reflex_edges;
	    double	minimum_angle;
	    int mem_iwa;
	    int mem_dwa;
	    int mem_vcl;
	    int mem_pffl;
	    int mem_pfil;
	    int mem_fl;
	    int mem_fvl;
	    int mem_total;

	    // calculate some statistical values
	    n_reflex_edges = 0;
	    minimum_angle = IVP_PI * 2.0f; // set minimum angle to 2*pi
	    for (i=0; i<this->n_work_vertices; i++) {
		if (this->g_edge_angles[i] > IVP_PI + this->tolerance) {
		    n_reflex_edges++;
		}
		if (this->g_edge_angles[i] > -1.0) {
		    // Computing minimum angle in convex decomposition
		    minimum_angle = min(minimum_angle, this->g_edge_angles[i]);
		}
	    }
	    minimum_angle = minimum_angle * 180.0f / IVP_PI;

	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "Intermediate decomposition statistics:\n");
	    mem_iwa   = this->size_intworkarray*sizeof(int);
	    mem_dwa   = this->size_doubleworkarray*sizeof(double);
	    mem_vcl   = this->size_vcl*3*sizeof(double);
	    mem_pffl  = this->size_polyhedronfirstfaceoffset*sizeof(int);
	    mem_pfil  = this->size_polyhedronfaceindices*2*sizeof(int);
	    mem_fl    = this->size_facearrays*sizeof(int)+this->size_facearrays*3*sizeof(int)+this->size_facearrays*3*sizeof(double);
	    mem_fvl   = this->size_facevertexarrays*6*sizeof(int)+this->size_facevertexarrays*sizeof(double);
	    mem_total = mem_iwa+ mem_dwa + mem_vcl + mem_pffl + mem_pfil + mem_fl + mem_fvl;
	    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL2) {
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for int WORK ARRAY: %d bytes\n"        , mem_iwa);
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for DOUBLE WORK ARRAY: %d bytes\n"         , mem_dwa);
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for VERTEX COORDINATE LIST: %d bytes\n"    , mem_vcl);
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for POLYHEDRON FIRST FACE LIST: %d bytes\n", mem_pffl);
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for POLYHEDRON FACE INDEX LIST: %d bytes\n", mem_pfil);
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for FACE LISTS: %d bytes\n"                , mem_fl);
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for FACE VERTICES LISTS: %d bytes\n"       , mem_fvl);
	    }
	    else {
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "Total memory allocated: %d bytes\n", mem_total);
	    }
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "N_ORIGINAL_VERTICES = %d\n", this->n_original_vertices);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "NFACE               = %d\n", this->nface);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "N_WORK_VERTICES     = %d\n", this->n_work_vertices);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "NPOLH               = %d\n", this->npolh);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "N_POLYHEDRONFACES   = %d\n", this->n_polyhedronfaces);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "N_REFLEX_EDGES      = %d\n", n_reflex_edges);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "MINIMUM_ANGLE       = %f\n", minimum_angle);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "\n");
	}
    }



    if (this->ierr == 327) { // unresolved reflex edge
        if (retry_counter < 36) { // orig. 3
	    this->angacc -= IVP_PI / 36.0; // orig. 36.0
	    if (this->angacc > 0.0) {
		this->rdacc *= 0.95;
		this->ierr = 0;
		retry_counter++;
		IVP_IF(1) {
		    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
			ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "Retrying with <angacc=%f> and <rdacc=%f>\n\n", this->angacc, this->rdacc);
		    }
		}
		goto Retry_convex_decomposition;
	    }
	}
	IVP_IF(1) {
	    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "Premature abort due to difficulties in resolving some reflex edge(s).\n\n");
	    }
	}
	goto GEOMPACK_abort;
    }


    // -----------------------------------------------------------------------
    // Final output.
    // -----------------------------------------------------------------------

    IVP_IF(1) {
	IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "+++ Convex decomposition successful!\n\n");
	}

	IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL2) {
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "VCL (vertex coordinate list)\n");
	    for (i=1; i<=this->n_original_vertices; i++) {
		int j;
		printf("#%d : \t", i);
		for (j=1; j<=3; j++) {
		    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "%f   ", this->g_vcl[j+(i*3)-4]);
		}
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "\n");
	    }
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "\n");


	    for (i=1; i<=this->nface; i++) {
		int j;
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "i=%d\tfacesdata={", i);
		for (j=1; j<=3; j++) {
		    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "%d ", this->g_facesdata[j+(i*3)-4]);
		}
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "}  facestype[%d]=%d\tnormals={", i-1, this->g_facestype[i-1]);
		for (j=1; j<=3; j++) {
		    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "%f  ", this->g_normals[j+(i*3)-4]);
		}
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "\n");
	    }
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "\n\n");


	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "FaceVertexList >>FVL<<\n");
	    for (i=1; i<=this->n_work_vertices; i++) {
		int j;
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "#%d\t", i);
		for (j=1; j<=6; j++) {
		    switch (j) {
		    case 1:
			ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "LOC = ");
			break;
		    case 2:
			ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "FACN = ");
			break;
		    case 3:
			ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "SUCC = ");
			break;
		    case 4:
			ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "PRED = ");
			break;
		    case 5:
			ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "EDGA = ");
			break;
		    case 6:
			ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "EDGC = ");
			break;
		    }
		    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "%d\t", this->g_faceverticeslist[j+(i*6)-7]);
		}
		ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "\t(%f Grad)\n", this->g_edge_angles[i-1]/IVP_PI*180.0);
	    }
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL2, "\n\n");
	}

#if 0
	s_wsfe(&io___90);
	for (i=0; i<this->npolh; i++) {
	    do_fio(1, (char *)&this->g_polyhedronfirstfaceoffset[i], (ftnlen)sizeof(int));
	}
	e_wsfe();

	s_wsfe(&io___91);
	for (i__ = 1; i__ <= this->n_polyhedronfaces; i__++) {
	    int j;
	    do_fio(1, (char *)&i__, (ftnlen)sizeof(int));
	    for (j=1; j<=2; j++) {
		do_fio(1, (char *)&this->g_polyhedronfaceindices[j + (i__ << 1) - 3], (ftnlen)
			sizeof(int));
	    }
	}
	e_wsfe();
#endif

    }

GEOMPACK_abort:

    IVP_IF(1) {
	IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "Final GEOMPACK statistics:\n");
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "N_ORIGINAL_VERTICES   \t= %d (number of vertex coordinates)\n", this->n_original_vertices);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "NFACE \t= %d (number of faces in polyhedral decomposition)\n", this->nface);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "N_WORK_VERTICES \t= %d (number of positions used in FVL array)\n", this->n_work_vertices);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "NPOLH \t= %d (number of polyhedra in decomposition)\n", this->npolh);
	    ivp_debugmanager.dprint(IVP_DM_GEOMPACK_LEVEL1, "\n\n");
	}
    }

    *n_original_vertices_out	= this->n_original_vertices;
    *nface_out			= this->nface;
    *n_work_vertices_out	= this->n_work_vertices;
    *npolh_out			= this->npolh;
    *facep_out			= &this->g_facesdata[0];
    *fvl_out			= &this->g_faceverticeslist[0];
    *vcl_out			= &this->g_vcl[0];

out_of_memory:
    P_FREE(this->g_facestype);
    P_FREE(this->g_hashtable);
    P_FREE(this->g_polyhedronfirstfaceoffset);
    P_FREE(this->g_polyhedronfaceindices);
    P_FREE(this->g_intworkarray);
    P_FREE(this->g_doubleworkarray);
    P_FREE(this->g_edge_angles);
    P_FREE(this->g_normals);
    P_FREE(this->g_ev);

    return;
}

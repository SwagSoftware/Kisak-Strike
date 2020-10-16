// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#include <ivp_physics.hxx>
#include <ivp_betterdebugmanager.hxx>

#include <ivp_convex_decompositor.hxx>
#include <geompack.hxx>
#include <cstdint> // lwss - x64 fixes


void IVP_Concave_Polyhedron_Face::add_offset(int offset_in) {

    IVP_Concave_Polyhedron_Face_Pointoffset *p_offset = new IVP_Concave_Polyhedron_Face_Pointoffset();
    p_offset->offset = offset_in;
    this->point_offset.add(p_offset);

    return;
}


IVP_Concave_Polyhedron_Face::~IVP_Concave_Polyhedron_Face() {

    int i;
    for (i=0; i<this->point_offset.len(); i++) {
	IVP_Concave_Polyhedron_Face_Pointoffset *offset = this->point_offset.element_at(i);
	P_DELETE(offset);
    }
    return;
}


IVP_Convex_Subpart::~IVP_Convex_Subpart() {

    int i;
    for (i=0; i<this->points.len(); i++) {
	delete (this->points.element_at(i));
    }
    return;
}




/********************************************************************************
 *  Class:	    IVP_Convex_Subpart_Work
 *  Description:    ???
 *******************************************************************************/

class IVP_Convex_Subpart_Work {
public:
    IVP_U_Vector<int> offsets; // @@@SF: replace with IVP_VHash someday!!!

    void add_offset(int offset_in);
};


void IVP_Convex_Subpart_Work::add_offset(int offset_in) {

    int i;
    for (i=0; i<this->offsets.len(); i++) {
	if ( this->offsets.element_at(i) == (int *)offset_in ) {
	    return;
	}
    }
    this->offsets.add((int *)offset_in);
    return;
}


/******************************************************************************
 *  Method:	    perform_convex_decomposition_on_concave_polyhedron
 *  Description:    This method will split a supplied concave polyhedron into
 *		    several convex subparts. It will return a pointsoup for
 *		    each created convex subpart.
 *  Input:	    <concave_polyhedron_in>  data of the concave polyhedron
 *		    <params>                 some user-definable parameters
 *		    <convex_subparts_out>    vector that will be filled with
 *		                             pointers to IVP_Convex_Subpart
 *		                             objects
 *  Output:	    Number of created subparts
 *  Internal note:  The allocated memory passed to GEOMPACK is very likely
 *		    to get reallocated by GEOMPACK! Thus these pointers are
 *		    no longer valid as soon as GEOMPACK has been started.
 *		    DO NOT FREE THIS MEMORY YOURSELF!
 *****************************************************************************/

int IVP_Convex_Decompositor::perform_convex_decomposition_on_concave_polyhedron(IVP_Concave_Polyhedron *concave_polyhedron_in,
										IVP_Convex_Decompositor_Parameters *params_in,
										IVP_U_BigVector<IVP_Convex_Subpart> *convex_subparts_out) {

    int i;

    int      n_points;  // number of points in concave object
    int      n_faces;   // number of faces in concave object
    double * vcl;       // vertex coordinate list: stores the points
    int *    fvl;       // face vertex list: stores the offsets (into vcl) of points for each face
    int *    facep;     // face pointer: stores offset (into fvl) of first point for face

    int      nvc_out;   // total number of points in convex subparts; currently unused!
    int      nface_out; // total number of faces in convex subparts; currently unused!
    int      nvert_out; // total number of entries in 'fvl_out'
    int      npolh_out; // total number of convex subparts
    int *    facep_out = NULL; // stores all faces of all convex subparts
    int *    fvl_out = NULL;   // stores relationship between points and faces
    double * vcl_out = NULL;   // stores all points of all convex subparts

    // ----------------------------------------------
    // prepare 'vertex coordinates list' for GEOMPACK
    // ----------------------------------------------
    {
	n_points = concave_polyhedron_in->points.len();

	// alloc an array for n points, consisting of 3 floats each
	vcl = (double *)p_calloc(n_points*3, sizeof(double));
	if ( !vcl ) {
	    IVP_IFDEBUG(IVP_DEBUG_IPION_ERROR_MSG) {
		ivp_debugmanager.dprint(IVP_DEBUG_IPION_ERROR_MSG,
					"IVP_Convex_Decompositor::perform_convex_decomposition_on_concave_polyhedron()\n"
					"Not enough memory to allocate 'points_list' (needed: %d bytes)\n", n_points*3*sizeof(float));
	    }
	    return(0);
	}

	// fill array with points
	IVP_U_BigVector<IVP_U_Point> *points = &concave_polyhedron_in->points;
	for (i=0; i<n_points; i++) {
	    vcl[(i*3)+0] = points->element_at(i)->k[0];
	    vcl[(i*3)+1] = points->element_at(i)->k[1];
	    vcl[(i*3)+2] = points->element_at(i)->k[2];
	}
    }


    // ---------------------------------------------------------------
    // prepare 'face pointer list' and 'face vertex list' for GEOMPACK
    // ---------------------------------------------------------------
    {
	n_faces = concave_polyhedron_in->faces.len();

	IVP_U_BigVector<IVP_Concave_Polyhedron_Face> *faces = &concave_polyhedron_in->faces;
	int fvl_length = 0;

	// sum up total number of all points of all faces
	for (i=0; i<n_faces; i++) {
	    fvl_length += faces->element_at(i)->point_offset.len();
	}

	// allocate memory
	fvl   = (int *)p_calloc(fvl_length,   sizeof(int));
	facep = (int *)p_calloc(n_faces+1 , 3*sizeof(int));

	int entry_ctr = 0;
	for (i=0; i<n_faces; i++) {

	    // fill 'face pointer list' will offset into 'fvl' for each face
	    facep[i*3] = entry_ctr + 1;

	    // fill 'face vertex list' with offsets of all the faces' points
	    {
		int j;
		IVP_Concave_Polyhedron_Face *face = faces->element_at(i);
		int n_points_of_face = face->point_offset.len();
		for (j=0; j<n_points_of_face; j++) {
		    IVP_Concave_Polyhedron_Face_Pointoffset *point_offset = face->point_offset.element_at(j);
		    fvl[entry_ctr] = point_offset->offset+1;
		    entry_ctr++;
		}
	    }

	}
	facep[i*3] = entry_ctr + 1; // one final offset
    }

    // -------------------------------------------
    // perform convex decomposition using GEOMPACK
    // -------------------------------------------
    struct geompack_parameters params;

    params.tolin  = params_in->tolin;
    params.angacc = params_in->angacc;
    params.rdacc  = params_in->rdacc;

    params.nvc       = n_points;
    params.nface     = n_faces;
    params.vcl       = vcl;
    params.facep     = facep;
    params.fvl       = fvl;
    params.nvc_out   = &nvc_out;
    params.nface_out = &nface_out;
    params.nvert_out = &nvert_out;
    params.npolh_out = &npolh_out;
    params.vcl_out   = &vcl_out;
    params.facep_out = &facep_out;
    params.fvl_out   = &fvl_out;

    // IMPORTANT: DO NOT free the above allocated memory yourself! GEOMPACK might reallocate it!
    IVP_Geompack geompack;
    geompack.decompose(&params);


    // ------------------------
    // process GEOMPACK results
    // ------------------------
    {
	IVP_U_Vector<IVP_Convex_Subpart_Work> subparts_offsets;

	// create all (empty) subparts
	for (i=0; i<npolh_out; i++) {

	    IVP_Convex_Subpart *subpart = new IVP_Convex_Subpart();
	    convex_subparts_out->add(subpart);

	    IVP_Convex_Subpart_Work *subpart_work = new IVP_Convex_Subpart_Work();
	    subparts_offsets.add(subpart_work);

	}
#if 0
FILE *fp = fopen("log.txt", "w");
	fprintf(fp, "VCL (vertex coordinate list)\n");
	for (i=1; i<=nvc_out; i++) {
	    fprintf(fp, "#%d : \t", i);
	    int j;
	    for (j=1; j<=3; j++) {
		fprintf(fp, "%f\t", vcl_out[j+(i*3)-4]);
	    }
	    fprintf(fp, "\n");
	}
	fprintf(fp, "\n");


	for (i=1; i<=nface_out; i++) {
	    fprintf(fp, "i=%d\tfacep={", i);
	    int j;
	    for (j=1; j<=3; j++) {
		fprintf(fp, "%d\t", facep_out[j+(i*3)-4]);
	    }
	    fprintf(fp, "\n");
	}
	fprintf(fp, "\n\n");
	fprintf(fp, "FaceVertexList >>FVL<<\n");
	for (i=1; i<=nvert_out; i++) {
	    fprintf(fp, "#%d\t", i);
	    int j;
	    for (j=1; j<=6; j++) {
		switch (j) {
		case 1:
		    fprintf(fp, "LOC = ");
		    break;
		case 2:
		    fprintf(fp, "FACN = ");
		    break;
		case 3:
		    fprintf(fp, "SUCC = ");
		    break;
		case 4:
		    fprintf(fp, "PRED = ");
		    break;
		case 5:
		    fprintf(fp, "EDGA = ");
		    break;
		case 6:
		    fprintf(fp, "EDGC = ");
		    break;
		}
		fprintf(fp, "%d\t", fvl_out[j+(i*6)-7]);
	    }
	    fprintf(fp, "\n");
	}
	fprintf(fp, "\n\n");
fclose(fp);
#endif
	// fill subpart offsets lists with offsets
	for (i=0; i<nvert_out; i++) {
	    int face_number = fvl_out[(i*6)+1]-1;
	    int subpart_number = facep_out[(face_number*3)+1];
	    if ( subpart_number < 0 ) subpart_number *= -1;
	    subpart_number--;

	    IVP_Convex_Subpart_Work *subpart_work = subparts_offsets.element_at(subpart_number);
	    subpart_work->add_offset(fvl_out[(i*6)+0]);
	}

	// traverse all subpart offset lists and copy the corresp. points
	// to subpart's pointsoups
	for (i=0; i<npolh_out; i++) {
	    IVP_Convex_Subpart_Work *subpart_work = subparts_offsets.element_at(i);
	    IVP_Convex_Subpart *subpart = convex_subparts_out->element_at(i);
	    IVP_IF(1) {
		IVP_IFDEBUG(IVP_DM_CONVEX_DECOMPOSITOR) {
		    ivp_debugmanager.dprint(IVP_DM_CONVEX_DECOMPOSITOR, "Points of convex subpart:\n");
		}
	    }
	    int j;
	    for (j=0; j<subpart_work->offsets.len(); j++) {
        //lwss - x64 fixes
		//int offset = (int)subpart_work->offsets.element_at(j)-1;
		intptr_t offset = (intptr_t)subpart_work->offsets.element_at(j)-1;
		//lwss end
		IVP_U_Point *point = new IVP_U_Point();
		point->k[0] = vcl_out[(offset*3)+0];
		point->k[1] = vcl_out[(offset*3)+1];
		point->k[2] = vcl_out[(offset*3)+2];
		IVP_IF(1) {
		    IVP_IFDEBUG(IVP_DM_CONVEX_DECOMPOSITOR) {
			ivp_debugmanager.dprint(IVP_DM_CONVEX_DECOMPOSITOR, "   %.6f %.6f %.6f\n", point->k[0], point->k[1], point->k[2]);
		    }
		}
		subpart->points.add(point);
	    }
	    P_DELETE(subpart_work);
	}
    }

    P_FREE(facep_out);
    P_FREE(fvl_out);
    P_FREE(vcl_out);


    return(npolh_out); // return total number of convex subparts
}


/*
 *  Object converter/optimizer
 */


#include <ivp_physics.hxx>
#include <ctype.h>
#ifndef WIN32
#	pragma implementation "ivp_surbuild_3ds.hxx"
#endif

#include <ivp_surbuild_3ds.hxx>
#include <ivu_geometry.hxx>
#include <ivp_convex_decompositor.hxx>
#include <ivp_surbuild_polyhdrn_cncv.hxx>
#include <3dsimport_load.hxx>

typedef struct
{
	dword ix,iy,iz;
	float x,y,z;
	dword iu,iv;
	byte  marked;

} H3dsMapVert;


#include "3dsimport_load.cxx"
#include "3dsimport_out.cxx"


IVP_Template_SurfaceBuilder_3ds::IVP_Template_SurfaceBuilder_3ds(){
    this->scale = 1.0f;
}

// OK, that was ugly but this way I
// don't have to mess with the linker.


// Default object name to set as public label
#define DEFNAME "rawobj"

int flags = 0;

#define  VERBOSE    0x0001    /* Verbose mode on                  */
#define  OVERWR     0x0002    /* Don't sak for file overwrite     */
#define  BINARY     0x0004    /* Binary output                    */
#define  ASSEMBLY   0x0008    /* Assembly source output           */
#define  CENTRE     0x0010    /* Centre objects                   */
#define  SCALE      0x0020    /* Scale objects                    */
#define  NOMAPFIX   0x0040    /* Don't fix bad mapping values     */
#define  NORMNULL   0x0080    /* Don't remove null faces          */
#define  NORMUNUSED 0x0100    /* Don't remove unused vertices     */
#define  NORMDUP    0x0200    /* Don't remove duplicated vertices */
#define  NOMAPPING  0x0400    /* Don't output mapping values      */


void FixMaps(H3dsScene * scene)
{
	for(int n=0; n<scene->meshobjs; n++) {
		H3dsMeshObj * mo = &scene->meshobjlist[n];
		if(mo->maps) {
			// Find maximum mapping value
			float32 max=0.0;
			for(int m=0; m<mo->maps; m++) {
				H3dsMap * map = &mo->maplist[m];
				if(map->u > max) max=map->u;
				if(map->v > max) max=map->v;
			}

			if(max > 1.0) {
				if(flags & VERBOSE)
					ivp_message("%-14s bad mapping %.3f, scaling...\n",
							mo->name, max);
				float32 scale=1.0/max;
				for(int mm=0; mm<mo->maps; mm++) {
					H3dsMap * mmap = &mo->maplist[mm];
					mmap->u *= scale;
					mmap->v *= scale;
				}
			}
		}
	}
}

void ScaleMaps(H3dsScene * scene, float32 us, float32 vs)
{
	for(int n=0; n<scene->meshobjs; n++) {
		H3dsMeshObj * mo = &scene->meshobjlist[n];
		for(int m=0; m<mo->maps; m++) {
			H3dsMap * map = &mo->maplist[m];
			map->u *= us;
			map->v *= vs;
		}
	}
}

void FindCentrePoints(H3dsScene * scene)
{
	float32 xmino= 1e30f, ymino= 1e30f, zmino= 1e30f;
	float32 xmaxo=-1e30f, ymaxo=-1e30f, zmaxo=-1e30f;
	for(int n=0; n<scene->meshobjs; n++) {
		float32 xmin= 1e30f, ymin= 1e30f, zmin= 1e30f;
		float32 xmax=-1e30f, ymax=-1e30f, zmax=-1e30f;
		H3dsMeshObj * mo = &scene->meshobjlist[n];
		for(int v=0; v<mo->verts; v++) {
			H3dsVert * vrt=&mo->vertlist[v];
			if(vrt->x > xmax) xmax=vrt->x;
			if(vrt->x < xmin) xmin=vrt->x;
			if(vrt->y > ymax) ymax=vrt->y;
			if(vrt->y < ymin) ymin=vrt->y;
			if(vrt->z > zmax) zmax=vrt->z;
			if(vrt->z < zmin) zmin=vrt->z;
		}
		mo->centre.x = xmax-(xmax-xmin)*0.5;
		mo->centre.y = ymax-(ymax-ymin)*0.5;
		mo->centre.z = zmax-(zmax-zmin)*0.5;

		if(mo->centre.x > xmaxo) xmaxo=mo->centre.x;
		if(mo->centre.x < xmino) xmino=mo->centre.x;
		if(mo->centre.y > ymaxo) ymaxo=mo->centre.y;
		if(mo->centre.y < ymino) ymino=mo->centre.y;
		if(mo->centre.z > zmaxo) zmaxo=mo->centre.z;
		if(mo->centre.z < zmino) zmino=mo->centre.z;
	}
	scene->centre.x = xmaxo-(xmaxo-xmino)*0.5;
	scene->centre.y = ymaxo-(ymaxo-ymino)*0.5;
	scene->centre.z = zmaxo-(zmaxo-zmino)*0.5;
}

void Move(H3dsScene * scene, float32 x, float32 y, float32 z)
{
    scene->centre.x+=x;
    scene->centre.y+=y;
    scene->centre.z+=z;
	for(int n=0; n<scene->meshobjs; n++) {
		H3dsMeshObj * mo = &scene->meshobjlist[n];
        mo->centre.x+=x;
        mo->centre.y+=y;
        mo->centre.z+=z;
        for(int v=0; v<mo->verts; v++) {
			H3dsVert * vrt=&mo->vertlist[v];
			vrt->x+=x;
			vrt->y+=y;
			vrt->z+=z;
		}
	}
}

void Scale(H3dsScene * scene, float32 x, float32 y, float32 z)
{
    scene->centre.x*=x;
    scene->centre.y*=y;
    scene->centre.z*=z;
	for(int n=0; n<scene->meshobjs; n++) {
		H3dsMeshObj * mo = &scene->meshobjlist[n];
        mo->centre.x*=x;
        mo->centre.y*=y;
        mo->centre.z*=z;
		for(int v=0; v<mo->verts; v++) {
			H3dsVert * vrt=&mo->vertlist[v];
            vrt->x*=x;
            vrt->y*=y;
            vrt->z*=z;
		}
	}
}

void ConvertFloatsToInts(H3dsScene * scene)
{
	for(int n=0; n<scene->meshobjs; n++) {
		H3dsMeshObj * mo = &scene->meshobjlist[n];
		for(int v=0; v<mo->verts; v++) {
			H3dsVert * vrt = &mo->vertlist[v];
			vrt->ix = (dword) vrt->x;
			vrt->iy = (dword) vrt->y;
			vrt->iz = (dword) vrt->z;
		}
		for(int m=0; m<mo->maps; m++) {
			H3dsMap * map = &mo->maplist[m];
			map->iu = (dword) map->u;
			map->iv = (dword) map->v;
		}
		mo->centre.ix = (dword) mo->centre.x;
		mo->centre.iy = (dword) mo->centre.y;
		mo->centre.iz = (dword) mo->centre.z;
	}
	scene->centre.ix = (dword) scene->centre.x;
	scene->centre.iy = (dword) scene->centre.y;
	scene->centre.iz = (dword) scene->centre.z;
}

int RemoveNullFaces(H3dsScene * scene)
{
	int bad=0;
	for(int o=0; o<scene->meshobjs; o++) {
		H3dsMeshObj * mo = &scene->meshobjlist[o];
		for(int f=0; f<mo->faces; f++) {

			int p0 = (int) mo->facelist[f].p0;
			int p1 = (int) mo->facelist[f].p1;
			int p2 = (int) mo->facelist[f].p2;

            dword p0x = mo->vertlist[p0].ix;
            dword p0y = mo->vertlist[p0].iy;
            dword p0z = mo->vertlist[p0].iz;
            dword p1x = mo->vertlist[p1].ix;
            dword p1y = mo->vertlist[p1].iy;
            dword p1z = mo->vertlist[p1].iz;
            dword p2x = mo->vertlist[p2].ix;
			dword p2y = mo->vertlist[p2].iy;
            dword p2z = mo->vertlist[p2].iz;

			if((p0x==p1x && p0y==p1y && p0z==p1z) ||
			   (p0x==p2x && p0y==p2y && p0z==p2z) ||
			   (p1x==p2x && p1y==p2y && p1z==p2z))   {
                // We found a null face! I.e. a line or just a point.
				bad++;
				mo->faces--;
				// Insert the other faces a slot down
				for(int ff=f; ff<mo->faces; ff++) {
					mo->facelist[ff] = mo->facelist[ff+1];
				}
			}
		}
	}
	return bad;
}

int RemoveUnusedVerts(H3dsScene * scene)
{
	int bad=0;
	for(int o=0; o<scene->meshobjs; o++) {
		H3dsMeshObj * mo = &scene->meshobjlist[o];
		for(int v=0; v<mo->verts; v++) {
			// Check if vertice v is used in any of the faces
			int used=0;
			for(int f=0; f<mo->faces; f++) {
				H3dsFace * fac = &mo->facelist[f];
				if((int)fac->p0==v || (int)fac->p1==v || (int)fac->p2==v) {
					// This vertice is used here
					used=1;
					break;
				}
			}
			if(!used) {
				// We found a vertice that is not used in any face.
				bad++;
				mo->verts--;
				// Insert the other vertices a slot down
                for(int vv=v; vv<mo->verts; vv++) 
					mo->vertlist[vv] = mo->vertlist[vv+1];
				// Also move the mapping vertices
                if(mo->maps) {
                    mo->maps--;
                    for(int mm=v; mm<mo->maps; mm++) 
                        mo->maplist[mm] = mo->maplist[mm+1];
                }
				// Modify the faces to reflect the moved vertices
				for(int ff=0; ff<mo->faces; ff++) {
					H3dsFace * ffac = &mo->facelist[ff];
					if(ffac->p0 >= v) ffac->p0--;
					if(ffac->p1 >= v) ffac->p1--;
					if(ffac->p2 >= v) ffac->p2--;
				}
			}
		}
	}
	return bad;
}

void FindExchange(H3dsScene * scene, int find, int exchange)
{
    // Find all references to the 'find' vertice and replace
    // them with references to the 'exchange' vertice
	for(int o=0; o<scene->meshobjs; o++) {
		H3dsMeshObj * mo = &scene->meshobjlist[o];
		for(int f=0; f<mo->faces; f++) {
			H3dsFace * fa = &mo->facelist[f];
			if(fa->p0 == find) fa->p0 = exchange;
			if(fa->p1 == find) fa->p1 = exchange;
			if(fa->p2 == find) fa->p2 = exchange;
		}
	}
}

int RemoveDupVerts(H3dsScene * scene, H3dsMapVert * vrtmap, int verts)
{
    int vrttop=0, dot=0;
	for(int currvtx=0; currvtx<verts; currvtx++) {

        // Only process those vertices that has not been
        // processed already.
        if(vrtmap[currvtx].marked == 0) {

            // OK, we have a vertex, currvtx. Try to find all other
            // vertices that have the same x,y,z values.
            for(int runvtx=currvtx+1; runvtx<verts; runvtx++) {
    
                // Skip all vertices that has been processed already.
                // We already know that they don't have the same values.
                if(vrtmap[runvtx].marked == 1)
                    continue;
    
                // If we find another vertex with the same x,y,z values
                // we must find and adjust all the indexes that point
                // to that vertex so that they point to currvtx.
                if(vrtmap[runvtx].ix == vrtmap[currvtx].ix &&
                   vrtmap[runvtx].iy == vrtmap[currvtx].iy &&
                   vrtmap[runvtx].iz == vrtmap[currvtx].iz)
                {
                    // Make them point to the top of our optimized array
                    FindExchange(scene, runvtx, vrttop);
    
                    // Mark it so we don't process it again.
                    vrtmap[runvtx].marked=1;
                }
            }
    
            // Now find all other indexes that points to currvtx
            // and adjust them to the top of our optimized array, vrttop.
            FindExchange(scene, currvtx, vrttop);
    
            // Put currvtx on top of our optimized array.
            vrtmap[vrttop] = vrtmap[currvtx];
            vrttop++;
        }

        // Print some dots so that the user don't fall asleep
		if((flags & VERBOSE) && dot++>20) {
			ivp_message( ".");
			dot=0;
		}
	}
	return vrttop;
}

void AdjustFaceIndexes(H3dsScene * scene)
{
	int m=0;
	for(int n=0; n<scene->meshobjs; n++) {
		H3dsMeshObj * mo = &scene->meshobjlist[n];
		for(int f=0; f<mo->faces; f++) {
			H3dsFace * fa = &mo->facelist[f];
			fa->p0 += m;
			fa->p1 += m;
			fa->p2 += m;
		}
		m+=mo->verts;
	}
}

void CollectVertsAndMaps(H3dsScene * scene, H3dsMapVert * vrtmap)
{
	int vn=0, mn;
	for(int n=0; n<scene->meshobjs; n++) {
		H3dsMeshObj * mo = &scene->meshobjlist[n];
		//		H3dsMeshObj * mo = &scene->meshobjlist[0];
		mn=vn;
		for(int v=0; v<mo->verts; v++) {
			vrtmap[vn].ix=mo->vertlist[v].ix;
			vrtmap[vn].iy=mo->vertlist[v].iy;
			vrtmap[vn].iz=mo->vertlist[v].iz;
			vrtmap[vn].x=mo->vertlist[v].x;
			vrtmap[vn].y=mo->vertlist[v].y;
			vrtmap[vn].z=mo->vertlist[v].z;
			vn++;
		}
		for(int m=0; m<mo->maps; m++) {
			vrtmap[mn].iu=mo->maplist[m].iu;
			vrtmap[mn].iv=mo->maplist[m].iv;
			mn++;
		}
		if(mn<vn) {
			if(flags & VERBOSE)
				ivp_message( "%-14s missing mapping, set to zero...\n",
						mo->name);
			for(int mmn=mn; mmn<vn; mmn++) {
				vrtmap[mmn].iu=0;
				vrtmap[mmn].iv=0;
			}
		}
	}
}

#define IVP_MERGE_EPS 10e-4

IVP_DOUBLE ipoe_calc_s_val(const IVP_U_Point *p_world, const IVP_U_Point *vec, const IVP_U_Point *tp, const IVP_U_Point *tp_next)
{
     // calcs intersect pos
     // von lot von p auf this (rel. zu this)
//    IVP_U_Point *tp      = give_world_coords_AT(edge, m_cache_edge);
//    IVP_U_Point *tp_next = give_world_coords_AT(edge->get_next(), m_cache_edge);
    
    IVP_U_Point vec1, vec2;
    vec1.subtract(tp_next, tp);
    vec2.subtract(p_world, tp);
    IVP_DOUBLE i_len = 1.0 / vec->fast_real_length();
    i_len *= i_len;
    IVP_DOUBLE s = vec1.dot_product(&vec2);
    s *= i_len;
    return s;
}

IVP_BOOL is_point_on_edge(IVP_U_Point *point, IVP_U_Point *edge_startpoint, IVP_U_Point *edge_endpoint) {

    IVP_U_Point vec(edge_endpoint);
    vec.subtract(edge_startpoint, &vec);
    IVP_U_Straight edge(edge_startpoint, &vec);
    if ( edge.get_quad_dist_to_point(point) < IVP_MERGE_EPS*IVP_MERGE_EPS ) {
	IVP_DOUBLE s_val = ipoe_calc_s_val(point, &vec, edge_startpoint, edge_endpoint);
	if ( (0.0 < s_val) && (s_val < 1.0) ) {
	    return(IVP_TRUE);
	}
	else {
	    return(IVP_FALSE);
	}
    }
    else {
	return(IVP_FALSE);
    }
}

void repair_geometry(IVP_Concave_Polyhedron *concave_polyhedron) {

    // process all points in object.
    int x;
    for (x=0; x<concave_polyhedron->points.len(); x++) {
	IVP_U_Point *p = concave_polyhedron->points.element_at(x);

	// for each point: process all faces in object.
	int y;
	for (y=0; y<concave_polyhedron->faces.len(); y++) {

	    IVP_Concave_Polyhedron_Face *face = concave_polyhedron->faces.element_at(y);
	    int first_point = face->point_offset.element_at(0)->offset;

	    // for each face process all edges in face.
	    int z;
	    int n_points = face->point_offset.len();
	    for (z=0; z<n_points; z++) {

		int start, end;
		if ( z == n_points-1 ) {
		    start = face->point_offset.element_at(z+0)->offset;
		    end   = first_point;
		}
		else {
		    start = face->point_offset.element_at(z+0)->offset;
		    end   = face->point_offset.element_at(z+1)->offset;
		}

		// if current point is either start or end point of current edge we can
		// gladly skip it.
		if ( p == concave_polyhedron->points.element_at(start) ) {
		    continue;
		}
		if ( p == concave_polyhedron->points.element_at(end) ) {
		    continue;
		}

		if ( is_point_on_edge(p, concave_polyhedron->points.element_at(start), concave_polyhedron->points.element_at(end)) ) {
		    printf("T-Junction found for point with offset <%d>\n", concave_polyhedron->points.index_of(p));
		    IVP_Concave_Polyhedron_Face_Pointoffset *new_offset = new IVP_Concave_Polyhedron_Face_Pointoffset();
		    new_offset->offset = concave_polyhedron->points.index_of(p);
		    face->point_offset.insert_after(z, new_offset);
		    n_points++;
		}
//		else {
//		    printf("Valid point\n");
//		}

	    }

	}
    }
    
    return;
}

IVP_Concave_Polyhedron * IVP_SurfaceBuilder_3ds::convert_3ds_to_concave(const char *filename, IVP_Template_SurfaceBuilder_3ds *params){
    
//****
#ifndef GEKKO
//****

    char * infn=0, * outfn=0; // , * name=DEFNAME;
	FILE * inf, * outf;
	int n;
	H3dsScene * scene;
//    float32 xscale, yscale, zscale;

#if 0
	argc--;
	argv++;

	if(!argc) {
        ivp_message(
               "3D Studio native objectfile converter v2.0\n"
			   "by Mats Byggmastar 1996 Espoo, Finland\n"
			   "Use:  3DSCO [params...] inputfile.3DS [outputfile]\n"
               "params:\n"
               " -o name   Object name (default "DEFNAME")\n"
               " -v        Verbose mode on\n"
               " -a        Assembly source output\n"
               " -b        Binary output (you must specify outputfile)\n"
               " -c        Centre objects\n"
               " -s x y z  Scale objects (x,y,z = floats)\n"
               " -y        Don't ask for file overwrite\n"
               " -f        Don't fix bad mapping coordinates\n"
               " -m        Don't output mapping coordinates\n"
               " -n        Don't remove null faces\n"
               " -u        Don't remove unused vertices\n"
               " -d        Don't remove duplicated vertices\n"
               );

		return 1;
	}

	// Get the parameters and filenames

	for(n=0; n<argc; n++) {
		if(argv[n][0] == '-' || argv[n][0] == '/') {
			switch(toupper(argv[n][1])) {
            case 'O': if(n+1<argc) { name=argv[++n]; break; }
                      else { ivp_message( "Missing object name!\n");
                             return 1; }
            case 'S': if(n+3<argc) { flags |= SCALE;
                                     xscale = atof(argv[++n]);
                                     yscale = atof(argv[++n]);
                                     zscale = atof(argv[++n]);
                                     break; }
                      else { ivp_message( "Missing scale value!\n");
                             return 1; }
            case 'V': flags |= VERBOSE;    break;
            case 'A': flags |= ASSEMBLY;   break;
            case 'B': flags |= BINARY;     break;
			case 'C': flags |= CENTRE;     break;
            case 'Y': flags |= OVERWR;     break;
			case 'F': flags |= NOMAPFIX;   break;
            case 'M': flags |= NOMAPPING;  break;
			case 'N': flags |= NORMNULL;   break;
			case 'U': flags |= NORMUNUSED; break;
			case 'D': flags |= NORMDUP;    break;
            default:  ivp_message( "Bad param: %s\n",argv[n]);
					  return 1;
			}
		} else {
			if(!infn)
				infn = argv[n];
			else if(!outfn)
				outfn = argv[n];
			else {
                ivp_message( "Too many filenames!\n");
				return 1;
			}
		}
	}
#else
	infn = (char *)filename;
	outf = NULL;
#endif

	if(!infn) {
        ivp_message( "No inputfile specified!\n");
		return NULL;
	}

	// Open inputfile

	if(!(inf = fopen(infn, "rb"))) {
	    ivp_message( "Failed to open %s\n", infn);
	    return NULL;
	}

	if(!outfn && (flags & BINARY) != 0) {
	    ivp_message( "Missing output filename!\n");
		fclose(inf);
		return NULL;
	}

	// Open, create or redirect outputfile

	if(outfn) {
		if((outf = fopen(outfn, "r+b")) != 0) {
			if((flags & OVERWR) == 0) {
				ivp_message( "%s exist, overwrite [y/n] ", outfn);
				if(toupper(getc(stdin)) != 'Y') {
					fclose(outf);
					fclose(inf);
					return 0;
				}
				ivp_message( "\n");
				fclose(outf);
				if((outf = fopen(outfn, "w+b")) == 0) {
                    ivp_message( "Unable to reopen %s\n", outfn);
					fclose(inf);
					return NULL;
				}
			}
		} else {
			if((outf = fopen(outfn, "w+b")) == 0) {
                ivp_message( "Unable to create %s\n", outfn);
				fclose(inf);
				return NULL;
			}
		}
	} else {
		// Use stdout if not binary output and no outputfile was specified.
		outf = stdout;
	}

    // Here we have both the source and destination files opened as:
	// FILE * inf  -> source file
	// FILE * outf -> destination file (could be stdout)

	long size;
	if(fseek(inf, 0, SEEK_END)) {
        ivp_message( "Error seeking %s\n", infn);
		if(outf!=stdout) fclose(outf);
		fclose(inf);
		return NULL;
	}

	if((size=ftell(inf)) == -1L) {
	    ivp_message( "Error seeking %s\n", infn);
		if(outf!=stdout) fclose(outf);
		fclose(inf);
		return NULL;
	}
	rewind(inf);

	if((scene = HRead3dsScene(inf, 0, size)) == 0) {
        ivp_message( "Failed to load %s\n", infn);
		if(outf!=stdout) fclose(outf);
		fclose(inf);
		return NULL;
	}
	fclose(inf);

    // At this point we have all object's data loaded into memory.
    // H3dsScene * scene  -> object data

#if 0
	if(flags & VERBOSE) {
		// Print some interesting information about what we have
		ivp_message( "3DS data size: %ld byte\n", size);
		ivp_message( "object-name     faces vertices  maps  matrix\n");
		int f=0, v=0, m=0;
		for(n=0; n<scene->meshobjs; n++) {
			H3dsMeshObj * mo = &scene->meshobjlist[n];
			char * mtrx="yes";
			if(mo->matrix==0)
				mtrx="no";
			ivp_message( "%-14s  %5d  %5d  %5d    %s\n",
							mo->name, mo->faces, mo->verts, mo->maps, mtrx);
			f+=mo->faces;
			v+=mo->verts;
			m+=mo->maps;
		}
		if(scene->meshobjs>1)
			ivp_message( "%d faces, %d vertices, %d maps in "
							"%d objects loaded\n",
							f, v, m, scene->meshobjs);
	}

	// Do any rotating, moving, scaling here before
	// we convert all floats to integers.

	if(!(flags & NOMAPFIX))
		FixMaps(scene);

	ScaleMaps(scene, 65535.0, 65535.0);

	FindCentrePoints(scene);

	if(flags & CENTRE)
		Move(scene, -scene->centre.x, -scene->centre.y, -scene->centre.z);

    if(flags & SCALE)
        Scale(scene, xscale, yscale, zscale);

//	ConvertFloatsToInts(scene);

	// From this point we'll only work with integers.

	if(!(flags & NORMNULL)) {
		n = RemoveNullFaces(scene);
		if(flags & VERBOSE)
			ivp_message( "Removed %d null faces\n", n);
	}

	if(!(flags & NORMUNUSED)) {
		n = RemoveUnusedVerts(scene);
		if(flags & VERBOSE)
			ivp_message( "Removed %d unused vertices\n", n);
	}
#endif

	if ( params->scale != 1.0f){
	    Scale(scene, params->scale, params->scale, params->scale);
	}
	// Prepare to collect all vertices to one big array

	int verts=0;
	for(n=0; n<scene->meshobjs; n++) {
		int v=scene->meshobjlist[n].verts;
		if(scene->meshobjlist[n].maps > v) {
			ivp_message( "%-14s more maps than vertices, quitting!\n",
					scene->meshobjlist[n].name);
			if(outf!=stdout) fclose(outf);
			return NULL;
		}
		// Get the total number of vertices in all objects
		verts+=v;
	}

	H3dsMapVert * vrtmap = (H3dsMapVert *) p_malloc(verts*sizeof(H3dsMapVert));
	if(!vrtmap) {
		ivp_message( "Failed to allocate mem for vertice array\n");
		HFree3dsScene(scene);
		fclose(outf);
		return NULL;
	}
	memset(vrtmap, 0, verts*sizeof(H3dsMapVert));

    // Do it! Collect them

	CollectVertsAndMaps(scene, vrtmap);

	// Adjust the face indexes to the new vertice array
	AdjustFaceIndexes(scene);

	if(!(flags & NORMDUP)) {
		if(flags & VERBOSE)
			ivp_message( "Removing duplicated vertices ");
		n = RemoveDupVerts(scene, vrtmap, verts);
		if(flags & VERBOSE)
			ivp_message( "\nRemoved %d duplicated vertices\n", verts-n);
		verts=n;
	}

#if 0
    // Output data

    int f=0;

    if(!(flags & NOMAPPING))
        f |= 1;

    if(flags & BINARY) {
        OutpHeadBin(outf, scene, verts, f);
        OutpVertsBin(outf, vrtmap, verts, f);
        OutpCentresBin(outf, scene);
        OutpObjectsBin(outf, scene);
    }
    else
    if(flags & ASSEMBLY) {
        OutpHeadAsm(outf, scene, verts, name, f);
        OutpVertsAsm(outf, vrtmap, verts, name, f);
        OutpCentresAsm(outf, scene, name);
        OutpObjectsAsm(outf, scene, name);
    }
    else {
        OutpHead(outf, scene, verts, name, f);
        OutpVerts(outf, vrtmap, verts, name, f);
        OutpCentres(outf, scene, name);
        OutpObjects(outf, scene, name);
    }

	P_FREE(vrtmap);
	HFree3dsScene(scene);
    if(outf!=stdout) fclose(outf);
#else
    IVP_Concave_Polyhedron *concave_polyhedron = new IVP_Concave_Polyhedron();

#if 1
    int x;
    for (x=0; x<(int)verts; x++) {
	H3dsMapVert *vm = &vrtmap[x];
	IVP_U_Point *point = new IVP_U_Point();
	point->k[0] = vm->x;
	point->k[1] = vm->y;
	point->k[2] = vm->z;
	concave_polyhedron->points.add(point);
    }

    for (int o = 0; o < scene->meshobjs; o++){
	H3dsMeshObj * mo = &scene->meshobjlist[o];
	for (x=0; x<(int)mo->faces; ) {
	    H3dsFace * fa = &mo->facelist[x++];
	    IVP_Concave_Polyhedron_Face *face = new IVP_Concave_Polyhedron_Face();
	    face->add_offset(fa->p0);
	    face->add_offset(fa->p1);
	    face->add_offset(fa->p2);
	    concave_polyhedron->faces.add(face);
	}
    }
#endif

    repair_geometry(concave_polyhedron);

#endif
    return concave_polyhedron;
    
    
 //****
#else // GEKKO
//****
	return 0;
//****
#endif // GEKKO
//****


}

#include <ivp_surbuild_pointsoup.hxx>

class P_Hardware;
int p_graphlib_robust_convert_3dmax_object_to_compact_ledges(P_Hardware *hw, const char *filename, IVP_U_BigVector<IVP_Compact_Ledge> *ledges) {

#ifdef WIN32    

//    P_Hardware_W95 *w95_hw = (P_Hardware_W95 *)hw;
flags |= NORMDUP;
    char * infn=0, * name=DEFNAME;
	FILE * inf, * outf;
	int n;
	H3dsScene * scene;
//    float32 xscale, yscale, zscale;

	infn = (char *)filename;
	outf = NULL;

	if(!infn) {
        ivp_message( "No inputfile specified!\n");
		return 1;
	}

	// Open inputfile

	if(!(inf = fopen(infn, "rb"))) {
        ivp_message( "Failed to open %s\n", infn);
		return 1;
	}

    // Here we have both the source and destination files opened as:
	// FILE * inf  -> source file
	// FILE * outf -> destination file (could be stdout)

	long size;
	if(fseek(inf, 0, SEEK_END)) {
        ivp_message( "Error seeking %s\n", infn);
		if(outf!=stdout) fclose(outf);
		fclose(inf);
		return 1;
	}

	if((size=ftell(inf)) == -1L) {
        ivp_message( "Error seeking %s\n", infn);
		if(outf!=stdout) fclose(outf);
		fclose(inf);
		return 1;
	}
	rewind(inf);

	if((scene = HRead3dsScene(inf, 0, size)) == 0) {
        ivp_message( "Failed to load %s\n", infn);
		if(outf!=stdout) fclose(outf);
		fclose(inf);
		return 1;
	}
	fclose(inf);

    // At this point we have all object's data loaded into memory.
    // H3dsScene * scene  -> object data

	// Prepare to collect all vertices to one big array

	int verts=0;
	for(n=0; n<scene->meshobjs; n++) {
		int v=scene->meshobjlist[n].verts;
		if(scene->meshobjlist[n].maps > v) {
			ivp_message( "%-14s more maps than vertices, quitting!\n",
					scene->meshobjlist[n].name);
			if(outf!=stdout) fclose(outf);
			return 1;
		}
		// Get the total number of vertices in all objects
		verts+=v;
	}

	H3dsMapVert * vrtmap = (H3dsMapVert *) p_malloc(verts*sizeof(H3dsMapVert));
	if(!vrtmap) {
		ivp_message( "Failed to allocate mem for vertice array\n");
		HFree3dsScene(scene);
		fclose(outf);
		return 1;
	}
	memset(vrtmap, 0, verts*sizeof(H3dsMapVert));

    // Do it! Collect them

	CollectVertsAndMaps(scene, vrtmap);

	// Adjust the face indexes to the new vertice array
	AdjustFaceIndexes(scene);

	if(!(flags & NORMDUP)) {
		if(flags & VERBOSE)
			ivp_message( "Removing duplicated vertices ");
		n = RemoveDupVerts(scene, vrtmap, verts);
		if(flags & VERBOSE)
			ivp_message( "\nRemoved %d duplicated vertices\n", verts-n);
		verts=n;
	}

#if 0
    int x;
    for (x=0; x<(int)verts; x++) {
	H3dsMapVert *vm = &vrtmap[x];
	IVP_U_Point *point = new IVP_U_Point();
	point->k[0] = vm->x;
	point->k[1] = vm->y;
	point->k[2] = vm->z;
	concave_polyhedron->points.add(point);
    }

    H3dsMeshObj * mo = &scene->meshobjlist[0];
    for (x=0; x<(int)mo->faces; ) {
        H3dsFace * fa = &mo->facelist[x++];
	IVP_Concave_Polyhedron_Face *face = new IVP_Concave_Polyhedron_Face();
	face->add_offset(fa->p0);
	face->add_offset(fa->p1);
	face->add_offset(fa->p2);
	concave_polyhedron->faces.add(face);
    }

//    repair_geometry(concave_polyhedron);
#endif

    //IVP_U_BigVector<IVP_Compact_Ledge> ledges;
    IVP_U_Vector<IVP_U_Point> polygon;

    int i;
    for (i=0; i<scene->meshobjs; i++) {
	H3dsMeshObj * mo = &scene->meshobjlist[i];
	int x;
	for (x=0; x<(int)mo->faces; ) {

	    H3dsFace * fa = &mo->facelist[x++];

	    IVP_U_Point *p0 = new IVP_U_Point();
	    p0->k[0] = vrtmap[fa->p0].x;
	    p0->k[1] = vrtmap[fa->p0].y;
	    p0->k[2] = vrtmap[fa->p0].z;
	    polygon.add(p0);

	    IVP_U_Point *p1 = new IVP_U_Point();
	    p1->k[0] = vrtmap[fa->p1].x;
	    p1->k[1] = vrtmap[fa->p1].y;
	    p1->k[2] = vrtmap[fa->p1].z;
	    polygon.add(p1);

	    IVP_U_Point *p2 = new IVP_U_Point();
	    p2->k[0] = vrtmap[fa->p2].x;
	    p2->k[1] = vrtmap[fa->p2].y;
	    p2->k[2] = vrtmap[fa->p2].z;
	    polygon.add(p2);

	    IVP_Compact_Ledge *compact_ledge = IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge(&polygon);
	    if ( compact_ledge ) {
		ledges->add(compact_ledge);
	    }
	    int y;
	    for (y=0; y<polygon.len(); y++) {
		delete(polygon.element_at(y));
	    }
	    polygon.clear();
	}
    }

#endif

    return(1);
}



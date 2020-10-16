// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// =====================================================================================

// This file contains software technology by Id Software, Inc. ("Id Technology").
// This data has been taken from the file 'bspfile.h' which is part of the
// archive 'qutils.zip'. The archive is freely available at ID's own ftp server
// 'ftp.id-software.com'.
//
// Ipion, June 8th 1999

#include <ivp_physics.hxx>
#ifdef LINUX
#	include <string.h>
#endif
#include <ivp_compact_ledge.hxx>

#include <ivp_halfspacesoup.hxx>
#include <ivp_surbuild_q12.hxx>
#include <ivp_surbuild_polygon_convex.hxx>
#include <ivp_surbuild_halfspacesoup.hxx>
#include <ivp_surbuild_ledge_soup.hxx>


// hack :)
class IVP_q12_int {
public:    
    int val;

    IVP_q12_int(int x) { this->val = x; }
    ~IVP_q12_int() { ; }
};
// hack :)


// ID software technology: start

//=============================================================================

typedef unsigned char byte;

#ifdef _SGI_SOURCE
#define	__BIG_ENDIAN__
#endif

#ifdef __BIG_ENDIAN__

short   LittleShort (short l)
{
    byte    b1,b2;

    b1 = l&255;
    b2 = (l>>8)&255;

    return (b1<<8) + b2;
}

short   BigShort (short l)
{
    return l;
}


int    LittleLong (int l)
{
    byte    b1,b2,b3,b4;

    b1 = l&255;
    b2 = (l>>8)&255;
    b3 = (l>>16)&255;
    b4 = (l>>24)&255;

    return ((int)b1<<24) + ((int)b2<<16) + ((int)b3<<8) + b4;
}

int    BigLong (int l)
{
    return l;
}


IVP_FLOAT	LittleFloat (IVP_FLOAT l)
{
    union {byte b[4]; IVP_FLOAT f;} in, out;
	
    in.f = l;
    out.b[0] = in.b[3];
    out.b[1] = in.b[2];
    out.b[2] = in.b[1];
    out.b[3] = in.b[0];
	
    return out.f;
}

IVP_FLOAT	BigFloat (IVP_FLOAT l)
{
    return l;
}


#else


short   BigShort (short l)
{
    byte    b1,b2;

    b1 = l&255;
    b2 = (l>>8)&255;

    return (b1<<8) + b2;
}

short   LittleShort (short l)
{
    return l;
}


int    BigLong (int l)
{
    byte    b1,b2,b3,b4;

    b1 = l&255;
    b2 = (l>>8)&255;
    b3 = (l>>16)&255;
    b4 = (l>>24)&255;

    return ((int)b1<<24) + ((int)b2<<16) + ((int)b3<<8) + b4;
}

int    LittleLong (int l)
{
    return l;
}

IVP_FLOAT	BigFloat (IVP_FLOAT l)
{
    union {byte b[4]; IVP_FLOAT f;} in, out;
	
    in.f = l;
    out.b[0] = in.b[3];
    out.b[1] = in.b[2];
    out.b[2] = in.b[1];
    out.b[3] = in.b[0];
	
    return out.f;
}

IVP_FLOAT	LittleFloat (IVP_FLOAT l)
{
    return l;
}


#endif

// upper design bounds

#define BSPVERSION	30

#define	LUMP_PLANES	1
#define	LUMP_NODES	5
#define	LUMP_CLIPNODES	9
#define	LUMP_MODELS	14


#if defined(SUN4) || defined(SUN) || (defined(__MWERKS__) && defined(__POWERPC__)) || defined(GEKKO)
void IVP_SurfaceBuilder_Q12::swap_bsp_data()
{
    int      i, j;
    dmodel_t *d;

	
    // models	
    for (i=0 ; i<this->n_models ; i++) {
	d = &this->dmodels[i];

	for (j=0 ; j<MAX_MAP_HULLS ; j++) {
	    d->headnode[j] = BigLong (d->headnode[j]);
	}

	d->visleafs = BigLong (d->visleafs);
	d->firstface = BigLong (d->firstface);
	d->numfaces = BigLong (d->numfaces);
		
	for (j=0 ; j<3 ; j++) {
	    d->mins[j] = BigFloat(d->mins[j]);
	    d->maxs[j] = BigFloat(d->maxs[j]);
	    d->origin[j] = BigFloat(d->origin[j]);
	}
    }

    // planes
    for (i=0 ; i<this->n_planes ; i++) {
	for (j=0 ; j<3 ; j++) {
	    this->dplanes[i].normal[j] = BigFloat (this->dplanes[i].normal[j]);
	}
	this->dplanes[i].dist = BigFloat (this->dplanes[i].dist);
	this->dplanes[i].type = BigLong (this->dplanes[i].type);
    }

    // nodes
    for (i=0 ; i<this->n_nodes ; i++) {
	this->dnodes[i].planenum = BigLong (this->dnodes[i].planenum);
	for (j=0 ; j<3 ; j++) {
	    this->dnodes[i].mins[j] = BigShort (this->dnodes[i].mins[j]);
	    this->dnodes[i].maxs[j] = BigShort (this->dnodes[i].maxs[j]);
	}
	this->dnodes[i].children[0] = BigShort (this->dnodes[i].children[0]);
	this->dnodes[i].children[1] = BigShort (this->dnodes[i].children[1]);
	this->dnodes[i].firstface = BigShort (this->dnodes[i].firstface);
	this->dnodes[i].numfaces = BigShort (this->dnodes[i].numfaces);
    }

    // clipnodes
    for (i=0 ; i<this->n_clipnodes ; i++) {
	this->dclipnodes[i].planenum = BigLong (this->dclipnodes[i].planenum);
	this->dclipnodes[i].children[0] = BigShort (this->dclipnodes[i].children[0]);
	this->dclipnodes[i].children[1] = BigShort (this->dclipnodes[i].children[1]);
    }

    return;
}
#else
void IVP_SurfaceBuilder_Q12::swap_bsp_data()
{
    int      i, j;
    dmodel_t *d;

	
    // models	
    for (i=0 ; i<this->n_models ; i++) {
	d = &this->dmodels[i];

	for (j=0 ; j<MAX_MAP_HULLS ; j++) {
	    d->headnode[j] = LittleLong (d->headnode[j]);
	}

	d->visleafs = LittleLong (d->visleafs);
	d->firstface = LittleLong (d->firstface);
	d->numfaces = LittleLong (d->numfaces);
		
	for (j=0 ; j<3 ; j++) {
	    d->mins[j] = LittleFloat(d->mins[j]);
	    d->maxs[j] = LittleFloat(d->maxs[j]);
	    d->origin[j] = LittleFloat(d->origin[j]);
	}
    }

    // planes
    for (i=0 ; i<this->n_planes ; i++) {
	for (j=0 ; j<3 ; j++) {
	    this->dplanes[i].normal[j] = LittleFloat (this->dplanes[i].normal[j]);
	}
	this->dplanes[i].dist = LittleFloat (this->dplanes[i].dist);
	this->dplanes[i].type = LittleLong (this->dplanes[i].type);
    }

    // nodes
    for (i=0 ; i<this->n_nodes ; i++) {
	this->dnodes[i].planenum = LittleLong (this->dnodes[i].planenum);
	for (j=0 ; j<3 ; j++) {
	    this->dnodes[i].mins[j] = LittleShort (this->dnodes[i].mins[j]);
	    this->dnodes[i].maxs[j] = LittleShort (this->dnodes[i].maxs[j]);
	}
	this->dnodes[i].children[0] = LittleShort (this->dnodes[i].children[0]);
	this->dnodes[i].children[1] = LittleShort (this->dnodes[i].children[1]);
	this->dnodes[i].firstface = LittleShort (this->dnodes[i].firstface);
	this->dnodes[i].numfaces = LittleShort (this->dnodes[i].numfaces);
    }

    // clipnodes
    for (i=0 ; i<this->n_clipnodes ; i++) {
	this->dclipnodes[i].planenum = LittleLong (this->dclipnodes[i].planenum);
	this->dclipnodes[i].children[0] = LittleShort (this->dclipnodes[i].children[0]);
	this->dclipnodes[i].children[1] = LittleShort (this->dclipnodes[i].children[1]);
    }

    return;
}
#endif


int IVP_SurfaceBuilder_Q12::init_bsp_data(int lump, void **dest, int size)
{
    int length, ofs;

    length = this->header->lumps[lump].filelen;
    ofs = this->header->lumps[lump].fileofs;

    *dest = (dmodel_t *)p_calloc(length, size);
	
    memcpy(*dest, (byte *)this->header + ofs, length);

    return(length / size);
}

// ID software technology: end



// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int q12_get_length_of_file(FILE *fp)
{
    int pos = ftell(fp);
    fseek(fp, 0, SEEK_END);
    int end = ftell(fp);
    fseek(fp, pos, SEEK_SET);

    return(end);
}

int IVP_SurfaceBuilder_Q12::load_q12bsp_file(char *filename)
{

// ****
#ifndef GEKKO
// ****

    // load the bsp file
    FILE *fp = fopen(filename, "rb");
    if ( !fp ) return(0);
    int length = q12_get_length_of_file(fp);
    this->header = (dheader_t *)p_malloc(length+1);
    ((char *)this->header)[length] = 0;
    fread((char *)this->header, 1, length, fp);
    fclose(fp);

    // swap the header
    int i;
    for (i=0 ; i< (int)sizeof(dheader_t)/4 ; i++) {
#if defined(SUN4) || defined(SUN)
	((int *)header)[i] = BigLong( ((int *)header)[i]);
#else
	((int *)header)[i] = LittleLong( ((int *)header)[i]);
#endif
    }

    IVP_IF(1) {
	if ( header->version > BSPVERSION ) {
		printf("****** ERROR ******\n%s is version %i, not %i\n", filename, header->version, BSPVERSION);
	}
    }

    this->n_models       = this->init_bsp_data(LUMP_MODELS      , (void **)&this->dmodels      , sizeof(dmodel_t));
    //numvertexes     = init_bsp_data(LUMP_VERTEXES    , dvertexes    , sizeof(dvertex_t));
    this->n_planes       = this->init_bsp_data(LUMP_PLANES      , (void **)&this->dplanes      , sizeof(dplane_t));
    //numleafs        = init_bsp_data(LUMP_LEAFS       , dleafs       , sizeof(dleaf_t));
    this->n_nodes        = this->init_bsp_data(LUMP_NODES       , (void **)&this->dnodes       , sizeof(dnode_t));
    //numtexinfo      = init_bsp_data(LUMP_TEXINFO     , texinfo      , sizeof(texinfo_t));
    this->n_clipnodes    = this->init_bsp_data(LUMP_CLIPNODES   , (void **)&this->dclipnodes   , sizeof(dclipnode_t));
    //numfaces        = this->init_bsp_data(LUMP_FACES       , dfaces       , sizeof(dface_t));
    //nummarksurfaces = this->init_bsp_data(LUMP_MARKSURFACES, dmarksurfaces, sizeof(dmarksurfaces[0]));
    //numsurfedges    = this->init_bsp_data(LUMP_SURFEDGES   , dsurfedges   , sizeof(dsurfedges[0]));
    //numedges        = this->init_bsp_data(LUMP_EDGES       , dedges       , sizeof(dedge_t));

    //texdatasize     = this->init_bsp_data(LUMP_TEXTURES    , dtexdata     , 1);
    //visdatasize     = this->init_bsp_data(LUMP_VISIBILITY  , dvisdata     , 1);
    //lightdatasize   = this->init_bsp_data(LUMP_LIGHTING    , dlightdata   , 1);
    //entdatasize     = this->init_bsp_data(LUMP_ENTITIES    , dentdata     , 1);

    P_FREE(this->header);		// everything has been copied out

    // swap everything
    this->swap_bsp_data();

    IVP_IF(1) {
	printf("Number of models in bsp file: %d\n", n_models);
    }

    this->bsptree_loaded_from_disk = IVP_TRUE;

    return(n_models);

// ****
#else  // GEKKO
// ****

    return 0;
    
// ****
#endif // GEKKO
// ****

}

    
void IVP_SurfaceBuilder_Q12::init_q12bsp_from_memory(int version,
						     int n_models_in   , dmodel_t    *dmodels_in,
						     int n_planes_in   , dplane_t    *dplanes_in,
						     int n_nodes_in    , dnode_t     *dnodes_in,
						     int n_clipnodes_in, dclipnode_t *dclipnodes_in)
{
    IVP_IF(1) {
	if ( version > BSPVERSION ) {
	    printf("****** ERROR ******\nSupplied bsptree is version %i, not %i\n", version, BSPVERSION);
	}
    }

    this->n_models    = n_models_in;
    this->n_planes    = n_planes_in;
    this->n_nodes     = n_nodes_in;
    this->n_clipnodes = n_clipnodes_in;

    this->dmodels    = dmodels_in;
    this->dplanes    = dplanes_in;
    this->dnodes     = dnodes_in;
    this->dclipnodes = dclipnodes_in;

    this->bsptree_loaded_from_disk = IVP_FALSE;

    return;
}

    
void IVP_SurfaceBuilder_Q12::convert_model(int model)
{
    this->min_x = (IVP_FLOAT)(dmodels[model].mins[0]-(1.0f/this->scale)); // blow-up (1m) necessary to avoid 2-dimensional objects on the level's boundaries
    this->min_y = (IVP_FLOAT)(dmodels[model].mins[1]-(1.0f/this->scale));
    this->min_z = (IVP_FLOAT)(dmodels[model].mins[2]-(1.0f/this->scale));
    this->max_x = (IVP_FLOAT)(dmodels[model].maxs[0]+(1.0f/this->scale));
    this->max_y = (IVP_FLOAT)(dmodels[model].maxs[1]+(1.0f/this->scale));
    this->max_z = (IVP_FLOAT)(dmodels[model].maxs[2]+(1.0f/this->scale));

    this->n_solid_nodes = 0;
    this->n_converted_nodes = 0;

    this->convert_node(dmodels[model].headnode[0]);

    IVP_IF(1) {
	printf("\nBSP tree conversion statistics:\n");
	printf("  # of original solid nodes : %d\n", this->n_solid_nodes);
	printf("  # of converted nodes      : %d\n", this->n_converted_nodes);
	printf("  # of dropped nodes        : %d\n", this->n_solid_nodes-this->n_converted_nodes);
	printf("\n\n");
    }

    this->nodes.clear();
    return;
}


void IVP_SurfaceBuilder_Q12::convert_node(int64_t node)
{
    //lwss - x64 fixes
    //this->nodes.add((int *)node);
    this->nodes.add((int64_t *)node);
    //lwss end

    if ( dnodes[node].children[0] > 0 ) {
	this->convert_node(dnodes[node].children[0]);
    }else if ( dnodes[node].children[0] == -1 ) {
	this->convert_solid_node();
    }
    
    if ( dnodes[node].children[1] > 0 ) {
	this->convert_node(dnodes[node].children[1]);
    }else if ( dnodes[node].children[1] == -1 ) {
	this->convert_solid_node();
    }

    this->nodes.remove_at(this->nodes.len()-1);
    return;
}


#if 0 /* not used right now, but don't delete!*/
void IVP_SurfaceBuilder_Q12::convert_clipnode(int clipnode)
{
    this->nodes.add((int *)clipnode);

    if ( dclipnodes[clipnode].children[0] > 0 ) {
	this->convert_clipnode(dclipnodes[clipnode].children[0]);
    }
    else if ( dclipnodes[clipnode].children[0] == -2 ) {
	this->convert_solid_clipnode();
    }
    if ( dclipnodes[clipnode].children[1] > 0 ) {
	this->convert_clipnode(dclipnodes[clipnode].children[1]);
    }
    else if ( dclipnodes[clipnode].children[1] == -2 ) {
	this->convert_solid_clipnode();
    }

    this->nodes.delete_at(this->nodes.len()-1);
    return;
}
#endif


// -------------------------------------------------------------------------
// convert_solid_node
// ==================
//
// extract a solid node from bsp tree and convert it into a polygon template
// -------------------------------------------------------------------------
void IVP_SurfaceBuilder_Q12::convert_solid_node()
{
    this->n_solid_nodes++;

    // --------------------------------
    // extract all planes from bsp tree
    // --------------------------------
    this->nodes_to_planes();


    // ------------------------------------------------
    // insert boundary planes to avoid infinite volumes
    // ------------------------------------------------
    this->create_and_insert_plane( 1.0f,  0.0f,  0.0f,  this->min_x);
    this->create_and_insert_plane( 0.0f,  1.0f,  0.0f,  this->min_y);
    this->create_and_insert_plane( 0.0f,  0.0f,  1.0f,  this->min_z);
    this->create_and_insert_plane(-1.0f,  0.0f,  0.0f, -this->max_x);
    this->create_and_insert_plane( 0.0f, -1.0f,  0.0f, -this->max_y);
    this->create_and_insert_plane( 0.0f,  0.0f, -1.0f, -this->max_z);


    // ------------------------------------------
    // build convex hull from intersecting planes
    // ------------------------------------------
    IVP_Compact_Ledge *new_ledge;
    new_ledge = IVP_SurfaceBuilder_Halfspacesoup::convert_halfspacesoup_to_compact_ledge(this->halfspaces, this->pointmerge_threshold);
    if ( new_ledge ) {
	this->ledges->add(new_ledge);
	this->n_converted_nodes++;
    }
    this->cleanup();    
    this->halfspaces = new IVP_Halfspacesoup();

    return;
}


#if 0 /* not used right now, but don't delete!*/
// ----------------------------------------------------------------------------
// convert_solid_clipnode
// ======================
//
// extract a solid clipnode from bsp tree and convert it into a physical object
// ----------------------------------------------------------------------------
void IVP_SurfaceBuilder_Q12::convert_solid_clipnode()
{

    // [...]

    // --------------------------------
    // extract all planes from bsp tree
    // --------------------------------
    this->clipnodes_to_planes();

    // [...]

    return;
}
#endif


void IVP_SurfaceBuilder_Q12::nodes_to_planes()
{
    int i;
    
    for (i=0; i<this->nodes.len(); i++) {
    //lwss - x64 fixes
	//dplane_t *bsp_plane = &dplanes[dnodes[(int)nodes.element_at(i)].planenum];
	dplane_t *bsp_plane = &dplanes[dnodes[(intptr_t)nodes.element_at(i)].planenum];
	if ( i == this->nodes.len()-1 ) {
	    //if ( dnodes[(int)this->nodes.element_at(i)].children[0] == -1 ) {
	    if ( dnodes[(intptr_t)this->nodes.element_at(i)].children[0] == -1 ) {
		create_and_insert_plane( bsp_plane->normal[0],  bsp_plane->normal[1],  bsp_plane->normal[2], bsp_plane->dist);
	    }
	    else {
		create_and_insert_plane(-bsp_plane->normal[0], -bsp_plane->normal[1], -bsp_plane->normal[2], -bsp_plane->dist);
	    }
	}
	else {
	    //if ( dnodes[(int)this->nodes.element_at(i)].children[0] == (int)this->nodes.element_at(i+1) ) {
	    if ( dnodes[(intptr_t)this->nodes.element_at(i)].children[0] == (intptr_t)this->nodes.element_at(i+1) ) {
		create_and_insert_plane( bsp_plane->normal[0],  bsp_plane->normal[1],  bsp_plane->normal[2], bsp_plane->dist);
	    }
	    else {
		create_and_insert_plane(-bsp_plane->normal[0], -bsp_plane->normal[1], -bsp_plane->normal[2], -bsp_plane->dist);
	    }
	}
    }

    return;
}


#if 0 /* not used right now, but don't delete!*/
void IVP_SurfaceBuilder_Q12::clipnodes_to_planes()
{
    int i;
    
    for (i=0; i<this->nodes.len(); i++) {
	dplane_t *bsp_plane = &dplanes[dclipnodes[(int)nodes.element_at(i)].planenum];
	if ( i == this->nodes.len()-1 ) {
	    if ( dclipnodes[(int)this->nodes.element_at(i)].children[0] == -1 ) {
		create_and_insert_plane( bsp_plane->normal[0],  bsp_plane->normal[1],  bsp_plane->normal[2], bsp_plane->dist);
	    }
	    else {
		create_and_insert_plane(-bsp_plane->normal[0], -bsp_plane->normal[1], -bsp_plane->normal[2], -bsp_plane->dist);
	    }
	}
	else {
	    if ( dclipnodes[(int)this->nodes.element_at(i)].children[0] == (int)this->nodes.element_at(i+1) ) {
		create_and_insert_plane( bsp_plane->normal[0],  bsp_plane->normal[1],  bsp_plane->normal[2], bsp_plane->dist);
	    }
	    else {
		create_and_insert_plane(-bsp_plane->normal[0], -bsp_plane->normal[1], -bsp_plane->normal[2], -bsp_plane->dist);
	    }
	}
    }

    return;
}
#endif


// ------------------------------------------------------------------------
// create_and_insert_plane
// =======================
//
// create a plane and insert it into the planelist
// ------------------------------------------------------------------------
void IVP_SurfaceBuilder_Q12::create_and_insert_plane(IVP_FLOAT nx, IVP_FLOAT ny, IVP_FLOAT nz, IVP_FLOAT dist)
{
#ifdef CREATE_AND_INSERT_PLANE_DEBUG    
    printf("New ivp plane         : {x:%f, y:%f, z:%f}, %f\n", nx, -nz, ny, -dist);
    printf("  (original bsp plane : {x:%f, y:%f, z:%f}, %f)\n", nx, ny, nz, dist);
#endif    

    IVP_U_Hesse plane;
    plane.set(nx, -nz, ny);
    plane.hesse_val = -(dist+this->shrink_value) * this->scale; // scale to a basis of 1 unit = 1 meter

    halfspaces->add_halfspace( &plane );
    return;
}





void IVP_SurfaceBuilder_Q12::cleanup()
{
    // --------
    // clean up
    // --------
    P_DELETE( this->halfspaces );
    return;
}


void IVP_SurfaceBuilder_Q12::unload_q12bsp()
{
    if ( this->bsptree_loaded_from_disk ) {
	P_FREE(this->dmodels);
	P_FREE(this->dplanes);
	P_FREE(this->dnodes);
	P_FREE(this->dclipnodes);
	this->bsptree_loaded_from_disk = IVP_FALSE;
    }
    return;
}


IVP_SurfaceBuilder_Q12::IVP_SurfaceBuilder_Q12()
{
    this->dmodels = NULL;
    this->dplanes = NULL;
    this->dnodes = NULL;
    this->dclipnodes = NULL;
    this->scale = 1.0f;
    this->shrink_value = 0.0f;
    this->bsptree_loaded_from_disk = IVP_FALSE;
    this->halfspaces = new IVP_Halfspacesoup();

    this->zero = new IVP_q12_int(0);
    this->one = new IVP_q12_int(1);
    return;
}


IVP_SurfaceBuilder_Q12::~IVP_SurfaceBuilder_Q12()
{
    this->unload_q12bsp();
    P_DELETE(this->zero);
    P_DELETE(this->one);
    P_DELETE(this->halfspaces);
    return;
}


void IVP_SurfaceBuilder_Q12::convert_q12bsp_model_to_compact_ledges(int model,
								    IVP_DOUBLE scaling_factor,
								    IVP_DOUBLE shrink_value_in,
 								    IVP_FLOAT pointmerge_threshold_in,
								    IVP_U_Vector<IVP_Compact_Ledge> *ledges_out)
{
    if ( !this->dmodels ) return; // something went wrong while loading/initializing the bsp data. aborting...

    this->scale = (IVP_FLOAT)scaling_factor;
    this->ledges = ledges_out;
    this->shrink_value = (IVP_FLOAT)((1.0f/scaling_factor)*shrink_value_in);
    this->pointmerge_threshold = pointmerge_threshold_in;

    this->convert_model(model);

    return;
}


IVP_Compact_Surface *IVP_SurfaceBuilder_Q12::convert_q12bsp_model_to_single_compact_surface(int model,
											    IVP_DOUBLE scaling_factor,
											    IVP_DOUBLE shrink_value_in,
											    IVP_FLOAT pointmerge_threshold_in)
{
    int i;
    IVP_U_Vector<IVP_Compact_Ledge> ledges_local;

    IVP_SurfaceBuilder_Q12::convert_q12bsp_model_to_compact_ledges(model, scaling_factor, shrink_value_in, pointmerge_threshold_in, &ledges_local);

    IVP_SurfaceBuilder_Ledge_Soup ledge_soup;
    for (i=0; i<ledges_local.len(); i++) {
	ledge_soup.insert_ledge(ledges_local.element_at(i));
    }
    IVP_Compact_Surface *surface = ledge_soup.compile();

    return(surface);
}



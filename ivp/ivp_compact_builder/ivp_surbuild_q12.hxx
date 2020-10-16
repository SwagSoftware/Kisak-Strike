// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_surbuild_q12.hxx
 *  Description:    This file provides you with a builder class for easy
 *		    conversion of Quake1/2 compliant bsptree level files into a
 *		    (possibly concave) compact surface for further processing
 *		    by the Ipion engine.
 *  Additional
 *  Copyright
 *  Notices:	    certain code segments Copyright (C) Id Software, Inc.
 *  Classes:	    IVP_SurfaceBuilder_Q12
 ********************************************************************************/

#ifndef _IVP_SURBUILD_Q12_INCLUDED
#define _IVP_SURBUILD_Q12_INCLUDED

// =====================================================================================

// This header file contains software technology by Id Software, Inc. ("Id Technology").
// This data has been taken from the file 'bspfile.h' which is part of the
// archive 'qutils.zip'. The archive is freely available at ID's own ftp server
// 'ftp.id-software.com'.
//
// Ipion Software GmbH, June 8th 1999

// ID software technology: start

#define	MAX_MAP_HULLS 4
#define	HEADER_LUMPS  15
class IVP_Compact_Surface;

typedef struct
{
    IVP_FLOAT		mins[3], maxs[3];
    IVP_FLOAT		origin[3];
    //lwss - x64 fixes
    //int			headnode[MAX_MAP_HULLS];
    int64_t 		headnode[MAX_MAP_HULLS];
    //lwss end
    int			visleafs;		// not including the solid leaf 0
    int			firstface, numfaces;
} dmodel_t;

typedef struct
{
    int		fileofs, filelen;
} lump_t;

typedef struct
{
    int			version;	
    lump_t		lumps[HEADER_LUMPS];
} dheader_t;

typedef struct
{
    IVP_FLOAT	normal[3];
    IVP_FLOAT	dist;
    int		type;		// PLANE_X - PLANE_ANYZ ?remove? trivial to regenerate
} dplane_t;

typedef struct
{
    int			planenum;
    short		children[2];	// negative numbers are -(leafs+1), not nodes
    short		mins[3];		// for sphere culling
    short		maxs[3];
    unsigned short	firstface;
    unsigned short	numfaces;	// counting both sides
} dnode_t;

typedef struct
{
    int			planenum;
    short		children[2];	// negative numbers are contents
} dclipnode_t;

// ID software technology: end

// =====================================================================================


class IVP_q12_int;
class IVP_Halfspacesoup;


/********************************************************************************
 *  Class:	    IVP_SurfaceBuilder_Q12
 *  Description:    This builder class allows for an easy import of standard
 *		    Quake 1/2 bsp levels into the Ipion engine.
 *		    You can either tell this class to load the level from disk
 *		    or to use a bsptree already present in memory.
 *		    You can then convert any of the level's models into a set of
 *		    compact ledges or into an already compiled (concave) compact
 *		    surface.
 *******************************************************************************/

class IVP_SurfaceBuilder_Q12 {
private:
    dheader_t   *header;

    int          n_models;
    dmodel_t    *dmodels;

    int          n_planes;
    dplane_t    *dplanes;

    int		 n_nodes;
    dnode_t     *dnodes;

    int		 n_clipnodes;
    dclipnode_t *dclipnodes;

    void swap_bsp_data();
    int init_bsp_data(int lump, void **dest, int size);
    
    IVP_q12_int *zero, *one;

    IVP_BOOL bsptree_loaded_from_disk;
    
    IVP_FLOAT min_x, min_y, min_z;
    IVP_FLOAT max_x, max_y, max_z;
    IVP_FLOAT shrink_value;
    IVP_FLOAT scale;
    IVP_FLOAT pointmerge_threshold;

    int n_solid_nodes;          // statistical data
    int n_converted_nodes;      // statistical data

    IVP_U_Vector<IVP_Compact_Ledge> *ledges;

    //lwss - x64 fixes
    //IVP_U_Vector<int>         nodes;
    IVP_U_Vector<int64_t>         nodes;
    //lwss end
    IVP_Halfspacesoup			  *halfspaces;
    
    void cleanup();
    
    void create_and_insert_plane(IVP_FLOAT nx, IVP_FLOAT ny, IVP_FLOAT nz, IVP_FLOAT dist);

    //void clipnodes_to_planes();          // not used right now, but don't delete!
    //void convert_clipnode(int clipnode); // not used right now, but don't delete!
    //void convert_solid_clipnode();       // not used right now, but don't delete!

    void nodes_to_planes();
    void convert_node(int64_t node);
    void convert_solid_node();

    void convert_model(int model);

public:

    /******************************************************************************
     *  Method:		convert_q12bsp_model_to_compact_ledges
     *  Description:    This method will convert the supplied bsptree model into a
     *			set of (convex) compact ledges.
     *	Input:		<model>                number of the bsptree's model you
     *			                       want to have converted
     *			<scaling_factor>       factor by which the contents of the
     *			                       bsptree will be scaled;
     *			<shrink_value>         all convex subparts of the level
     *			                       will be shrunk by this value.
     *			                       (unit: m)
     *			                       [should be 0]
     *			<pointmerge_threshold> all points closer than this value
     *			                       will be merged
     *			<ledges_out>           address of vector to fill with
     *			                       the resulting compact ledges
     *	Note:		The IPION engine uses meters as basic scale (so to convert
     *			e.g. a bsptree using inches as basic scale you would 
     *			provide a factor of 0.0254f).
     *****************************************************************************/
    void convert_q12bsp_model_to_compact_ledges(int model,
						IVP_DOUBLE scaling_factor,
						IVP_DOUBLE shrink_value,
						IVP_FLOAT pointmerge_threshold,
						IVP_U_Vector<IVP_Compact_Ledge> *ledges_out);
    
    /******************************************************************************
     *  Method:		convert_q12bsp_model_to_single_compact_surface
     *  Description:    This method will compile a (concave) compact surface from
     *			the supplied bsptree model.
     *	Input:		<model>                number of the bsptree's model you
     *			                       want to have converted
     *			<scaling_factor>       factor by which the contents of the
     *			                       bsptree will be scaled;
     *			<shrink_value>         all convex subparts of the level
     *			                       will be shrunk by this value.
     *			                       (unit: m)
     *			                       [should be 0]
     *			<pointmerge_threshold> all points closer than this value
     *			                       will be merged
     *	Output:		Pointer to IVP_Compact_Surface structure
     *	Note:		The IPION engine uses meters as basic scale (so to convert
     *			e.g. a bsptree using inches as basic scale you would 
     *			provide a factor of 0.0254f).
     *****************************************************************************/
    IVP_Compact_Surface *convert_q12bsp_model_to_single_compact_surface(int model,
									IVP_DOUBLE scaling_factor,
									IVP_DOUBLE shrink_value,
									IVP_FLOAT pointmerge_threshold);
    
    /********************************************************************************
     *
     * (UN)LOADING BSPTREES
     *
     * we provide two different ways to convert a bsptree:
     *
     * - 'load_q12bsp_file()'        : will load the supplied bspfile from disk and
     *                                 import its data into the physical world
     * - 'init_q12bsp_from_memory()' : will import the data from an already loaded
     *                                 and initialized bsptree in memory
     * - 'unload_q12bsp()'           : frees all allocated memory
     *
     ********************************************************************************/

    /******************************************************************************
     *  Method:		load_q12bsp_file
     *  Description:    This method will load the supplied bsptree from disk.
     *	Input:		<filename> the level's path & filname
     *	Output:		Number of models in bsptree
     *****************************************************************************/
    int load_q12bsp_file(char *filename);

    /******************************************************************************
     *  Method:		init_q12bsp_from_memory
     *  Description:    This method will use a bsptree already present in memory
     *	Input:		<version>        the bsptree's version number
     *			<n_models_in>    number of models
     *			<dmodels_in>     address of model array in memory
     *			<n_planes_in>    number of planes
     *			<dplanes_in>     address of planes array in memory
     *			<n_nodes_in>     number of nodes
     *			<dnodes_in>      address of nodes array in memory
     *			<n_clipnodes_in> number of clipnodes
     *			<dclipnodes_in>  address of clipnodes array in memory
     *****************************************************************************/
    void init_q12bsp_from_memory(int version,
				 int n_models_in   , dmodel_t    *dmodels_in,
				 int n_planes_in   , dplane_t    *dplanes_in,
				 int n_nodes_in    , dnode_t     *dnodes_in,
				 int n_clipnodes_in, dclipnode_t *dclipnodes_in);

    /******************************************************************************
     *  Method:		unload_q12bsp
     *  Description:    This method will free all memory used by this builder 
     *			class. It will not free external bsptree memory (i.e.
     *			memory passed to the class by the "init_q12bsp_from_memory"
     *			method)!
     *****************************************************************************/
   void unload_q12bsp();

    IVP_SurfaceBuilder_Q12();
    ~IVP_SurfaceBuilder_Q12();

};

#endif

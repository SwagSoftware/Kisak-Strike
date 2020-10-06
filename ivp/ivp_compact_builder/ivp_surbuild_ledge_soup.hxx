// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_surbuild_ledge_soup.hxx
 *  Description:    This file provides you with a builder class for assembling
 *		    different (convex) compact ledges into one (possibly concave)
 *		    compact surface which you can then use to create a physical
 *		    object.
 *  Classes:	    IVP_SurfaceBuilder_Ledge_Soup
 ********************************************************************************/

#ifndef IVP_SURBUILD_LEDGE_SOUP_INCLUDED
#define IVP_SURBUILD_LEDGE_SOUP_INCLUDED

class IVV_Cluster_Min_Hash;
class IVP_point_hash;
class IVP_Template_Ledge_Polygon_Soup;
class IVP_Compact_Ledge;
class IVP_Compact_Surface;
class IVP_Template_Polygon;
class IVV_Sphere;
class IVP_Compact_Poly_Point;
class IVP_Template_Surbuild_Ledgesoup;

/********************************************************************************
 *  Class:	    IVP_SurfaceBuilder_Ledge_Soup
 *  Description:    Ipion's ledgesoup builder takes one or more (convex) compact
 *		    ledges as input, combines them and builds a (possibly
 *		    concave) compact surface. You can then use this compact
 *		    surface to create a physical object.
 *  Note:	    Algorithm is fast but still O(n*sqrt(n));
 *******************************************************************************/
class IVP_SurfaceBuilder_Ledge_Soup {
    friend class P_Listener_X11; // internally needed by graph_lib
private:
    // internal methods
    IVP_Compact_Surface *compact_surface;
    int number_of_terminal_spheres;
    int number_of_nodes;
    IVP_DOUBLE smallest_radius;
    int size_of_tree_in_bytes; // benchmarking
    struct IVV_Sphere_Cluster *spheres_cluster;
    IVP_U_Vector<IVP_Compact_Ledge> c_ledge_vec;  // including inner recursive compact ledges

    IVP_U_Vector<IVV_Sphere> rec_spheres;         // only spheres with inner recursive ledges
    IVP_U_Vector<IVV_Sphere> terminal_spheres;

    IVP_U_Float_Point extents_min;
    IVP_U_Float_Point extents_max;
    int longest_axis;
    int number_of_unclustered_spheres;
    IVV_Cluster_Min_Hash *interval_minhash;
    IVP_U_Vector<IVV_Sphere> overlapping_spheres;
    IVP_U_Vector<IVV_Sphere> built_spheres; // temporary; contains newly built minimal spheres during each pass

    class IVP_Template_Surbuild_LedgeSoup *parameters;

    // compact surface creation
    IVP_Compact_Ledge *first_compact_ledge;
    IVP_Compact_Poly_Point *first_poly_point;
    int	    n_poly_points_allocated;

    class IVP_I_FPoint_VHash *point_hash;

    class IVP_Compact_Ledgetree_Node *ledgetree_work;

    char *clt_highmem; // debugging
    char *clt_lowmem; // debugging
protected:
    IVP_U_Vector<IVV_Sphere> all_spheres;
protected:


    void ledges_to_spheres(); // soon to be outdated!
    void ledges_to_boxes_and_spheres();
    void generate_interval_minhash(float blowup_factor);
    void combine_spheres_in_vector(IVV_Cluster_Min_Hash *cluster_min_hash);
    void remove_all_connected_combinations_from_hash(IVV_Cluster_Min_Hash *cluster_min_hash, int sphere_1, int sphere_2);
    void remove_all_further_spherecombinations_from_hash(IVV_Cluster_Min_Hash *cluster_min_hash, int spherenumber);
    IVV_Sphere *build_minimal_sphere(IVV_Sphere *sphere1, IVV_Sphere *sphere2);
    void replace_childspheres_in_spherelist_with_motherspheres();
    void calculate_boundingbox(IVP_U_Vector<IVV_Sphere> *terminals, IVP_U_Float_Point *ext_min, IVP_U_Float_Point *ext_max);

    void recalc_min_hash_with_fixed_max_radius(IVV_Cluster_Min_Hash *cluster_min_hash, IVP_DOUBLE fixed_max_radius);
    void combine_sphere_and_calc_minimal_radius(int base_sphere, IVV_Cluster_Min_Hash *cluster_min_hash, IVP_DOUBLE fixed_max_radius);
    int recompile_point_indizes_of_compact_ledge(IVP_Compact_Ledge *ledge_source, char *dest);
    IVP_Compact_Ledgetree_Node *build_ledgetree(IVV_Sphere *node,
						IVV_Sphere *predecessor_node,
						IVP_Compact_Ledgetree_Node *predecessor_ltnode);
    IVP_Compact_Ledgetree_Node *build_ledgetree(IVV_Sphere *node);
    void ledgetree_debug_output(const IVP_Compact_Ledgetree_Node *node) const; // debugging
    void ledgetree_array_debug_output();

    void insert_compact_ledges();
    void insert_radius_in_compact_surface();
    void cluster_spheres_bottomup(IVP_DOUBLE threshold_increase);
    void cluster_spheres_topdown_mediancut(IVP_DOUBLE threshold_increase);
    IVV_Sphere *cluster_spheres_topdown_mediancut_recursively(IVP_U_Vector<IVV_Sphere> *terminals);

    void cleanup();

    // finally call both functions to fill ledge
    IVP_Compact_Surface *allocate_compact_surface();
    IVP_RETURN_TYPE create_compact_ledgetree();

    IVP_Compact_Ledge *insert_ledge(IVP_Template_Ledge_Polygon_Soup *ledge_templ);// add triangle, old style, will be deleted in the future
#if defined(LINUX) || defined(SUN) || ( defined(__MWERKS__) && defined(__POWERPC__) ) || defined(GEKKO)
    static void convert_ledges_to_templates(IVP_U_BigVector<IVP_Compact_Ledge> &ledges,	// old function, needed for ivp_graphlib
					    IVP_U_Vector<IVP_Template_Polygon> *templates_out);
#endif    
    void add_ledge_tree_to_convex_hull( class IVP_Compact_Recursive &, class IVV_Sphere *node);
    void build_root_convex_hull();
  
public:

    /******************************************************************************
     *  Method:		insert_ledge
     *  Description:    Use this method to add additional ledges (i.e. convex
     *			subparts of your object) to the ledgesoup builder
     *	Input:		<ledge> new ledge to add to the soup
     *	Output:		Pointer to the original (supplied) IVP_Compact_Ledge
     *			(compatibility reason)
     *****************************************************************************/
    void insert_ledge(IVP_Compact_Ledge *ledge);

    /******************************************************************************
     *  Method:		compile
     *  Description:    This method will actually build the new (possibly concave)
     *			compact surface out of the (convex) ledges
     *  build_convex_hull if set to true, the algorithm generates an additional
     *			convex hull around the object. For moveable objects this
     *			hull can increase performance to the same level as convex
     *			objects.
     *			Only for huge landscapes this optimization makes no sense at
     *			all, as all interesting objects are penetrating the convex hull
     *			all the time.
     *	Output:		Pointer to the new IVP_Compact_Surface
     *	Note:		Call this method only after inserting at least one ledge.
     *			Otherwise this method will return NULL!
     *	Note:		Algorithm is fast but still O(n*n);
     *			E.g. 5000 ledges take 50 seconds to compute (on PIII 600)
     *	Warning:	This method will [optionally] free all original ledges (as supplied
     *			by the "insert_ledge()" method)! DO NOT FREE THEM
     *			YOURSELF!
     *	Note:		Invoking this function a second time (without adding
     *			new ledges in the meantime) will return NULL.
     *****************************************************************************/
    IVP_Compact_Surface *compile( IVP_Template_Surbuild_LedgeSoup *templ = NULL);

    IVP_SurfaceBuilder_Ledge_Soup();
    ~IVP_SurfaceBuilder_Ledge_Soup();

};

#endif

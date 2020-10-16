// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <string.h>

//#define STATS
//#define IVP_CLUSTER_SHORTRANGE_BOTTOMUP
#define IVP_CLUSTER_SHORTRANGE_TOPDOWN
#define IVP_CLUSTER_SHORTRANGE_BALANCED
//#define IVP_CLUSTER_SHORTRANGE_BALANCE_VOLUME

#include <ivp_betterdebugmanager.hxx>
#include <ivp_template_surbuild.hxx>
#include <ivp_surbuild_ledge_soup.hxx>

#include <ivp_i_fpoint_vhash.hxx>

#include <ivp_compact_ledge.hxx>
#include <ivp_compact_surface.hxx>

#include <ivp_compact_ledge_solver.hxx>
#include <ivp_rot_inertia_solver.hxx>
#include <ivv_cluster_min_hash.hxx>
#include <ivp_surbuild_pointsoup.hxx>
#include <ivp_compact_recursive.hxx>


class IVV_Sphere {
public:
    int		number;
    IVP_U_Point	center;
    IVP_DOUBLE	radius;
    uchar	box_sizes[IVP_CLT_N_DIRECTIONS];
  
    IVP_Compact_Ledge *compact_ledge;
    IVV_Sphere	*child_1, *child_2;
    IVV_Sphere(){ P_MEM_CLEAR(this); };
};

struct IVV_Sphere_Cluster {
    ushort previous, next;
    IVV_Sphere *sphere;
};


int ivp_debug_sf_indent=0;
int ivp_debug_sf_treedepth=0;
int ivp_debug_sf_max_treedepth=0;
int ivp_debug_sf_n_nodes_with_one_terminal=0;
IVP_BOOL ivp_debug_sf_last_node_was_terminal = IVP_FALSE;

void debug_sphere_output(IVV_Sphere *sphere)
{
    ivp_debug_sf_treedepth++;
    if ( ivp_debug_sf_max_treedepth < ivp_debug_sf_treedepth ) ivp_debug_sf_max_treedepth = ivp_debug_sf_treedepth;
    
    //for (int x=0; x<ivp_debug_sf_indent; x++) {
    //	printf("  ");
    //}
    //printf("center: %.6f/%.6f/%.6f --- Radius: %.6f\n", sphere->center.k[0], sphere->center.k[1], sphere->center.k[2], sphere->radius);
    //ivp_debug_sf_indent++;
    if ( sphere->child_1 ) {
	IVP_ASSERT(sphere->child_1); // prevent nodes with one single child
	IVP_ASSERT(sphere->child_2);

	int n_terminal_children = 0;
	
	debug_sphere_output(sphere->child_1);
	if ( ivp_debug_sf_last_node_was_terminal ) n_terminal_children++;
	
	debug_sphere_output(sphere->child_2);
	if ( ivp_debug_sf_last_node_was_terminal ) n_terminal_children++;

	if ( n_terminal_children == 1 ) ivp_debug_sf_n_nodes_with_one_terminal++;
	
	ivp_debug_sf_last_node_was_terminal = IVP_FALSE;
    }
    else {
	IVP_ASSERT(!sphere->child_1); // prevent nodes with one single child
	IVP_ASSERT(!sphere->child_2);
	ivp_debug_sf_last_node_was_terminal = IVP_TRUE;
    }
    //ivp_debug_sf_indent--;
    ivp_debug_sf_treedepth--;
}



IVP_SurfaceBuilder_Ledge_Soup::IVP_SurfaceBuilder_Ledge_Soup()
{
    P_MEM_CLEAR(this);
    this->smallest_radius = 0;
    this->compact_surface = NULL;

    this->extents_min.set( 1000000.0f,  1000000.0f,  1000000.0f);
    this->extents_max.set(-1000000.0f, -1000000.0f, -1000000.0f);
    
    this->interval_minhash = NULL;

    return;
}

IVP_SurfaceBuilder_Ledge_Soup::~IVP_SurfaceBuilder_Ledge_Soup(){
}


void IVP_SurfaceBuilder_Ledge_Soup::insert_ledge(IVP_Compact_Ledge *c_ledge)
{
  if (!c_ledge){
    IVP_IFDEBUG(IVP_DM_SURBUILD_LEDGESOUP) {
      ivp_debugmanager.dprint(IVP_DM_SURBUILD_LEDGESOUP, "warning: tried to add NULL ledge in IVP_SurfaceBuilder_Ledge_Soup::insert_ledge()\n");
    }
    return;

  }
  this->c_ledge_vec.add(c_ledge);
}


IVP_Compact_Surface *IVP_SurfaceBuilder_Ledge_Soup::compile(IVP_Template_Surbuild_LedgeSoup *templ){
    IVP_Template_Surbuild_LedgeSoup t2;
    if (!templ) templ = &t2;
    this->parameters = templ;

    if ( this->c_ledge_vec.len() == 0 ) return NULL; // invalid (empty) ledge list. aborting...

    // To be called after all ledges are inserted.
    // Builds tree.
#if 0
    this->ledges_to_spheres();
#else
    this->ledges_to_boxes_and_spheres();
#endif

#ifdef IVP_CLUSTER_SHORTRANGE_BOTTOMUP    
    this->cluster_spheres_bottomup(1.1f);
#endif
#ifdef IVP_CLUSTER_SHORTRANGE_TOPDOWN    
    this->cluster_spheres_topdown_mediancut(1.0f);
#endif	

#if defined(STATS)
    if(number_of_terminal_spheres > 1){
	ivp_debug_sf_treedepth = 0;
	ivp_debug_sf_max_treedepth = 0;
	ivp_debug_sf_n_nodes_with_one_terminal = 0;
	ivp_debug_sf_last_node_was_terminal = IVP_FALSE;
	debug_sphere_output(this->spheres_cluster[this->spheres_cluster[0].next].sphere);
	printf("Tree depth: %d\n", ivp_debug_sf_max_treedepth);
	printf("# of terminals: %d\n", this->number_of_terminal_spheres);
    }
    //printf("# of nodes with single terminal child: %d\n", ivp_debug_sf_n_nodes_with_one_terminal);
#endif
    
    IVP_ASSERT(compact_surface == 0);

    if (templ->build_root_convex_hull && c_ledge_vec.len()>1){
      this->build_root_convex_hull();
    }
    
    this->allocate_compact_surface();
    this->create_compact_ledgetree();
    this->insert_radius_in_compact_surface();
    
    this->cleanup();

    IVP_Compact_Surface *res = compact_surface;

    if ( this->number_of_terminal_spheres > 1 && !parameters->link_to_input_compact_ledges ) {
	if ( parameters->merge_points == IVP_SLMP_MERGE_AND_REALLOCATE ){
		res = (IVP_Compact_Surface *)ivp_malloc_aligned( compact_surface->byte_size, 16);
		memcpy( (void *)res, this->compact_surface, compact_surface->byte_size);
		ivp_free_aligned( compact_surface );
        }
    }
    compact_surface = NULL;
    
    return res;
}

void IVP_SurfaceBuilder_Ledge_Soup::add_ledge_tree_to_convex_hull(class IVP_Compact_Recursive &cr, IVV_Sphere *node){
  if (!node) return;
  if (node->compact_ledge){
    cr.add_compact_ledge(node->compact_ledge);
    return;
  }
  if (node->child_1) add_ledge_tree_to_convex_hull(cr, node->child_1);
  if (node->child_2) add_ledge_tree_to_convex_hull(cr, node->child_2);
}


void IVP_SurfaceBuilder_Ledge_Soup::build_root_convex_hull(){
    IVV_Sphere *node = this->spheres_cluster[this->spheres_cluster[0].next].sphere;

    IVP_Compact_Recursive cr;
    add_ledge_tree_to_convex_hull(cr, node);
    
    IVP_Compact_Ledge *hull = cr.compile(); 
    c_ledge_vec.add(hull);
    node->compact_ledge = hull;
    rec_spheres.add(node);
}


void IVP_SurfaceBuilder_Ledge_Soup::cleanup()
{
    this->terminal_spheres.clear();
    int i;
    for (i=0; i<this->all_spheres.len(); i++) {
	IVV_Sphere *sp = this->all_spheres.element_at(i);
	P_DELETE(sp);
    }
    this->all_spheres.clear();
    P_FREE(this->spheres_cluster);
    return;

}


void IVP_SurfaceBuilder_Ledge_Soup::ledges_to_spheres()
{
    // -----------------------------------------------------------------------------------
    // create a minimal sphere around each ledge and save it into a fix-sized array...
    // -----------------------------------------------------------------------------------    

    this->size_of_tree_in_bytes = 0; // debugging
    IVV_Sphere *sphere;
    
    // find number of spheres to generate
    int n_ledges = this->c_ledge_vec.len();
    this->number_of_terminal_spheres = n_ledges;

    // allocate array: element #0 (dummy) always indicates the first valid element through its 'next' pointer!
    this->spheres_cluster = (struct IVV_Sphere_Cluster *)p_calloc(this->number_of_terminal_spheres+1, sizeof(IVV_Sphere_Cluster));

    this->spheres_cluster[0].next = 1;
    int n = 1;
    int ledge_cnt;
    for (ledge_cnt=0; ledge_cnt<n_ledges; ledge_cnt++) {
	IVP_Compact_Ledge *c_ledge = this->c_ledge_vec.element_at(ledge_cnt);
	IVP_ASSERT (c_ledge->get_n_triangles() > 0);

	sphere = new IVV_Sphere();
	this->all_spheres.add(sphere);
	IVP_U_Point center;
	IVP_DOUBLE radius;
	IVP_CLS.calc_geom_center_and_radius( c_ledge, &center, &radius);
	IVP_CLS.calc_radius_to_given_center( c_ledge, &center, &radius);

	if ( center.k[0]-radius < this->extents_min.k[0] ) this->extents_min.k[0] = center.k[0]-radius;
	if ( center.k[0]+radius > this->extents_max.k[0] ) this->extents_max.k[0] = center.k[0]+radius;
	if ( center.k[1]-radius < this->extents_min.k[1] ) this->extents_min.k[1] = center.k[1]-radius;
	if ( center.k[1]+radius > this->extents_max.k[1] ) this->extents_max.k[1] = center.k[1]+radius;
	if ( center.k[2]-radius < this->extents_min.k[2] ) this->extents_min.k[2] = center.k[2]-radius;
	if ( center.k[2]+radius > this->extents_max.k[2] ) this->extents_max.k[2] = center.k[2]+radius;
	
	sphere->radius = radius;
	sphere->center.set(&center);

	this->size_of_tree_in_bytes += sizeof(IVV_Sphere); // debugging
	
	sphere->compact_ledge = c_ledge;
	//	sphere->parent = NULL;
	sphere->child_1 = NULL;
	sphere->child_2 = NULL;
	sphere->number = n;

	this->spheres_cluster[n].previous = n-1;
	this->spheres_cluster[n].next     = n+1;
	this->spheres_cluster[n].sphere   = sphere;

	if ( this->smallest_radius == 0 ) {
	    this->smallest_radius = sphere->radius;
	}
	else if ( sphere->radius < this->smallest_radius ) {
	    this->smallest_radius = sphere->radius;
	}

	this->terminal_spheres.add(sphere);

	n++;
    }

    this->spheres_cluster[n-1].next = 0;
    
    //printf("Radius of smallest sphere: %f\n", this->smallest_radius);

    // @@@SF: for speedup of single-convex objects, move this into sphere-clustering!
    if ( ledge_cnt > 1 ) {
	IVP_DOUBLE ext_x = this->extents_max.k[0] - this->extents_min.k[0];
	IVP_DOUBLE ext_y = this->extents_max.k[1] - this->extents_min.k[1];
	IVP_DOUBLE ext_z = this->extents_max.k[2] - this->extents_min.k[2];
	if ( ext_x < ext_y ) {
	    if ( ext_y < ext_z ) this->longest_axis = 2;
	    else                 this->longest_axis = 1;
	}
	else {
	    if ( ext_x < ext_z ) this->longest_axis = 2;
	    else                 this->longest_axis = 0;
	}
	//printf("Extension in x-direction: %f\n", this->extents_max_x - this->extents_min_x);
	//printf("Extension in y-direction: %f\n", this->extents_max_y - this->extents_min_y);
	//printf("Extension in z-direction: %f\n", this->extents_max_z - this->extents_min_z);
    }
    
    return;
}


/******************************************************************************
 *  Method:	    ledges_to_boxes_and_spheres
 *  Description:    This method will ...
 *  Input:	    <>  ...
 *  Output:	    ...
 *****************************************************************************/
void IVP_SurfaceBuilder_Ledge_Soup::ledges_to_boxes_and_spheres()
{
    // -----------------------------------------------------------------------------------
    // create a minimal sphere around each ledge and save it into a fix-sized array...
    // -----------------------------------------------------------------------------------    

    IVP_DOUBLE dimension_steps = IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;
    
    this->size_of_tree_in_bytes = 0; // debugging
    IVV_Sphere *sphere;
    
    // find number of spheres to generate
    int n_ledges = this->c_ledge_vec.len();
    this->number_of_terminal_spheres = n_ledges;

    // allocate array: element #0 (dummy) always indicates the first valid element through its 'next' pointer!
    this->spheres_cluster = (struct IVV_Sphere_Cluster *)p_calloc(this->number_of_terminal_spheres+1, sizeof(IVV_Sphere_Cluster));

    this->spheres_cluster[0].next = 1;
    int n = 1;
	int ledge_cnt;

    for (ledge_cnt = 0; ledge_cnt < n_ledges; ledge_cnt++) {
		IVP_Compact_Ledge* compact_ledge = this->c_ledge_vec.element_at(ledge_cnt);

		IVP_ASSERT(compact_ledge->get_n_triangles() > 0);

		sphere = new IVV_Sphere();
		this->all_spheres.add(sphere);

		IVP_U_Point min, max, rad;
		IVP_CLS.calc_bounding_box(compact_ledge, &min, &max);

		sphere->center.set_interpolate(&max, &min, 0.5f); // use center of bounding box as center of sphere
		rad.subtract(&max, &sphere->center);
		sphere->radius = rad.real_length(); // calculate radius of box-enclosing sphere

		if ( sphere->center.k[0]-sphere->radius < this->extents_min.k[0] ) this->extents_min.k[0] = sphere->center.k[0]-sphere->radius;
		if ( sphere->center.k[0]+sphere->radius > this->extents_max.k[0] ) this->extents_max.k[0] = sphere->center.k[0]+sphere->radius;
		if ( sphere->center.k[1]-sphere->radius < this->extents_min.k[1] ) this->extents_min.k[1] = sphere->center.k[1]-sphere->radius;
		if ( sphere->center.k[1]+sphere->radius > this->extents_max.k[1] ) this->extents_max.k[1] = sphere->center.k[1]+sphere->radius;
		if ( sphere->center.k[2]-sphere->radius < this->extents_min.k[2] ) this->extents_min.k[2] = sphere->center.k[2]-sphere->radius;
		if ( sphere->center.k[2]+sphere->radius > this->extents_max.k[2] ) this->extents_max.k[2] = sphere->center.k[2]+sphere->radius;
	
		// @@CB - scaling
		IVP_DOUBLE work = sphere->radius * dimension_steps; // dimension_steps is constanct, 1/250 @@CB
		sphere->box_sizes[0] = int((max.k[0] - sphere->center.k[0])/work)+1; // translate to origin, scale by 250, divide 
		sphere->box_sizes[1] = int((max.k[1] - sphere->center.k[1])/work)+1; // by radius, add 1
		sphere->box_sizes[2] = int((max.k[2] - sphere->center.k[2])/work)+1;
	
		this->size_of_tree_in_bytes += sizeof(IVV_Sphere); // debugging
	
		sphere->compact_ledge = compact_ledge;
		//	sphere->parent = NULL;
		sphere->child_1 = NULL;
		sphere->child_2 = NULL;
		sphere->number = n;

		this->spheres_cluster[n].previous = n-1;
		this->spheres_cluster[n].next     = n+1;
		this->spheres_cluster[n].sphere   = sphere;

		if ( this->smallest_radius == 0 ) {
		    this->smallest_radius = sphere->radius;
		} else if ( sphere->radius < this->smallest_radius ) {
		    this->smallest_radius = sphere->radius;
		}

		this->terminal_spheres.add(sphere);

		n++;
    }

    this->spheres_cluster[n-1].next = 0;
    
    // @@@SF: for speedup of single-convex objects, move this into sphere-clustering!
    if ( ledge_cnt > 1 ) {
		IVP_DOUBLE ext_x = this->extents_max.k[0] - this->extents_min.k[0];
		IVP_DOUBLE ext_y = this->extents_max.k[1] - this->extents_min.k[1];
		IVP_DOUBLE ext_z = this->extents_max.k[2] - this->extents_min.k[2];

		if ( ext_x < ext_y ) {

		    if ( ext_y < ext_z ) 
				this->longest_axis = 2;
			else                 
				this->longest_axis = 1;

		} else {

			if ( ext_x < ext_z ) 
				this->longest_axis = 2;
			else
				this->longest_axis = 0;

		}
    }
    
    return;
}


class IVP_Clustering_Shortrange_Interval_Min_Hash_Entry {
public:    
    IVP_BOOL    start;
    IVV_Sphere *sphere;

    IVP_Clustering_Shortrange_Interval_Min_Hash_Entry(IVP_BOOL start_in, IVV_Sphere *sphere_in) {
	this->start = start_in;
	this->sphere = sphere_in;
	return;
    }
};



/********************************************************************************
 *
 *  DIFFERENT CLUSTERING-METHODS
 *
 *******************************************************************************/


// ------------------------------------------------------------------------------
//   bottom-up
// ------------------------------------------------------------------------------

void IVP_SurfaceBuilder_Ledge_Soup::cluster_spheres_bottomup(IVP_DOUBLE threshold_increase) {

    float fixed_max_radius_for_new_sphere = this->smallest_radius;
	
    // initialize outer (interval) MinHash
    if ( this->number_of_terminal_spheres > 512 ) {
	this->interval_minhash = new IVV_Cluster_Min_Hash(4096*4);
    } else {
	this->interval_minhash = new IVV_Cluster_Min_Hash(256);
    }

    this->number_of_nodes = this->number_of_terminal_spheres; // [# of nodes] = [# of term.nodes] + [# of internal nodes]; needed for calculation of compact_surface size!

    this->number_of_unclustered_spheres = this->number_of_terminal_spheres;
    while ( this->number_of_unclustered_spheres > 1 ) {

	IVV_Cluster_Min_Hash cluster_min_hash(256); //@@@SF: start value still has to be optimized!

	// project all spheres onto the longest axis of ledge soup
	this->generate_interval_minhash(0.5f * fixed_max_radius_for_new_sphere);

	while ( this->interval_minhash->counter > 0 ) {

	    // retrieve smallest value on interval axis
	    IVP_Clustering_Shortrange_Interval_Min_Hash_Entry *entry = (IVP_Clustering_Shortrange_Interval_Min_Hash_Entry *)this->interval_minhash->find_min_elem();
	    this->interval_minhash->remove((void *)entry);

	    // value is left interval border (i.e. the smallest point occupied by sphere)
	    if ( entry->start ) {
		this->overlapping_spheres.add(entry->sphere);
		this->combine_spheres_in_vector(&cluster_min_hash); // calculate all possible combinations of all overlapping spheres
		
		// retrieve smallest mothersphere (if available!)
		IVV_Cluster_Min_Hash_Key key;
		//lwss - x64 fixes
		//key.key = (int)cluster_min_hash.find_min_elem();
		key.key = (intptr_t)cluster_min_hash.find_min_elem();
		//lwss end
		if ( key.key != (unsigned int)NULL ) { // verify that there were at least 2 spheres in vector and a new minimal sphere could be generated!
		    //IVP_DOUBLE radius = cluster_min_hash->find_min_value(); // radius of minimal sphere
		    int sphere_1_number = key.spheres.s1;
		    int sphere_2_number = key.spheres.s2;
		    
		    // remove all combinations from hash that contain the (soon to be) clustered child spheres...
		    this->remove_all_further_spherecombinations_from_hash(&cluster_min_hash, sphere_1_number);
		    this->remove_all_further_spherecombinations_from_hash(&cluster_min_hash, sphere_2_number);

		    // remove child-spheres of new minimal sphere from vector
		    IVV_Sphere *sphere1 = this->spheres_cluster[sphere_1_number].sphere;
		    IVV_Sphere *sphere2 = this->spheres_cluster[sphere_2_number].sphere;
		    this->overlapping_spheres.remove(sphere1);
		    this->overlapping_spheres.remove(sphere2);

		    // build new mothersphere and store it in separate list
		    IVV_Sphere *new_sphere = this->build_minimal_sphere(sphere1, sphere2);
		    this->built_spheres.add(new_sphere);
		}
	    }
	    // value is right interval border (i.e. the largest point occupied by sphere)
	    else {
		if ( this->overlapping_spheres.index_of(entry->sphere) != -1 ) {
		    this->overlapping_spheres.remove(entry->sphere);
		}
		this->remove_all_further_spherecombinations_from_hash(&cluster_min_hash, entry->sphere->number);
	    }

	    P_DELETE(entry);

	}

	IVP_ASSERT(this->interval_minhash->counter == 0);
	IVP_ASSERT(cluster_min_hash.counter == 0);
	IVP_ASSERT(this->overlapping_spheres.len() == 0);

	// increase threshold/blowup (if necessary)
	if ( this->built_spheres.len() == 0 ) {
	    //printf("No spheres could be combined. Increasing radius threshold from %f to %f.\n", fixed_max_radius_for_new_sphere, fixed_max_radius_for_new_sphere+threshold_increase);
	    fixed_max_radius_for_new_sphere *= threshold_increase;
	}

	// replace old spheres with newly built motherspheres
	this->replace_childspheres_in_spherelist_with_motherspheres();
	IVP_ASSERT(this->built_spheres.len() == 0);
    }

    P_DELETE(this->interval_minhash);

    return;
}


void IVP_SurfaceBuilder_Ledge_Soup::generate_interval_minhash(float fixed_max_radius) {

    IVV_Cluster_Min_Hash_Key new_key;
    
    int next_sphere;
    for (next_sphere=this->spheres_cluster[0].next; next_sphere!=0; next_sphere=this->spheres_cluster[next_sphere].next) {

	IVV_Sphere *sphere = this->spheres_cluster[next_sphere].sphere;
	IVP_DOUBLE radius = sphere->radius;
	if ( radius < fixed_max_radius ) {
	    radius = fixed_max_radius;
	}

	// insert interval start into interval MinHash
	IVP_Clustering_Shortrange_Interval_Min_Hash_Entry *new_entry_interval_start = new IVP_Clustering_Shortrange_Interval_Min_Hash_Entry(IVP_TRUE, sphere);
	//new_key.key = (int)new_entry_interval_start;
	new_key.key = (intptr_t)new_entry_interval_start; //lwss - x64 fix
	this->interval_minhash->add((void *)new_key.key, sphere->center.k[this->longest_axis]-radius);

	// insert interval end into interval MinHash
	IVP_Clustering_Shortrange_Interval_Min_Hash_Entry *new_entry_interval_end = new IVP_Clustering_Shortrange_Interval_Min_Hash_Entry(IVP_FALSE, sphere);
	//new_key.key = (int)new_entry_interval_end;
	new_key.key = (intptr_t)new_entry_interval_end; //lwss - x64 fix
	this->interval_minhash->add((void *)new_key.key, sphere->center.k[this->longest_axis]+radius);

    }

    return;
}


void IVP_SurfaceBuilder_Ledge_Soup::combine_spheres_in_vector(IVV_Cluster_Min_Hash *cluster_min_hash) {

    IVV_Cluster_Min_Hash_Key newkey;
    
    int x;
    for (x=0; x<this->overlapping_spheres.len(); x++) {

	IVV_Sphere *sphere_1 = this->overlapping_spheres.element_at(x);

	int y;
	for (y=0; y<this->overlapping_spheres.len(); y++) {
	    if ( x == y ) continue;
	    IVV_Sphere *sphere_2 = this->overlapping_spheres.element_at(y);

#if 0	    
	    IVP_DOUBLE qdistance = sphere_1->center.quad_distance_to(&sphere_2->center);
	    IVP_DOUBLE qdiff_radius = sphere_1->radius - sphere_2->radius;
	    qdiff_radius *= qdiff_radius;

	    // sort spheres: larger sphere is always left (i.e. position 1 :)
	    if ( sphere_1->radius < sphere_2->radius ) {
		IVV_Sphere *sphere_buf = sphere_1;
		sphere_1 = sphere_2;
		sphere_2 = sphere_buf;
	    }
	    
	    IVP_DOUBLE radius;
	    if ( qdistance <= qdiff_radius ) { // sphere 2 completely within sphere 1
		radius = sphere_1->radius;
	    }else {
		IVP_DOUBLE distance_2 = sqrt(qdistance) * 0.5f;
		IVP_DOUBLE r1r2_2 = IVP_Inline_Math::fabsd(sphere_1->radius - sphere_2->radius) * 0.5f;
		radius = distance_2 + sphere_1->radius - r1r2_2;
	    }
#else
 	    IVP_DOUBLE work1 = IVP_COMPACT_BOUNDINGBOX_STEP_SIZE * sphere_1->radius;
	    IVP_DOUBLE work2 = IVP_COMPACT_BOUNDINGBOX_STEP_SIZE * sphere_2->radius;
	
	    IVP_DOUBLE max1_x = sphere_1->center.k[0] + ( sphere_1->box_sizes[0] * work1 );
	    IVP_DOUBLE max1_y = sphere_1->center.k[1] + ( sphere_1->box_sizes[1] * work1 );
	    IVP_DOUBLE max1_z = sphere_1->center.k[2] + ( sphere_1->box_sizes[2] * work1 );
	    IVP_DOUBLE min1_x = sphere_1->center.k[0] - ( sphere_1->box_sizes[0] * work1 );
	    IVP_DOUBLE min1_y = sphere_1->center.k[1] - ( sphere_1->box_sizes[1] * work1 );
	    IVP_DOUBLE min1_z = sphere_1->center.k[2] - ( sphere_1->box_sizes[2] * work1 );

	    IVP_DOUBLE max2_x = sphere_2->center.k[0] + ( sphere_2->box_sizes[0] * work2 );
	    IVP_DOUBLE max2_y = sphere_2->center.k[1] + ( sphere_2->box_sizes[1] * work2 );
	    IVP_DOUBLE max2_z = sphere_2->center.k[2] + ( sphere_2->box_sizes[2] * work2 );
	    IVP_DOUBLE min2_x = sphere_2->center.k[0] - ( sphere_2->box_sizes[0] * work2 );
	    IVP_DOUBLE min2_y = sphere_2->center.k[1] - ( sphere_2->box_sizes[1] * work2 );
	    IVP_DOUBLE min2_z = sphere_2->center.k[2] - ( sphere_2->box_sizes[2] * work2 );

	    IVP_U_Point min, max, center, rad;
	    if ( min1_x < min2_x ) min.k[0] = min1_x; else min.k[0] = min2_x;
	    if ( min1_y < min2_y ) min.k[1] = min1_y; else min.k[1] = min2_y;
	    if ( min1_z < min2_z ) min.k[2] = min1_z; else min.k[2] = min2_z;
	    if ( max1_x > max2_x ) max.k[0] = max1_x; else max.k[0] = max2_x;
	    if ( max1_y > max2_y ) max.k[1] = max1_y; else max.k[1] = max2_y;
	    if ( max1_z > max2_z ) max.k[2] = max1_z; else max.k[2] = max2_z;

	    center.set_interpolate(&max, &min, 0.5f); // use center of bounding box as center of sphere
	    rad.subtract(&max, &center);
	    IVP_DOUBLE radius = rad.real_length(); // calculate radius of box-enclosing sphere
#endif	    

	    newkey.spheres.s1 = sphere_1->number;
	    newkey.spheres.s2 = sphere_2->number;
	    if ( !cluster_min_hash->is_elem((void *)newkey.key) ) {
		cluster_min_hash->add((void *)newkey.key, radius);
	    }

	}

    }
    
    return;
}


void IVP_SurfaceBuilder_Ledge_Soup::remove_all_further_spherecombinations_from_hash(IVV_Cluster_Min_Hash *cluster_min_hash, int spherenumber)
{

    // ----------------------------------------------------------------------
    // remove all possible combinations (even if not present in hash) of the
    // supplied sphere from hashtable
    // ----------------------------------------------------------------------
    
    IVV_Cluster_Min_Hash_Key remove_key;

    int x;
    for (x=0; x<this->overlapping_spheres.len(); x++) {

	IVV_Sphere *partner_sphere = this->overlapping_spheres.element_at(x);
	if ( partner_sphere->number == spherenumber ) continue;
	
	remove_key.spheres.s1 = spherenumber;
	remove_key.spheres.s2 = partner_sphere->number;
	cluster_min_hash->remove((void *)remove_key.key);

	remove_key.spheres.s1 = partner_sphere->number;
	remove_key.spheres.s2 = spherenumber;
	cluster_min_hash->remove((void *)remove_key.key);

    }

    return;
}


IVV_Sphere *IVP_SurfaceBuilder_Ledge_Soup::build_minimal_sphere(IVV_Sphere *sphere_1, IVV_Sphere *sphere_2) {

    static IVP_DOUBLE dimension_steps = IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;
    IVP_DOUBLE new_radius;
    IVP_U_Point new_center;
    
    IVV_Sphere *new_sphere = new IVV_Sphere();

#if 0    
    IVP_U_Point dist_vec;
    IVP_DOUBLE distance, distance_2;
    IVP_DOUBLE r1r2_2;
    IVP_DOUBLE interpolation_factor;
    
    // calculate the new mothersphere
    dist_vec.subtract(&sphere_1->center, &sphere_2->center);
    distance = dist_vec.fast_real_length();

    if ( distance + sphere_2->radius <= sphere_1->radius ) {
	new_radius = sphere_1->radius;
	new_center = sphere_1->center;
    }
    else {
	distance_2 = distance * 0.5f;
	r1r2_2 = IVP_Inline_Math::fabsd(sphere_1->radius - sphere_2->radius) * 0.5f;
	new_radius = distance_2 + sphere_1->radius - r1r2_2;
	
	interpolation_factor = ( distance_2 - r1r2_2 ) / distance;
	new_center.set_interpolate(&sphere_1->center, &sphere_2->center, interpolation_factor);
    }
#else
    IVP_DOUBLE work1 = IVP_COMPACT_BOUNDINGBOX_STEP_SIZE * sphere_1->radius;
    IVP_DOUBLE work2 = IVP_COMPACT_BOUNDINGBOX_STEP_SIZE * sphere_2->radius;
	
    IVP_DOUBLE max1_x = sphere_1->center.k[0] + ( sphere_1->box_sizes[0] * work1 );
    IVP_DOUBLE max1_y = sphere_1->center.k[1] + ( sphere_1->box_sizes[1] * work1 );
    IVP_DOUBLE max1_z = sphere_1->center.k[2] + ( sphere_1->box_sizes[2] * work1 );
    IVP_DOUBLE min1_x = sphere_1->center.k[0] - ( sphere_1->box_sizes[0] * work1 );
    IVP_DOUBLE min1_y = sphere_1->center.k[1] - ( sphere_1->box_sizes[1] * work1 );
    IVP_DOUBLE min1_z = sphere_1->center.k[2] - ( sphere_1->box_sizes[2] * work1 );

    IVP_DOUBLE max2_x = sphere_2->center.k[0] + ( sphere_2->box_sizes[0] * work2 );
    IVP_DOUBLE max2_y = sphere_2->center.k[1] + ( sphere_2->box_sizes[1] * work2 );
    IVP_DOUBLE max2_z = sphere_2->center.k[2] + ( sphere_2->box_sizes[2] * work2 );
    IVP_DOUBLE min2_x = sphere_2->center.k[0] - ( sphere_2->box_sizes[0] * work2 );
    IVP_DOUBLE min2_y = sphere_2->center.k[1] - ( sphere_2->box_sizes[1] * work2 );
    IVP_DOUBLE min2_z = sphere_2->center.k[2] - ( sphere_2->box_sizes[2] * work2 );

    IVP_U_Point min, max, rad;
    if ( min1_x < min2_x ) min.k[0] = min1_x; else min.k[0] = min2_x;
    if ( min1_y < min2_y ) min.k[1] = min1_y; else min.k[1] = min2_y;
    if ( min1_z < min2_z ) min.k[2] = min1_z; else min.k[2] = min2_z;
    if ( max1_x > max2_x ) max.k[0] = max1_x; else max.k[0] = max2_x;
    if ( max1_y > max2_y ) max.k[1] = max1_y; else max.k[1] = max2_y;
    if ( max1_z > max2_z ) max.k[2] = max1_z; else max.k[2] = max2_z;

    new_center.set_interpolate(&max, &min, 0.5f); // use center of bounding box as center of sphere
    rad.subtract(&max, &new_center);
    new_radius = rad.real_length(); // calculate radius of box-enclosing sphere
    
    IVP_DOUBLE work = new_radius * dimension_steps;
    new_sphere->box_sizes[0] = int((max.k[0]-new_center.k[0])/work)+1;
    new_sphere->box_sizes[1] = int((max.k[1]-new_center.k[1])/work)+1;
    new_sphere->box_sizes[2] = int((max.k[2]-new_center.k[2])/work)+1;
#endif    
    
    // initialize new mothersphere
    new_sphere->number = sphere_1->number; // replace sphere_1 (left sphere) with mothersphere
    new_sphere->radius = new_radius;
    new_sphere->center = new_center;
    new_sphere->compact_ledge = NULL;
    //    new_sphere->parent = NULL;
    new_sphere->child_1 = sphere_1;
    new_sphere->child_2 = sphere_2;

    return(new_sphere);
}


void IVP_SurfaceBuilder_Ledge_Soup::replace_childspheres_in_spherelist_with_motherspheres() {

    int x;
    for (x=0; x<this->built_spheres.len(); x++) {

	IVV_Sphere *mothersphere   = this->built_spheres.element_at(x);
	IVV_Sphere *child_sphere_1 = mothersphere->child_1;
	IVV_Sphere *child_sphere_2 = mothersphere->child_2;
	
	//	child_sphere_1->parent = mothersphere;
	//	child_sphere_2->parent = mothersphere;
	
	// replace old left sphere with new mothersphere, remove old right sphere completely...
	this->spheres_cluster[child_sphere_1->number].sphere = mothersphere;

	if ( child_sphere_2->number == this->spheres_cluster[0].next ) {
	    this->spheres_cluster[0].next = this->spheres_cluster[child_sphere_2->number].next;
	}
	this->spheres_cluster[child_sphere_2->number].sphere = NULL;
	this->spheres_cluster[this->spheres_cluster[child_sphere_2->number].previous].next = this->spheres_cluster[child_sphere_2->number].next;
	this->spheres_cluster[this->spheres_cluster[child_sphere_2->number].next].previous = this->spheres_cluster[child_sphere_2->number].previous;

	this->number_of_nodes++; // [# of nodes] = [# of term.nodes] + [# of internal nodes]
    
	// decrease number of unclustered spheres by one...
	this->number_of_unclustered_spheres--;

    }
    this->built_spheres.clear();

    return;

}


// ------------------------------------------------------------------------------
//   top-down
// ------------------------------------------------------------------------------

void IVP_SurfaceBuilder_Ledge_Soup::cluster_spheres_topdown_mediancut(IVP_DOUBLE /*threshold_increase*/) {

    IVP_U_Vector<IVV_Sphere> terminals;
    
    for(int next_sphere=this->spheres_cluster[0].next; next_sphere != 0; next_sphere = this->spheres_cluster[next_sphere].next) {
		terminals.add(this->spheres_cluster[next_sphere].sphere);
    }

    this->number_of_nodes = this->number_of_terminal_spheres; // [# of nodes] = [# of term.nodes] + [# of internal nodes]; needed for calculation of compact_surface size!
    
    this->spheres_cluster[this->spheres_cluster[0].next].sphere = this->cluster_spheres_topdown_mediancut_recursively(&terminals);
    
    return;
}


void IVP_SurfaceBuilder_Ledge_Soup::calculate_boundingbox(IVP_U_Vector<IVV_Sphere> *terminals, IVP_U_Float_Point *ext_min, IVP_U_Float_Point *ext_max) {

    static IVP_DOUBLE dimension_steps = IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;
    
    ext_min->set( 1000000.0f,  1000000.0f,  1000000.0f);
    ext_max->set(-1000000.0f, -1000000.0f, -1000000.0f);
    
    for(int x = 0; x<terminals->len(); x++) {
	
		IVV_Sphere *sphere = terminals->element_at(x);

		IVP_DOUBLE work = dimension_steps * sphere->radius;

		IVP_DOUBLE min[3], max[3];
		min[0] = sphere->center.k[0] - ( sphere->box_sizes[0] * work );
		min[1] = sphere->center.k[1] - ( sphere->box_sizes[1] * work );
		min[2] = sphere->center.k[2] - ( sphere->box_sizes[2] * work );
		max[0] = sphere->center.k[0] + ( sphere->box_sizes[0] * work );
		max[1] = sphere->center.k[1] + ( sphere->box_sizes[1] * work );
		max[2] = sphere->center.k[2] + ( sphere->box_sizes[2] * work );

		if ( min[0] < ext_min->k[0] ) ext_min->k[0] = min[0];
		if ( max[0] > ext_max->k[0] ) ext_max->k[0] = max[0];
		if ( min[1] < ext_min->k[1] ) ext_min->k[1] = min[1];
		if ( max[1] > ext_max->k[1] ) ext_max->k[1] = max[1];
		if ( min[2] < ext_min->k[2] ) ext_min->k[2] = min[2];
		if ( max[2] > ext_max->k[2] ) ext_max->k[2] = max[2];

    }

    return;
}


IVV_Sphere *IVP_SurfaceBuilder_Ledge_Soup::cluster_spheres_topdown_mediancut_recursively(IVP_U_Vector<IVV_Sphere> *terminals) {

    // Proceedings:
    // 1. calculate bounding box (and smallest enclosing sphere) for set of terminals
    // 2. find longest axis
    // 3. calculate median
    // 4. assign terminals to left- and righthand vectors according to their centers' positions
    // 5. recursively process left- and righthand vectors

    IVP_ASSERT(terminals->len() > 0);

    if ( terminals->len() == 1 ) return( terminals->element_at(0) ); // only one terminal left: return it

    // calculate the bounding box for the supplied set of terminals
    this->calculate_boundingbox(terminals, &this->extents_min, &this->extents_max);
    
    static IVP_DOUBLE dimension_steps = IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;
    IVP_U_Point rad;
    IVP_U_Point new_center;
    IVP_DOUBLE new_radius;
    new_center.set_interpolate(&this->extents_max, &this->extents_min, 0.5f); // use center of bounding box as center of sphere
    rad.subtract(&this->extents_max, &new_center);
    new_radius = rad.real_length(); // calculate radius of box-enclosing sphere
    IVP_DOUBLE work = new_radius * dimension_steps;

    // initialize new mothersphere
    IVV_Sphere *new_sphere = new IVV_Sphere();
    this->all_spheres.add(new_sphere);
    new_sphere->number = 0;
    new_sphere->radius = new_radius;
    new_sphere->center = new_center;
    new_sphere->compact_ledge = NULL;
    new_sphere->box_sizes[0] = int((this->extents_max.k[0]-new_center.k[0])/work)+1;
    new_sphere->box_sizes[1] = int((this->extents_max.k[1]-new_center.k[1])/work)+1;
    new_sphere->box_sizes[2] = int((this->extents_max.k[2]-new_center.k[2])/work)+1;

    // special case: "two spheres left in branch"
    // independent of the spheres' position we simply assign one of them to the left branch and one to the right branch.
    if ( terminals->len() == 2 ) {
	
		// recursively process subsets
		IVP_U_Vector<IVV_Sphere> lefthand_terminals, righthand_terminals;
		lefthand_terminals.add(terminals->element_at(0));
		righthand_terminals.add(terminals->element_at(1));
		new_sphere->child_1 = this->cluster_spheres_topdown_mediancut_recursively(&lefthand_terminals);
		new_sphere->child_2 = this->cluster_spheres_topdown_mediancut_recursively(&righthand_terminals);
    
		IVP_ASSERT(new_sphere->child_1 != 0);
		IVP_ASSERT(new_sphere->child_2 != 0);
		
		this->number_of_nodes++; // [# of nodes] = [# of term.nodes] + [# of internal nodes]
		
		return(new_sphere);

    }
    
    // set order of axes
    IVP_DOUBLE exts[3];
    exts[0] = this->extents_max.k[0] - this->extents_min.k[0];
    exts[1] = this->extents_max.k[1] - this->extents_min.k[1];
    exts[2] = this->extents_max.k[2] - this->extents_min.k[2];

#ifndef IVP_CLUSTER_SHORTRANGE_BALANCED
    int axes_order[3];

    if ( exts[0] < exts[1] ) {

		if ( exts[1] < exts[2] ) { 
			axes_order[0]=2;  
			axes_order[1]=1;  
			axes_order[2]=0; 
		} else if ( exts[0] < exts[2] ) { 
			axes_order[0]=1;  
			axes_order[1]=2;  
			axes_order[2]=0; 
		} else { 
			axes_order[0]=1;  
			axes_order[1]=0;  
			axes_order[2]=2; 
		}

    } else {

		if ( exts[0] < exts[2] ) { 
			axes_order[0]=2;  
			axes_order[1]=0;  
			axes_order[2]=1; 
		} else if ( exts[1] < exts[2] ) { 
			axes_order[0]=0;  
			axes_order[1]=2;  
			axes_order[2]=1; 
		} else { 
			axes_order[0]=0;  
			axes_order[1]=1;  
			axes_order[2]=2; 
		}

    }
#endif    

    //IVP_U_Vector<IVV_Sphere> lefthand_terminals[3], righthand_terminals[3];
    IVP_U_Vector<IVV_Sphere> *lefthand_terminals[3], *righthand_terminals[3];
    lefthand_terminals[0] = new IVP_U_Vector<IVV_Sphere>(terminals->len());
    lefthand_terminals[1] = new IVP_U_Vector<IVV_Sphere>(terminals->len());
    lefthand_terminals[2] = new IVP_U_Vector<IVV_Sphere>(terminals->len());
    righthand_terminals[0] = new IVP_U_Vector<IVV_Sphere>(terminals->len());
    righthand_terminals[1] = new IVP_U_Vector<IVV_Sphere>(terminals->len());
    righthand_terminals[2] = new IVP_U_Vector<IVV_Sphere>(terminals->len());
    
    IVP_FLOAT difference_in_boundingbox_volumes[3];

#ifdef IVP_CLUSTER_SHORTRANGE_BALANCED
    for(int axis = 0; axis<3; axis++) // we are balancing over three axes
#else
	int axis = axes_order[0]; // we just balance over the first
#endif
	{    
		// calculate median on chosen axis
		IVP_DOUBLE median = 0.0f;

		for(int median_average = 0; median_average < terminals->len(); median_average++) {
			median += terminals->element_at(median_average)->center.k[axis];
		}

		median /= terminals->len();

		IVP_IF(0) {
//		    ivp_message("Median on axis %d: %f\n", axis, median);
		}

		// assign terminals to either side of splitting plane
		// (this is a bit tricky as we have to carefully handle terminals lying on the median itself;
		//  to avoid ending up with an empty subbox by assigning a median-terminal randomly to the
		//  same side as the non-median-terminal, we examine the non-median-terminal and choose the
		//  opposite side; in case of more than one median-terminal we alternate between left and
		//  right side)
		IVP_BOOL toggle = IVP_TRUE;

		for(int x = 0; x < terminals->len(); x++) {
			IVV_Sphere* terminal_sphere = terminals->element_at(x);

			if ( terminal_sphere->center.k[axis] < (median-P_FLOAT_RES) ) {

				lefthand_terminals[axis]->add(terminal_sphere); // sphere center left of the median

			} else if ( terminal_sphere->center.k[axis] > (median+P_FLOAT_RES) ) {

				righthand_terminals[axis]->add(terminal_sphere); // sphere center right of the median

			} else { // current sphere center on median

				IVV_Sphere* reference_sphere;

				if ( x == terminals->len() - 1 ) {
					reference_sphere = terminals->element_at(x - 1);
				} else {
					reference_sphere = terminals->element_at(x + 1);
				}

				if ( reference_sphere->center.k[axis] < (median-P_FLOAT_RES) ) {
					righthand_terminals[axis]->add(terminal_sphere);
					continue;
				}

				if ( reference_sphere->center.k[axis] > (median+P_FLOAT_RES) ) {
					lefthand_terminals[axis]->add(terminal_sphere);
					continue;
				}

				{ // reference sphere also had center on median!!
    				if ( lefthand_terminals[axis]->len() == 0 ) {
						lefthand_terminals[axis]->add(terminal_sphere);
						continue;
					}

    				if ( righthand_terminals[axis]->len() == 0 ) {
						righthand_terminals[axis]->add(terminal_sphere);
						continue;
					}

					if ( toggle ) {
						lefthand_terminals[axis]->add(terminal_sphere);
						toggle = IVP_FALSE;
					} else {
						righthand_terminals[axis]->add(terminal_sphere);
						toggle = IVP_TRUE;
					}
				}
			}
		}

		// calculate children's boundingboxes for all three possible splits and choose the split with
		// optimal balanced boundingbox volume/surface
		IVP_U_Float_Point min[2], max[2];
		this->calculate_boundingbox(lefthand_terminals[axis], &min[0], &max[0]);
		this->calculate_boundingbox(righthand_terminals[axis], &min[1], &max[1]);

#ifdef IVP_CLUSTER_SHORTRANGE_BALANCE_VOLUME	
		IVP_U_Float_Point dim1(IVP_Inline_Math::fabsd(max[0].k[0]-min[0].k[0]),
			IVP_Inline_Math::fabsd(max[0].k[1]-min[0].k[1]),
			IVP_Inline_Math::fabsd(max[0].k[2]-min[0].k[2])); // slower!
		IVP_U_Float_Point dim2(IVP_Inline_Math::fabsd(max[1].k[0]-min[1].k[0]),
			IVP_Inline_Math::fabsd(max[1].k[1]-min[1].k[1]),
			IVP_Inline_Math::fabsd(max[1].k[2]-min[1].k[2])); // slower!
		IVP_FLOAT volume1 = dim1.k[0] * dim1.k[1] * dim1.k[2];
		IVP_FLOAT volume2 = dim2.k[0] * dim2.k[1] * dim2.k[2];
		//IVP_FLOAT volume1 = fabs(max[0].k[0]-min[0].k[0]) * fabs(max[0].k[1]-min[0].k[1]) * fabs(max[0].k[2]-min[0].k[2]); // faster!
		//IVP_FLOAT volume2 = fabs(max[1].k[0]-min[1].k[0]) * fabs(max[1].k[1]-min[1].k[1]) * fabs(max[1].k[2]-min[1].k[2]); // faster!
		difference_in_boundingbox_volumes[axis] = IVP_Inline_Math::fabsd(volume2-volume1);
#else

		//difference_in_boundingbox_volumes[axis] = max[left].k[axis] - min[right].k[axis];
		IVP_U_Float_Point dim1(IVP_Inline_Math::fabsd(max[0].k[0]-min[0].k[0]), IVP_Inline_Math::fabsd(max[0].k[1]-min[0].k[1]), IVP_Inline_Math::fabsd(max[0].k[2]-min[0].k[2])); // slower!
		IVP_U_Float_Point dim2(IVP_Inline_Math::fabsd(max[1].k[0]-min[1].k[0]), IVP_Inline_Math::fabsd(max[1].k[1]-min[1].k[1]), IVP_Inline_Math::fabsd(max[1].k[2]-min[1].k[2])); // slower!
		IVP_FLOAT volume1 = dim1.k[0] * dim1.k[1] * dim1.k[2];
		IVP_FLOAT volume2 = dim2.k[0] * dim2.k[1] * dim2.k[2];
		//IVP_FLOAT volume1 = fabs(max[0].k[0]-min[0].k[0]) * fabs(max[0].k[1]-min[0].k[1]) * fabs(max[0].k[2]-min[0].k[2]); // faster!
		//IVP_FLOAT volume2 = fabs(max[1].k[0]-min[1].k[0]) * fabs(max[1].k[1]-min[1].k[1]) * fabs(max[1].k[2]-min[1].k[2]); // faster!
		difference_in_boundingbox_volumes[axis] = volume2+volume1;
#endif	
    }

#ifdef IVP_CLUSTER_SHORTRANGE_BALANCED    
    int chosen_axis = 0;
    { // find optimum axis
		IVP_DOUBLE max_diff = P_DOUBLE_MAX;

		for (int i = 0; i< 3; i++) {
			if ( difference_in_boundingbox_volumes[i] < max_diff ) {
				if ( (lefthand_terminals[i]->len() != 0) && (righthand_terminals[i]->len() != 0) )
					chosen_axis = i;

				max_diff = difference_in_boundingbox_volumes[i];
		    }
		}
    }
#else
    int chosen_axis = axes_order[0];
#endif    

    //printf("Chosen axis: %d\n", chosen_axis);
    
    IVP_ASSERT(lefthand_terminals[chosen_axis]->len() != 0);
    IVP_ASSERT(righthand_terminals[chosen_axis]->len() != 0);

    // recursively process subsets
    new_sphere->child_1 = this->cluster_spheres_topdown_mediancut_recursively(lefthand_terminals[chosen_axis]);
    new_sphere->child_2 = this->cluster_spheres_topdown_mediancut_recursively(righthand_terminals[chosen_axis]);

    P_DELETE(lefthand_terminals[0]);
    P_DELETE(lefthand_terminals[1]);
    P_DELETE(lefthand_terminals[2]);
    P_DELETE(righthand_terminals[0]);
    P_DELETE(righthand_terminals[1]);
    P_DELETE(righthand_terminals[2]);
    
    IVP_ASSERT(new_sphere->child_1 != 0);
    IVP_ASSERT(new_sphere->child_2 != 0);
    
    this->number_of_nodes++; // [# of nodes] = [# of term.nodes] + [# of internal nodes]
    
    return(new_sphere);
}




/********************************************************************************
 *	Name:	     	allocate_compact_surface
 *	Description:	allocates memory for compact surface and initializes
 *			some basic values
 ********************************************************************************/
IVP_Compact_Surface *IVP_SurfaceBuilder_Ledge_Soup::allocate_compact_surface()
{
        
    // allocate memory for "Compact Surface"
    int cs_header_size     = sizeof(IVP_Compact_Surface);

    int cs_estimated_ledglelist_size = 0;
    int number_of_triangles_compiled = 0;
    int number_of_ledges_compiled = 0;

    if (parameters->link_to_input_compact_ledges == IVP_FALSE){
	for (int i=0; i<this->terminal_spheres.len(); i++) {
       	    IVV_Sphere *sphere = this->terminal_spheres.element_at(i);
	    IVP_Compact_Ledge *source = sphere->compact_ledge;	    
	    int ledge_size = source->get_size(); // 'IVP_Compact_Ledge's have to be 16bytes-aligned!
    	    cs_estimated_ledglelist_size += ledge_size;
	    number_of_triangles_compiled += source->get_n_triangles();
	    number_of_ledges_compiled ++;
	}
    }

    for (int j=0; j<this->rec_spheres.len(); j++) {
       	IVV_Sphere *sphere = this->rec_spheres.element_at(j);
	IVP_Compact_Ledge *source = sphere->compact_ledge;
        int ledge_size = source->get_size(); // 'IVP_Compact_Ledge's have to be 16bytes-aligned!
        cs_estimated_ledglelist_size += ledge_size;
        number_of_triangles_compiled += source->get_n_triangles();
	number_of_ledges_compiled++;
    }

    int cs_ledgetree_size = this->number_of_nodes * sizeof(IVP_Compact_Ledgetree_Node);
    // complete size of compact surface
    int cs_size = cs_header_size + cs_estimated_ledglelist_size + cs_ledgetree_size;

    cs_size = (cs_size + 15) & ~0xf;

    IVP_IF(1){	/*printf("Compact surface size : %d\n", cs_size);*/    }
    
    this->compact_surface = (IVP_Compact_Surface *)ivp_malloc_aligned(cs_size*sizeof(char), 16);
    if ( this->compact_surface == 0 ) {
	return(0);
    }
    if ( number_of_ledges_compiled > 1 && !parameters->link_to_input_compact_ledges){
	if ( parameters->merge_points == IVP_SLMP_MERGE_AND_REALLOCATE ||
	    parameters->merge_points == IVP_SLMP_MERGE_NO_REALLOCATE){
	    int a = number_of_triangles_compiled; int b = 2;
	    while (a) { a = a>>1; b = b<<1; }
	    this->point_hash = new IVP_I_FPoint_VHash( b ); // very rough estimate
	    this->first_poly_point = (IVP_Compact_Poly_Point *) ( (char *)this->compact_surface + cs_header_size + 
	    number_of_triangles_compiled * sizeof(IVP_Compact_Triangle) + number_of_ledges_compiled * sizeof(IVP_Compact_Ledge));
	}
    }else{
	this->first_poly_point = NULL;
	this->point_hash = NULL;
    }

    n_poly_points_allocated  = 0;
    // now insert triangles (and maybe points
    {
        this->first_compact_ledge = (IVP_Compact_Ledge *)((char *)this->compact_surface + cs_header_size);
	this->insert_compact_ledges();
    }
    int cs_ledglelist_size;
    if ( this->point_hash ){
	P_DELETE ( this->point_hash );
	cs_ledglelist_size = number_of_triangles_compiled * sizeof(IVP_Compact_Triangle) + 
			     number_of_ledges_compiled * sizeof(IVP_Compact_Ledge) + 
			     n_poly_points_allocated * sizeof(IVP_Compact_Poly_Point);

    }else{
	cs_ledglelist_size = cs_estimated_ledglelist_size;
    }

     // set size values
    int cs_real_size = cs_header_size + cs_ledglelist_size + cs_ledgetree_size;
    this->compact_surface->byte_size = cs_real_size; // remember size of whole compact surface; useful for e.g. saving to disk
    this->compact_surface->offset_ledgetree_root = cs_header_size + cs_ledglelist_size;
    this->compact_surface->dummy[0] = 0; // otherwise RUI on SPARC
    this->compact_surface->dummy[1] = 0; // otherwise RUI on SPARC
    this->compact_surface->dummy[2] = 0; // otherwise RUI on SPARC

    // set pointer to first node in ledgetree array
    
   this->ledgetree_work =
	(IVP_Compact_Ledgetree_Node *)( (char *)this->compact_surface +   this->compact_surface->offset_ledgetree_root);
    IVP_IF(1) {
	this->clt_lowmem = (char *)this->ledgetree_work;
	this->clt_highmem = (char *)compact_surface + cs_real_size;
    }

    return(compact_surface);
}

int IVP_SurfaceBuilder_Ledge_Soup::recompile_point_indizes_of_compact_ledge(IVP_Compact_Ledge *ledge_source, char *dest){
    if ( !point_hash ){
	memcpy((void *)dest, ledge_source, ledge_source->get_size());
	return ledge_source->get_size();
    }
    IVP_Compact_Ledge *dl = (IVP_Compact_Ledge *)dest;
    int size = sizeof(IVP_Compact_Ledge) + sizeof(IVP_Compact_Triangle) * ledge_source->get_n_triangles();
    memcpy((void *)dest, ledge_source, size);

    IVP_Compact_Triangle *tri = dl->get_first_triangle();
    IVP_Compact_Poly_Point *source_ledge_points = ledge_source->get_point_array();

    for (int t = dl->get_n_triangles()-1; t>=0; t--){
	IVP_Compact_Edge *edge = tri->get_first_edge();
	for (int e = 0; e<3;e++){
	    int pi = edge->get_start_point_index();
	    IVP_Compact_Poly_Point *p_in = & source_ledge_points[pi];
	    IVP_Compact_Poly_Point *p_out = (IVP_Compact_Poly_Point *)point_hash->find_point( p_in);
	    if (p_out){
		edge->set_start_point_index( p_out - first_poly_point);
	    }else{
		edge->set_start_point_index( this->n_poly_points_allocated );
		p_out = &first_poly_point[n_poly_points_allocated++];
		*p_out = *p_in;
		point_hash->add_point( p_out );
	    }
	    edge++;
	}   

	tri = tri->get_next_tri();
    }
    dl->set_offset_ledge_points( (char *)first_poly_point - (char *)dl );
    return size;
}

/********************************************************************************
 *	Name:	     	insert_compact_ledges
 *	Description:	insert compact ledges into compact surface structure
 ********************************************************************************/
void IVP_SurfaceBuilder_Ledge_Soup::insert_compact_ledges(){
    char *dest = (char *)this->first_compact_ledge;

    if (parameters->link_to_input_compact_ledges == IVP_FALSE){
	for (int i=0; i<this->terminal_spheres.len(); i++) {
       	    IVV_Sphere *sphere = this->terminal_spheres.element_at(i);
	    IVP_Compact_Ledge *source = sphere->compact_ledge;

	    IVP_ASSERT((long(dest) & 0xf) == 0);

	    sphere->compact_ledge = (IVP_Compact_Ledge *)dest;	    
	    int ledge_size = recompile_point_indizes_of_compact_ledge(source,dest);
	    dest += ledge_size;

	    if (parameters->free_input_compact_ledges == IVP_TRUE){
		P_FREE_ALIGNED(source);
	    }
	}
    }
    for (int j=0; j<this->rec_spheres.len(); j++) {
       	IVV_Sphere *sphere = this->rec_spheres.element_at(j);
	IVP_Compact_Ledge *source = sphere->compact_ledge;

	IVP_ASSERT((long(dest) & 0xf) == 0);

	sphere->compact_ledge = (IVP_Compact_Ledge *)dest;
	
	int ledge_size = recompile_point_indizes_of_compact_ledge(source,dest);
	dest += ledge_size;

	P_FREE_ALIGNED(source);
    }    
    this->c_ledge_vec.clear();
    IVP_ASSERT( !first_poly_point || dest == (char *)this->first_poly_point );
    return;
}


/********************************************************************************
 *	Name:	     	build_ledgetree
 *	Description:	recursively builds the tree of ledges into allocated
 *			compact surface memory;
 *			initiating call by 'create_compact_ledgetree()'
 ********************************************************************************/
IVP_Compact_Ledgetree_Node *IVP_SurfaceBuilder_Ledge_Soup::build_ledgetree(IVV_Sphere *node) {

    IVP_Compact_Ledgetree_Node *current_node = this->ledgetree_work;

    IVP_ASSERT((intptr_t)current_node <  (intptr_t)this->clt_highmem); // ledgetree memory overwrite!
    IVP_ASSERT((intptr_t)current_node >= (intptr_t)this->clt_lowmem);  // ledgetree memory underwrite!
	
    this->ledgetree_work++;
    
    // fill in generic ledgetree_node data
    current_node->center.set(node->center.k);
    current_node->radius = (IVP_FLOAT)(node->radius);

    for (int x=0; x < IVP_CLT_N_DIRECTIONS; x++) {
	current_node->box_sizes[x] = node->box_sizes[x];
    }
    current_node->free_0 = 0;

    
    if ( node->child_1 ) {
	IVP_ASSERT(node->child_2);
	if ( node->compact_ledge){
	    current_node->offset_compact_ledge = (char *)(node->compact_ledge)-(char *)current_node;
	    node->compact_ledge->ledgetree_node_offset = (char *) current_node - (char *)node->compact_ledge;
	    node->compact_ledge->has_chilren_flag = IVP_TRUE;
	}else{
	    current_node->offset_compact_ledge = 0;
	}

	// process child nodes
	build_ledgetree(node->child_1);
	IVP_Compact_Ledgetree_Node *address_of_right_branch = build_ledgetree(node->child_2);
	
	// fill in specific internal ledgetree_node data
	current_node->offset_right_node = (char *)address_of_right_branch - (char *)current_node; // calulcate offset to right branch
    } else {
	IVP_ASSERT(!node->child_2);	
	node->compact_ledge->has_chilren_flag = IVP_FALSE;

	// fill in specific terminal ledgetree_node data
	current_node->offset_compact_ledge = (char *)node->compact_ledge - (char *)current_node; // calculate backward offset to compact ledge
	current_node->offset_right_node = 0;
    }    
    IVP_ASSERT(current_node->box_sizes[0]!=0);

    return(current_node);
}


/********************************************************************************
 *	Name:	     	ledgetree_debug_output
 *	Description:	DEBUGGING ONLY
 ********************************************************************************/
void IVP_SurfaceBuilder_Ledge_Soup::ledgetree_debug_output(const IVP_Compact_Ledgetree_Node *node) const
{
    // *** debugging START ******************************************************
    IVP_IF(1) {
	IVP_ASSERT((intptr_t)node < (intptr_t)this->clt_highmem); // ledgetree memory overread!
	IVP_ASSERT((intptr_t)node >= (intptr_t)this->clt_lowmem); // ledgetree memory underread!
    }
    
    //for (int x=0; x<ivp_debug_sf_indent; x++) {
    //    printf("  ");
    //}
    //node->center.print("center");
    //printf("Radius: %.6f\n", node->radius);
    if (node->offset_right_node==0) return;
    //ivp_debug_sf_indent++;
    //printf("%d\n", node->offset);
    if ( !node->is_terminal() ) {
	ledgetree_debug_output(node->left_son());
	ledgetree_debug_output(node->right_son());
    }
    //ivp_debug_sf_indent--;
    // *** debugging END ********************************************************
    return;
}


/********************************************************************************
 *	Name:	     	ledgetree_array_debug_output
 *	Description:	DEBUGGING ONLY
 ********************************************************************************/
void IVP_SurfaceBuilder_Ledge_Soup::ledgetree_array_debug_output() {

    // *** debugging START ******************************************************
    //lwss hack - commented out during the x64 fixes. Does not seem used.
    //const IVP_Compact_Ledgetree_Node *node;
    //const IVP_Compact_Ledgetree_Node *nodes = compact_surface->get_compact_ledge_tree_root();
//
    //int i;
    //for (i=0; i<this->number_of_nodes; i++) {
	//node = &nodes[i];
	//printf("Node %d (address: 0x%x / %d)\n", i, (int)node, (int)node);
	////node->center.print("     center ");
	//printf("        radius %.6f)\n", node->radius);
	//printf("         left branch offset: %d (address: 0x%x / %d)\n", sizeof(*node), (int)(node+1), (int)(node+1));
	//printf("        right branch offset: %d (address: 0x%x / %d)\n", node->offset_right_node, (int)node+node->offset_right_node, (int)node+node->offset_right_node);
	//printf("\n");
    //}
    //lwss end
    // *** debugging END ********************************************************
    return;
}


void IVP_SurfaceBuilder_Ledge_Soup::insert_radius_in_compact_surface() {

    IVP_U_Point mass_center;
    IVP_U_Point rotation_inertia;
    IVP_Rot_Inertia_Solver::calc_mass_center_and_rotation_inertia(this->compact_surface, &mass_center, &rotation_inertia);

    IVP_DOUBLE mass_radius, mass_radius_dev;
    IVP_CLS.calc_radius_to_given_center( this->compact_surface, &mass_center,
					 &mass_radius, &mass_radius_dev);

    this->compact_surface->rotation_inertia.set(rotation_inertia.k);
    this->compact_surface->mass_center.set(mass_center.k);
    this->compact_surface->upper_limit_radius = (IVP_FLOAT)mass_radius;
    IVP_ASSERT( mass_radius_dev >=0.0f);
    IVP_ASSERT( mass_radius_dev <= mass_radius + P_FLOAT_RES);
    this->compact_surface->max_factor_surface_deviation = int(1.0f + mass_radius_dev / (mass_radius * IVP_COMPACT_SURFACE_DEVIATION_STEP_SIZE));
}


/********************************************************************************
 *	Name:	     	create_compact_ledgetree
 *	Description:	creates tree of ledges
 ********************************************************************************/
IVP_RETURN_TYPE IVP_SurfaceBuilder_Ledge_Soup::create_compact_ledgetree() {

    IVV_Sphere *cluster_node = this->spheres_cluster[this->spheres_cluster[0].next].sphere;

    // *** debugging START ******************************************************
    IVP_IF(0) {
	ivp_debug_sf_indent=0;
	debug_sphere_output(this->spheres_cluster[this->spheres_cluster[0].next].sphere);
    }
    // *** debugging END ********************************************************
#ifdef DEBUG
    IVP_Compact_Ledgetree_Node *root =
#endif
	this->build_ledgetree(cluster_node);
    this->spheres_cluster[0].next = 0;
    IVP_ASSERT(this->compact_surface->get_compact_ledge_tree_root()==root);
    
    // *** debugging START ******************************************************
    IVP_IF(1) {
	//ivp_debug_sf_indent=0;
	ledgetree_debug_output(this->compact_surface->get_compact_ledge_tree_root());
	//ledgetree_array_debug_output();
    }
    // *** debugging END ********************************************************

    return(IVP_OK);
}

#if defined(LINUX) || defined(SUN) || (__MWERKS__ && POWERPC)
void IVP_SurfaceBuilder_Ledge_Soup::convert_ledges_to_templates(IVP_U_BigVector<IVP_Compact_Ledge> &ledges,
								IVP_U_Vector<IVP_Template_Polygon> *templates_out)
{

    int i;
    for (i=0; i<ledges.len(); i++) {
	IVP_Compact_Ledge *ledge = ledges.element_at(i);

	int n_points_for_ledge = ledge->get_n_points();

	// copy all points of ledge (i.e. all points of all triangles) into pointlist
	int j;
	IVP_U_Vector<IVP_U_Point> points;
	const IVP_Compact_Poly_Point *cpp_array = ledge->get_point_array();
	for (j=0; j<n_points_for_ledge; j++) {
	    IVP_U_Point *p = new IVP_U_Point();
	    p->k[0] = cpp_array[j].k[0];
	    p->k[1] = cpp_array[j].k[1];
	    p->k[2] = cpp_array[j].k[2];
	    points.add(p);
	    //p->print();
	}

	// process all triangles in ledge
	IVP_U_Vector<IVP_SurMan_PS_Plane> planes;
	int n_faces = ledge->get_n_triangles();
	const IVP_Compact_Triangle *tri = ledge->get_first_triangle();
	int k;
	for (k=0; k<n_faces; k++) {
	    
	    const IVP_Compact_Edge *edge = tri->get_first_edge();
	    IVP_SurMan_PS_Plane *plane = new IVP_SurMan_PS_Plane();
	    
	    IVP_CLS.calc_hesse_vec_object_not_normized(edge, ledge, plane);
	    plane->normize();
 	    planes.add(plane);

	    int m;
	    for (m=0; m<3; m++) {
		const IVP_Compact_Poly_Point *p = edge->get_start_point(ledge);
		int index;
		for (index=0; index<n_points_for_ledge; index++) {
		    if ( &cpp_array[index] == p ) break;
		}
		plane->points.add(points.element_at(index));
		edge = edge->get_prev();
	    }
	    
	    tri = tri->get_next_tri();
	}

	
	// --------------------------------------------------
	// convert convex points & planes to polygon template
	// --------------------------------------------------
	IVP_Template_Polygon *templ;
	templ = IVP_SurfaceBuilder_Pointsoup::planes_to_template(&points, &planes);
	templates_out->add(templ);

	// -----------------------
	// clean up temporary data
	// -----------------------
	for (j=0; j<points.len(); j++) {
	    IVP_U_Point *p = points.element_at(j);
	    P_DELETE(p);
	}
	for (j=0; j<planes.len(); j++) {
	    IVP_SurMan_PS_Plane *p = planes.element_at(j);
	    P_DELETE(p);
	}
    }

    // --------------------------------------------------
    // convert convex points & planes to polygon template
    // --------------------------------------------------
//    *template_out = IVP_SurfaceBuilder_Pointsoup::planes_to_template(&points, &planes);
    return;
}

#endif




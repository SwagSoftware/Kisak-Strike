// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#define MY_DEBUG
//#define DEBUG_BASIC_OUTPUT
#define DEBUG_ERROR_OUTPUT
//#define DEBUG_CONNECTBOXES
//#define DEBUG_CALCOPTIMALBOX
//#define DEBUG_EXPANDTREE
//#define DEBUG_VECTORLISTS

#define DEBUG_IF if ( ctr > 0 )

#include <ivu_float.hxx>

#include <ivu_vhash.hxx>
#include <ivp_mindist.hxx>
#include <ivp_mindist_intern.hxx>
#include <ivp_clustering_longrange.hxx>
#include <ivp_clustering_lrange_hash.hxx>
#include <ivp_hull_manager.hxx>
#include <ivp_clustering_visualizer.hxx>

int ivp_debug_indent=0;

void ivp_indent_output()
{
    for (int indent_x=0; indent_x<ivp_debug_indent; indent_x++) {
        printf(" ");
    }
    return;
}



IVP_OV_Element::IVP_OV_Element(IVP_Real_Object *obj): collision_fvector(16)
{
    this->node = NULL;
    this->center.set_to_zero();
    this->radius = -1.0f;
    this->real_object = obj;
    this->hull_manager = NULL;
    return;
}

IVP_OV_Element::~IVP_OV_Element(){
    if (hull_manager){
	hull_manager->remove_synapse(this);
	hull_manager = NULL;
    }
    this->real_object->get_environment()->fire_object_is_removed_from_collision_detection(real_object);
    IVP_ASSERT( collision_fvector.len() == 0);
    this->real_object->get_environment()->get_ov_tree_manager()->remove_ov_element(this);
}

void IVP_OV_Element::add_to_hull_manager(IVP_Hull_Manager *hm, IVP_DOUBLE hull_time){
    const IVP_Time &current_time = real_object->get_environment()->get_current_time();
    if (hull_manager){
	IVP_ASSERT(hull_manager == hm);
	hull_manager->update_synapse(this,current_time,hull_time);
	return;
    }
    hull_manager = hm;
    hm->insert_synapse(this,current_time, hull_time);
}

IVP_HULL_ELEM_TYPE IVP_OV_Element::get_type(){ return IVP_HULL_ELEM_OO_CONNECTOR; };

void IVP_OV_Element::hull_manager_is_going_to_be_deleted_event(IVP_Hull_Manager *hm){
    IVP_ASSERT(hm == hull_manager );
    delete this;
}

void IVP_OV_Element::hull_limit_exceeded_event(IVP_Hull_Manager *hm, IVP_HTIME){
    IVP_ASSERT(hm == hull_manager);
    real_object->get_environment()->get_mindist_manager()->recheck_ov_element( real_object );
}

void IVP_OV_Element::add_oo_collision(IVP_Collision *connector){
    collision_fvector.add(connector);
}

void IVP_OV_Element::remove_oo_collision(IVP_Collision *connector){
    collision_fvector.remove_allow_resort(connector);
}
    


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


IVP_OV_Node::IVP_OV_Node()
{
    this->parent = 0;
    return;
}


IVP_OV_Node::~IVP_OV_Node()
{
    if (parent){
	parent->children.remove(this);
    }
    while (children.len()){
	delete children.element_at(0);
    }
    // do not delete the elements
    return;
}


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


IVP_OV_Tree_Manager::IVP_OV_Tree_Manager()
{
    
    this->environment = NULL;
    this->root = NULL;
    this->hash_table = new IVP_ov_tree_hash(256);

    IVP_DOUBLE y = 1.0f;
    IVP_DOUBLE iy = 1.0f;
    
    for (int x=0 ; x<=40; x++) {
	this->powerlist[40+x] = y;
	this->powerlist[40-x] = iy;
	y *= 2.0f;
	iy *= 0.5f;
    }
    
    return;
}


IVP_OV_Tree_Manager::~IVP_OV_Tree_Manager()
{
    P_DELETE(this->hash_table);
    return;
}


int IVP_OV_Tree_Manager::log_base2(IVP_DOUBLE x) const
{
    return PFM_LD(x);
}



IVP_BOOL IVP_OV_Tree_Manager::box_contains_box(const struct IVP_OV_Node_Data *master_data,
					       const IVP_OV_Node *sub_node,
					       const int rasterlevel_diff) const
{
  // returns IVP_TRUE  if second box completely fits into first box
  // returns IVP_FALSE if second box is (partially) outside of first box
  
    int master_x1 =  master_data->x << rasterlevel_diff; // convert master rasterpoints to sub rasterpoints
    int master_y1 =  master_data->y << rasterlevel_diff;
    int master_z1 =  master_data->z << rasterlevel_diff;
    int offset = (2 << rasterlevel_diff) - 2;

    if ( sub_node->data.x < master_x1 ) return(IVP_FALSE);
    if ( sub_node->data.y < master_y1 ) return(IVP_FALSE);
    if ( sub_node->data.z < master_z1 ) return(IVP_FALSE);

    if ( sub_node->data.x > (master_x1+offset) ) return(IVP_FALSE);
    if ( sub_node->data.y > (master_y1+offset) ) return(IVP_FALSE);
    if ( sub_node->data.z > (master_z1+offset) ) return(IVP_FALSE);

    return(IVP_TRUE);
}
#if !defined(SUN) && !defined(LINUX) && !(__MWERKS__ && __POWERPC__) && !defined(GEKKO)
inline int ivp_int_floor(IVP_DOUBLE x){
    return (int)floorf(x);
}
inline int ivp_int_ceil(IVP_DOUBLE x){
    return (int)ceilf(x);
}
#else
inline int ivp_int_floor(IVP_DOUBLE x){
    return (int)floor(x);
}
inline int ivp_int_ceil(IVP_DOUBLE x){
    return (int)ceil(x);
}
#endif

IVP_DOUBLE IVP_OV_Tree_Manager::calc_optimal_box(const IVP_OV_Element *element, IVP_DOUBLE min_radius, IVP_DOUBLE max_radius)
{
    IVP_DOUBLE raster_size   = 2.0f * min_radius;
    int    raster_level  = this->log_base2(raster_size)+1;
    int    raster_points;

    // as our selfmade power/log algorithm only works for the range of -40 to 40 we have to manually
    // crop any possible underflow (caused by extremely small objects) to the minimum value, i.e. we
    // have a minimum boxsize of 2^-40 meters to be used by our tree
    if ( raster_level < -40 ) {
	raster_level = -40;
	IVP_IF(1) {
	    printf("IVP_OV_Tree_Manager - WARNING: Object too small! Adjusting raster size to 2^-40 meters.\n");
	}
    }
    
#ifdef DEBUG_CALCOPTIMALBOX
    DEBUG_IF {
	ivp_indent_output();
	printf("log2(%f) = %f\n", raster_size, this->log2(raster_size));
	ivp_indent_output();
	printf("initial rasterlevel: %d\n", raster_level);
    }
#endif    

    int   x_min, x_max;
    int   y_min, y_max;
    int   z_min, z_max;

    // first find the perfect box for our element using the minimal radius
    for (;; raster_level++) {

        raster_points = raster_level - 1;
        IVP_DOUBLE iraster_dist   = this->power2(-raster_points);
#ifdef DEBUG_CALCOPTIMALBOX
	DEBUG_IF {
	    ivp_indent_output();
	    printf("raster distance: %f\n", raster_dist);
	}
#endif    
        x_min = ivp_int_floor((element->center.k[0]-min_radius) * iraster_dist);
        x_max = ivp_int_ceil((element->center.k[0]+min_radius) * iraster_dist);

#ifdef DEBUG_CALCOPTIMALBOX
	DEBUG_IF {
	    ivp_indent_output();
	    printf("x: %d - %d\n", x_min, x_max);
	}
#endif    
        if ( x_max > x_min+2 ) continue;
        
        y_min = ivp_int_floor((element->center.k[1]-min_radius) * iraster_dist);
        y_max = ivp_int_ceil((element->center.k[1]+min_radius) *iraster_dist);

#ifdef DEBUG_CALCOPTIMALBOX
	DEBUG_IF {
	    ivp_indent_output();
	    printf("y: %d - %d\n", y_min, y_max);
	}
#endif    
        if ( y_max > y_min+2 )       continue;
        
        z_min = ivp_int_floor((element->center.k[2]-min_radius) * iraster_dist);
        z_max = ivp_int_ceil((element->center.k[2]+min_radius) * iraster_dist);

#ifdef DEBUG_CALCOPTIMALBOX
	DEBUG_IF {
	    ivp_indent_output();
	    printf("z: %d - %d\n", z_min, z_max);
	}
#endif    
        if ( z_max > z_min+2 )       continue;

        break;

    }

    this->search_node.data.x           = x_min;
    this->search_node.data.y           = y_min;
    this->search_node.data.z           = z_min;
    this->search_node.data.rasterlevel = raster_points;
    this->search_node.data.sizelevel   = raster_level;

    
    // check whether the element will fit into a box one level above using the
    // maximum radius; if this operation fails, we simply keep the values from
    // the first (smaller) box
    if ( min_radius >= max_radius ) {
        return(min_radius);
    }

    raster_level++;
    raster_points = raster_level - 1;
    IVP_DOUBLE iraster_dist   = this->power2(-raster_points);

    x_min = ivp_int_floor((element->center.k[0]-max_radius) * iraster_dist);
    x_max = ivp_int_ceil((element->center.k[0]+max_radius) *iraster_dist);

#ifdef DEBUG_CALCOPTIMALBOX
    DEBUG_IF {
	ivp_indent_output();
	printf("x: %d - %d\n", x_min, x_max);
    }
#endif    
    if ( x_max > x_min+2 ) {
        return(min_radius);
    }
        
    y_min = ivp_int_floor((element->center.k[1]-max_radius) * iraster_dist);
    y_max = ivp_int_ceil((element->center.k[1]+max_radius) * iraster_dist);

#ifdef DEBUG_CALCOPTIMALBOX
    DEBUG_IF {
	ivp_indent_output();
	printf("y: %d - %d\n", y_min, y_max);
    }
#endif    
    if ( y_max > y_min+2 ) {
        return(min_radius);
    }
        
    z_min = ivp_int_floor((element->center.k[2]-max_radius) * iraster_dist);
    z_max = ivp_int_ceil((element->center.k[2]+max_radius) * iraster_dist);

#ifdef DEBUG_CALCOPTIMALBOX
    DEBUG_IF {
	ivp_indent_output();
	printf("z: %d - %d\n", z_min, z_max);
    }
#endif    
    if ( z_max > z_min+2 ) {
        return(min_radius);
    }

    this->search_node.data.x           = x_min;
    this->search_node.data.y           = y_min;
    this->search_node.data.z           = z_min;
    this->search_node.data.rasterlevel = raster_points;
    this->search_node.data.sizelevel   = raster_level;

    return(max_radius);
}


IVP_BOOL IVP_OV_Tree_Manager::box_overlaps_with_box(const IVP_OV_Node *largenode,
						    const IVP_OV_Node *smallnode,
						    const int rasterlevel_diff) const
// returns IVP_TRUE  if the two boxes overlap
// returns IVP_FALSE if the two boxes are disjunkt
{
    if ( smallnode->data.x+2 <= (largenode->data.x << rasterlevel_diff) ) return(IVP_FALSE);
    if ( smallnode->data.y+2 <= (largenode->data.y << rasterlevel_diff) ) return(IVP_FALSE);
    if ( smallnode->data.z+2 <= (largenode->data.z << rasterlevel_diff) ) return(IVP_FALSE);

    if ( smallnode->data.x >= ((largenode->data.x + 2) << rasterlevel_diff) ) return(IVP_FALSE);
    if ( smallnode->data.y >= ((largenode->data.y + 2) << rasterlevel_diff) ) return(IVP_FALSE);
    if ( smallnode->data.z >= ((largenode->data.z + 2) << rasterlevel_diff) ) return(IVP_FALSE);

    return(IVP_TRUE);
}


IVP_OV_Node *IVP_OV_Tree_Manager::find_smallest_box(const IVP_OV_Node *master_node, const IVP_OV_Node *sub_node) const
{
    int i;
    int rasterlevel_diff = (master_node->data.rasterlevel-1) - sub_node->data.rasterlevel;
    for (i=0; i<master_node->children.len(); i++) {
	const IVP_OV_Node *child = master_node->children.element_at(i);
        if ( box_contains_box(&child->data, sub_node, rasterlevel_diff) == IVP_TRUE ) {
            return(find_smallest_box(child, sub_node));
        }
    }
    
    return((IVP_OV_Node *)master_node);
}


void IVP_OV_Tree_Manager::connect_boxes(IVP_OV_Node *node, IVP_OV_Node *new_node)
{
    int rasterlevel_diff = node->data.sizelevel - new_node->data.sizelevel;

    IVP_ASSERT(rasterlevel_diff>0);

    if ( rasterlevel_diff == 1 ) {
	// new node is exactly one level below -> simply insert it as one of our children
	new_node->parent = node;
	node->children.add(new_node);
#ifdef MY_DEBUG
	if ( node->children.n_elems > 27 ) printf("*** ERROR *** Excessive amount of children: %d\n", node->children.n_elems);
#endif			

#ifdef DEBUG_CONNECTBOXES
	DEBUG_IF {
	    ivp_indent_output();
	    printf("New box found one level below. Inserting it...\n");
	}
#endif
	return;
    }
    
    // create the new subbox,
    // add it to the current node, and continue the search using
    // the new subnode;
    IVP_OV_Node *new_subnode = new IVP_OV_Node();
	

    // if new node lower than center, use lowest row
    if ( new_node->data.x >= ((node->data.x+1)<<rasterlevel_diff) ) {
	new_subnode->data.x = (node->data.x+1)*2;
    }
    // if new node lower than quarter, use middle row
    else if ( new_node->data.x >= (((node->data.x<<1)+1)<<(rasterlevel_diff-1)) ) {
	new_subnode->data.x = (node->data.x*2)+1;
    }
    // if none of the above is true, use highest row
    else {
	new_subnode->data.x = node->data.x*2;
    }

    // if new node lower than center, use lowest row
    if ( new_node->data.y >= ((node->data.y+1)<<rasterlevel_diff) ) {
	new_subnode->data.y = (node->data.y+1)*2;
    }
    // if new node lower than quarter, use middle row
    else if ( new_node->data.y >= (((node->data.y<<1)+1)<<(rasterlevel_diff-1)) ) {
	new_subnode->data.y = (node->data.y*2)+1;
    }
    // if none of the above is true, use highest row
    else {
	new_subnode->data.y = node->data.y*2;
    }

    // if new node lower than center, use lowest row
    if ( new_node->data.z >= ((node->data.z+1)<<rasterlevel_diff) ) {
	new_subnode->data.z = (node->data.z+1)*2;
    }
    // if new node lower than quarter, use middle row
    else if ( new_node->data.z >= (((node->data.z<<1)+1)<<(rasterlevel_diff-1)) ) {
	new_subnode->data.z = (node->data.z*2)+1;
    }
    // if none of the above is true, use highest row
    else {
	new_subnode->data.z = node->data.z*2;
    }

    new_subnode->data.rasterlevel = node->data.rasterlevel - 1;
    new_subnode->data.sizelevel   = node->data.sizelevel - 1;
    new_subnode->parent           = node;

    node->children.add(new_subnode);
    this->hash_table->add_node(new_subnode);

#ifdef MY_DEBUG
    if ( node->children.n_elems > 27 ) printf("*** ERROR *** Excessive amount of children: %d\n", node->children.n_elems);
#endif

#ifdef DEBUG_CONNECTBOXES
    DEBUG_IF {
	ivp_indent_output();
	printf("Created new subbox (%+.4f/%+.4f/%+.4f [%d: %+d/%+d/%+d] +++ %d: %f)\n", \
	       this->power2(new_subnode->data.rasterlevel)*new_subnode->data.x, \
	       this->power2(new_subnode->data.rasterlevel)*new_subnode->data.y, \
	       this->power2(new_subnode->data.rasterlevel)*new_subnode->data.z, \
	       new_subnode->data.rasterlevel, \
	       new_subnode->data.x, \
	       new_subnode->data.y, \
	       new_subnode->data.z, \
	       new_subnode->data.sizelevel, \
	       this->power2(new_subnode->data.sizelevel));
    }	
#endif
		    
    connect_boxes(new_subnode, new_node);

    return;
}


void IVP_OV_Tree_Manager::expand_tree(const IVP_OV_Node *new_node)
{
#ifdef DEBUG_EXPANDTREE
    DEBUG_IF {
	ivp_indent_output();
	printf("Expanding tree...\n");
    }
#endif
#ifdef DEBUG_ERROR_OUTPUT
    if ( this->root->data.sizelevel > 40 ) printf("*** ERROR *** Excessive sizelevel (%d) for element\n", this->root->data.sizelevel);
#endif

    IVP_ASSERT(this->root->data.sizelevel<41);
    
    IVP_OV_Node *old_root = this->root;

    IVP_OV_Node *new_root = new IVP_OV_Node();

    int rest_x = old_root->data.x%2;
    int rest_y = old_root->data.y%2;
    int rest_z = old_root->data.z%2;
#ifdef DEBUG_EXPANDTREE
    DEBUG_IF {
	ivp_indent_output();
	printf("Reste: %d %d %d\n", rest_x, rest_y, rest_z);
    }
#endif    
    new_root->data.x           = IVP_Inline_Math::int_div_2(old_root->data.x);
    new_root->data.y           = IVP_Inline_Math::int_div_2(old_root->data.y);
    new_root->data.z           = IVP_Inline_Math::int_div_2(old_root->data.z);
    new_root->data.rasterlevel = old_root->data.rasterlevel+1;
    new_root->data.sizelevel   = old_root->data.sizelevel+1;

#ifdef DEBUG_EXPANDTREE
    DEBUG_IF {
	ivp_indent_output();
	printf("Created new (larger) box...\n");
    }
#endif
    
    IVP_DOUBLE rasterpoints         = this->power2(new_root->data.rasterlevel);
    IVP_DOUBLE rasterpoints_newnode = this->power2(new_node->data.rasterlevel);
    
    if ( rest_x == -1 ) {
	new_root->data.x--;
    }
    else if ( rest_x == 0 ) {
	if ( (new_node->data.x*rasterpoints_newnode) < (rasterpoints * new_root->data.x) ) {
#ifdef DEBUG_EXPANDTREE
	    DEBUG_IF {
		ivp_indent_output();
		printf("Shifting box one rasterpoint to the left (x)...\n");
	    }
#endif
	    new_root->data.x--;
	}
    }
    if ( rest_y == -1 ) {
	new_root->data.y--;
    }
    else if ( rest_y == 0 ) {
	if ( (new_node->data.y*rasterpoints_newnode) < (rasterpoints * new_root->data.y) ) {
#ifdef DEBUG_EXPANDTREE
	    DEBUG_IF {
		ivp_indent_output();
		printf("Shifting box one rasterpoint back (y)...\n");
	    }
#endif
	    new_root->data.y--;
	}
    }
    if ( rest_z == -1 ) {
	new_root->data.z--;
    }
    else if ( rest_z == 0 ) {
	if ( (new_node->data.z*rasterpoints_newnode) < (rasterpoints * new_root->data.z) ) {
#ifdef DEBUG_EXPANDTREE
	    DEBUG_IF {
		ivp_indent_output();
		printf("Shifting box one rasterpoint down (z)...\n");
	    }
#endif
	    new_root->data.z--;
	}
    }

#ifdef DEBUG_EXPANDTREE
    DEBUG_IF {
	ivp_indent_output();
	printf("Connecting new (larger) box to existing tree...\n");
    }
#endif
    
#ifdef MY_DEBUG
    if ( new_root->children.n_elems >= 27 ) printf("*** ERROR *** : Mehr als 27 Kinder *********************************\n");
#endif			
    new_root->children.add(old_root);
    old_root->parent = new_root;
    this->hash_table->add_node(new_root);

    this->root = new_root;
    
#ifdef DEBUG_EXPANDTREE    
    DEBUG_IF {
	ivp_indent_output();
	printf("New root (%+.4f/%+.4f/%+.4f [%d: %+d/%+d/%+d] +++ %d: %f)\n", \
	       this->power2(this->root->data.rasterlevel)*this->root->data.x, \
	       this->power2(this->root->data.rasterlevel)*this->root->data.y, \
	       this->power2(this->root->data.rasterlevel)*this->root->data.z, \
	       this->root->data.rasterlevel, \
	       this->root->data.x, \
	       this->root->data.y, \
	       this->root->data.z, \
	       this->root->data.sizelevel, \
	       this->power2(this->root->data.sizelevel);
    }
#endif    
    return;
}


void IVP_OV_Tree_Manager::collect_subbox_collision_partners(const IVP_OV_Element *elem, const IVP_OV_Node *node)
{
    {
	for (int i=node->elements.len()-1;i>=0; i--) {		// check real distance of spheres
	    IVP_OV_Element *el = node->elements.element_at(i);
	    IVP_DOUBLE qdist = el->center.quad_distance_to(&elem->center);

	    IVP_DOUBLE minimal_dist = elem->radius + el->radius;
	    if ( qdist > minimal_dist * minimal_dist) continue;
	    this->collision_partners->add(el);
	}
    }
    {
	for (int i=node->children.len()-1; i>=0; i--) {
	    collect_subbox_collision_partners(elem, node->children.element_at(i));
	}
    }
    return;
}


void IVP_OV_Tree_Manager::collect_collision_partners(const IVP_OV_Element *elem, const IVP_OV_Node *masternode, const IVP_OV_Node *new_node)
{
    { // insert all element of higher levels 
	for (int i=masternode->elements.len()-1;i>=0; i--) {
		IVP_OV_Element *el = masternode->elements.element_at(i);
		IVP_DOUBLE qdist = el->center.quad_distance_to(&elem->center);

		IVP_DOUBLE minimal_dist = elem->radius + el->radius;
		if ( qdist > minimal_dist * minimal_dist) continue;
		this->collision_partners->add(el);
	}
    }
    // we have to sort the two cubes depending on their size
    if ( (masternode->data.sizelevel-1) > new_node->data.sizelevel ) {
	int rasterlevel_diff = (masternode->data.rasterlevel-1) - new_node->data.rasterlevel;
	for (int i=0; i<masternode->children.len(); i++) {
	    IVP_OV_Node *child = masternode->children.element_at(i);
	    if ( child == new_node ) {
		collect_subbox_collision_partners(elem, child);
	    } else {
		if ( box_overlaps_with_box(child, new_node, rasterlevel_diff) == IVP_TRUE ) {
		    collect_collision_partners(elem,child, new_node);
		}
	    }
	}
    }else {
	int rasterlevel_diff =  new_node->data.rasterlevel - (masternode->data.rasterlevel-1);
	for (int i=0; i<masternode->children.len(); i++) {
	    IVP_OV_Node *child = masternode->children.element_at(i);
	    if ( child == new_node ) {
		collect_subbox_collision_partners(elem, child);
	    }else {
		if ( box_overlaps_with_box(new_node, child, rasterlevel_diff) == IVP_TRUE ) {
		    collect_collision_partners(elem,child, new_node);
		}
	    }
        }
    }
    
    return;
}


IVP_DOUBLE IVP_OV_Tree_Manager::insert_ov_element(IVP_OV_Element *element,
                         IVP_DOUBLE min_radius,
                         IVP_DOUBLE max_radius,
                         IVP_U_Vector<IVP_OV_Element> *colliding_balls)
{
    if ( element == NULL ) {
        return(0);
    }
    
    IVP_DOUBLE used_radius;
    
    used_radius = calc_optimal_box(element, min_radius, max_radius);
	element->radius = (IVP_FLOAT)used_radius;

#ifdef IVP_HOME_BUILD
    // IVP_SUPREME_SUPPORT
    ivp_global_clustering_visualizer.longrange.add_object(element->real_object, &element->center, used_radius);
#endif

    // check if node (in terms of coordinates and size) is already present
    IVP_OV_Node *new_node=NULL;
    new_node = this->hash_table->find_node(&this->search_node);
    
    if ( new_node != NULL ) {

	// ------------------------------------------------
	// node already present in tree. Lets re-use it...
	// ------------------------------------------------
	
        new_node->elements.add(element);
        element->node = new_node;
    }else {
    
	// ------------------------------------------
	// desired node couldn't be found in tree...
	// ------------------------------------------
    

	// create new node...
	new_node = new IVP_OV_Node();
	new_node->data.x           = this->search_node.data.x;
	new_node->data.y           = this->search_node.data.y;
	new_node->data.z           = this->search_node.data.z;
	new_node->data.rasterlevel = this->search_node.data.rasterlevel;
	new_node->data.sizelevel   = this->search_node.data.sizelevel;

	new_node->elements.add(element);
	element->node = new_node;

	// the first object ever inserted defines our (preliminary) root box...
	if ( !this->root ) {
	    this->root = new_node;
	    this->hash_table->add_node(new_node);        
	    return(used_radius);
	}

	// check whether root box is already large enough for the element to completely fit in...
	for (;;) {
	    if ( this->root->data.rasterlevel >= new_node->data.rasterlevel ) {
		if ( !box_contains_box(&this->root->data, new_node, this->root->data.rasterlevel-new_node->data.rasterlevel) ) {
		    // new object won't fit into existing tree;
		    // expand tree until new object at least fits into root of tree...
		    expand_tree(new_node);
		    continue;
		}
	    }
	    else {
		// new object won't fit into existing tree;
		// expand tree until new object at least fits into root of tree...
		expand_tree(new_node);
		continue;
	    }
	    break;
	}

	// ---------------------------------------------------
	// new element will now fit (somewhere) into our tree
	// ---------------------------------------------------

	// new root cube is on same level as new node --> both are the same
	if ( this->root->data.rasterlevel == new_node->data.rasterlevel ) {
	    this->root->elements.add(element);
	    element->node = this->root;
	
	    P_DELETE(new_node);
	    new_node = this->root; // necessary for e.g. finding the collision partners (see below)

	} else {

	// insert new node into existing tree
	    this->hash_table->add_node(new_node);
	    
	    // search for the smallest box in one of the branches that completely embraces
	    // the new box
	    IVP_OV_Node *smallest_node;
	    smallest_node = find_smallest_box(this->root, new_node);

	    // now we can insert our new node (i.e. subbox) into our tree
	    connect_boxes(smallest_node, new_node);
	}
    }


    this->collision_partners = colliding_balls;
    if ( this->collision_partners ) {
    
	// search for colliding objects and insert them into the supplied list
	collect_collision_partners(element, this->root, new_node);

	// remove new element again from list (as we do not want our element to collide with itself :)
	//this->collision_partners->remove_allow_resort(element);  checks after are just faster than scanning this whole vector
    }
    
    // DONE...    
    return(used_radius);
}


IVP_OV_Node *IVP_OV_Tree_Manager::cleanup_node(IVP_OV_Node *node)
{
    if ( node->elements.n_elems != 0 ) return(NULL); // still elements left in node!
    if ( node->children.n_elems != 0 ) return(NULL); // still children left!

    if ( node->parent == NULL ) {
	this->root = NULL;
    }
    
    this->hash_table->remove_node(node);

    IVP_OV_Node *parent = node->parent;
    P_DELETE(node);

    return(parent);
}


void IVP_OV_Tree_Manager::remove_ov_element(IVP_OV_Element *element)
{
    
    IVP_OV_Node *node = element->node;
    if (!node) return;

#ifdef IVP_HOME_BUILD
    // IVP_SUPREME_SUPPORT
    ivp_global_clustering_visualizer.longrange.remove_object(element->real_object);
#endif

    element->node = NULL;
    node->elements.remove(element);

    while ( (node = cleanup_node(node)) != NULL ) { ; }

    return;
}


void IVP_OV_Tree_Manager::get_luf_coordinates_ws(const IVP_OV_Node *node, IVP_U_Float_Point *p, IVP_FLOAT *cubesize)
{
    p->k[0]   = (IVP_FLOAT)( this->power2(node->data.rasterlevel) * node->data.x );
    p->k[1]   = (IVP_FLOAT)( this->power2(node->data.rasterlevel) * node->data.y );
    p->k[2]   = (IVP_FLOAT)( this->power2(node->data.rasterlevel) * node->data.z );
    *cubesize = (IVP_FLOAT)( this->power2(node->data.sizelevel) );

    return;
}

void IVP_OV_Tree_Manager::get_center_coordinates_ws(const IVP_OV_Node *node, IVP_U_Float_Point *p, IVP_FLOAT *cubesize)
{
    p->k[0]   = (IVP_FLOAT)( this->power2(node->data.rasterlevel) * (node->data.x+1) );
    p->k[1]   = (IVP_FLOAT)( this->power2(node->data.rasterlevel) * (node->data.y+1) );
    p->k[2]   = (IVP_FLOAT)( this->power2(node->data.rasterlevel) * (node->data.z+1) );
    *cubesize = (IVP_FLOAT)( this->power2(node->data.sizelevel) );

    return;
}

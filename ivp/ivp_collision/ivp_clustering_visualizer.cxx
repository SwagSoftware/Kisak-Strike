// Copyright (C) Ipion Software GmbH 2000. All rights reserved.

// IVP_EXPORT_PRIVATE

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/ 


// IVP includes
#include <ivp_physics.hxx>
#include <ivu_vhash.hxx>
#include <ivp_surman_polygon.hxx>
#include <ivp_compact_ledge.hxx>
#include <ivp_compact_surface.hxx>
#include <ivp_clustering_visual_hash.hxx>
#include <ivp_cache_object.hxx>

#include <ivp_clustering_visualizer.hxx>

 

/*******************************************************************************
 * INTERNAL CLASSES & STRUCTS
 ******************************************************************************/ 

class IVP_Clustering_Visualizer_Shortrange_Nodedata {
public:
    IVP_Compact_Ledgetree_Node * node;
    int                          n_touches;
    IVP_BOOL                     reported_ledge;

    IVP_Clustering_Visualizer_Shortrange_Nodedata() {
	this->node = NULL;
	this->n_touches = 0;
	this->reported_ledge = IVP_FALSE;
    };
};

class IVP_Clustering_Visualizer_Shortrange_Objectdata {
public:
    IVP_Real_Object *                                           real_object;
    IVP_U_Vector<IVP_Clustering_Visualizer_Shortrange_Nodedata> ledgetree_nodes;
    IVP_U_Point center;
    IVP_DOUBLE  radius;

    IVP_Clustering_Visualizer_Shortrange_Objectdata() {
	this->real_object = NULL;
    };
};


/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/ 
#ifdef IVP_HOME_BUILD
IVP_Clustering_Visualizer ivp_global_clustering_visualizer;
#endif

/*******************************************************************************
 * PRIVATE METHODS
 ******************************************************************************/ 

IVP_Clustering_Visualizer_Shortrange_Callback::~IVP_Clustering_Visualizer_Shortrange_Callback() {

    return;
}


void IVP_Clustering_Visualizer_Shortrange::add_node_to_pipeline(const IVP_Compact_Ledgetree_Node *node, IVP_BOOL reported_ledge) {

    int x;
    for (x=0; x<this->pipelined_private_property_data->ledgetree_nodes.len(); x++) {
	IVP_Clustering_Visualizer_Shortrange_Nodedata *nodedata = this->pipelined_private_property_data->ledgetree_nodes.element_at(x);
	if ( nodedata->node == node ) {
	    nodedata->reported_ledge = (IVP_BOOL)((int)nodedata->reported_ledge || (int)reported_ledge);
	    if ( node->is_terminal() ) {
		nodedata->n_touches++; // no reference counting for non-terminal nodes!
	    }
	    return;
	}
    }

    IVP_Clustering_Visualizer_Shortrange_Nodedata *new_nodedata = new IVP_Clustering_Visualizer_Shortrange_Nodedata();
    new_nodedata->node = (IVP_Compact_Ledgetree_Node *)node;
    if ( node->is_terminal() ) new_nodedata->n_touches = 1;
    else                       new_nodedata->n_touches = 0; // no reference counting for non-terminal nodes!
    new_nodedata->reported_ledge = reported_ledge;

    this->pipelined_private_property_data->ledgetree_nodes.add(new_nodedata);

    //if ( this->remove_longrange_visualization ) {
    //	ivp_global_clustering_visualizer.longrange.devisualize(this->private_property);
    //}

    return;
}


IVP_BOOL IVP_Clustering_Visualizer_Shortrange::recursively_traverse_cluster(const IVP_Compact_Ledgetree_Node *node) {

    // returns: IVP_TRUE, whenever node has been aborted (this includes terminal nodes!)
    //          IVP_FALSE, whenever node is inner node of traversal tree

    // calculate some global values
    IVP_U_Point distance_between_centers;

    IVP_U_Point center; center.set(node->center.k);
    distance_between_centers.subtract(&center, &this->intruder_position_in_private_property_cs);

    {
	// first check sphere against sphere.
	this->stats_n_sphere_collisiontests++;

	IVP_DOUBLE sum_of_radii = this->intruder_radius + node->radius;

	if ( distance_between_centers.quad_length() >= sum_of_radii * sum_of_radii ) goto abort;
    }

    // --> sphere collision detected!

    {
	// now check bounding box against bounding box.
	this->stats_n_box_collisiontests++;

	register IVP_FLOAT work = IVP_COMPACT_BOUNDINGBOX_STEP_SIZE * node->radius;

	if ( IVP_Inline_Math::fabsd(distance_between_centers.k[0]) >= (node->box_sizes[0] * work + this->intruder_radius) ) goto abort;
	if ( IVP_Inline_Math::fabsd(distance_between_centers.k[1]) >= (node->box_sizes[1] * work + this->intruder_radius) ) goto abort;
	if ( IVP_Inline_Math::fabsd(distance_between_centers.k[2]) >= (node->box_sizes[2] * work + this->intruder_radius) ) goto abort;
    }

    // --> bounding box collision detected!

    if ( node->is_terminal() ) {
	this->add_node_to_pipeline(node, IVP_TRUE);
	return(IVP_TRUE);
    }
    else if ( this->boxmode == TRAVERSED_BOXES ) {
	this->add_node_to_pipeline(node, IVP_FALSE);
    }

    {
	this->recursively_traverse_cluster(node->left_son());
	this->recursively_traverse_cluster(node->right_son());
    }

    return(IVP_FALSE);

abort:    
    if ( this->boxmode != TERMINALS_ONLY ) { // display for ABORTED_BOXES and TRAVERSED_BOXES
	this->add_node_to_pipeline(node, IVP_FALSE);
    }
    return(IVP_TRUE);
}




/*******************************************************************************
 * PUBLIC METHODS
 ******************************************************************************/ 

void IVP_Clustering_Visualizer_Shortrange::visualize_collisions() {

    if ( !this->enabled ) return;

    this->stats_n_reported_ledges = 0;

    int i;
    for (i=0; i<this->output_callbacks.len(); i++) {

	IVP_Clustering_Visualizer_Shortrange_Callback *callback = this->output_callbacks.element_at(i);

	int x;
	for (x=this->pipelined_objects.len()-1; x>=0; x--) {
	    IVP_Clustering_Visualizer_Shortrange_Objectdata *objectdata = this->pipelined_objects.element_at(x);

	    int y;
	    for (y=objectdata->ledgetree_nodes.len()-1; y>=0; y--) {

		IVP_Clustering_Visualizer_Shortrange_Nodedata *nodedata = objectdata->ledgetree_nodes.element_at(y);
		IVP_Compact_Ledgetree_Node *node = nodedata->node;

		callback->private_property = objectdata->real_object; //this->private_property;
		callback->node = node;
		callback->n_touches = nodedata->n_touches;

		callback->position.set(node->center.k);
		callback->sphere_radius = node->radius;
		callback->reported_ledge = nodedata->reported_ledge;

		if ( callback->reported_ledge ) this->stats_n_reported_ledges++;

		IVP_DOUBLE work = IVP_COMPACT_BOUNDINGBOX_STEP_SIZE * node->radius;
		callback->box_extents.k[0] = node->box_sizes[0] * work;
		callback->box_extents.k[1] = node->box_sizes[1] * work;
		callback->box_extents.k[2] = node->box_sizes[2] * work;

		callback->visualize_request_for_node();

		objectdata->ledgetree_nodes.remove(nodedata);
		P_DELETE(nodedata);
	    }

	    // *** 
	    if ( objectdata->radius > 0.0 ) {
		callback->position.set(&objectdata->center);
		callback->box_extents.k[0] = objectdata->radius;
		callback->visualize_request_for_intruder_radius();
	    }
	    // *** 

	    this->pipelined_objects.remove(objectdata);
	    P_DELETE(objectdata);
	}

    }

    // reset values for next frame
    this->stats_n_sphere_collisiontests = 0;
    this->stats_n_box_collisiontests = 0;

    return;
}


void IVP_Clustering_Visualizer_Shortrange::analyze_collision(IVP_Real_Object *object0, IVP_DOUBLE radius0, IVP_Real_Object *object1, IVP_DOUBLE radius1) {

    if ( !this->enabled ) return;

    int x;
    for (x=0; x<2; x++) {

	if ( x == 0 ) {
	    this->private_property = object1;
	    this->intruder         = object0;
	    this->intruder_radius  = radius0;
	} else {
	    this->private_property = object0;
	    this->intruder         = object1;
	    this->intruder_radius  = radius1;
	}

	// filter out any unwanted objects
	if ( this->use_intruder_accept_filter && !this->accept_filter_intruder->find(this->intruder) )  continue; // only process intruder objects that are mentioned in the filter!
	if ( this->use_intruder_reject_filter &&  this->reject_filter_intruder->find(this->intruder) )  continue; // only process intruder objects that are mentioned in the filter!
	if ( this->use_private_property_accept_filter && !this->accept_filter_private_property->find(this->private_property) )  continue; // only process private_property objects that are mentioned in the filter!
	if ( this->use_private_property_reject_filter &&  this->reject_filter_private_property->find(this->private_property) )  continue; // only process private_property objects that are mentioned in the filter!


	IVP_SurfaceManager_Polygon *surface_manager = (IVP_SurfaceManager_Polygon *)private_property->get_surface_manager(); // because of above verification we KNOW that object is of type 'polygon'
	const IVP_Compact_Surface *compact_surface = surface_manager->get_compact_surface();
	const IVP_Compact_Ledgetree_Node *private_property_ledgetree_rootnode = compact_surface->get_compact_ledge_tree_root();

	this->intruder->get_m_world_f_object_AT(&intruder_matrix);

	IVP_Cache_Object *object_cache = this->private_property->get_cache_object();
	object_cache->transform_position_to_object_coords(&intruder_matrix.vv, &this->intruder_position_in_private_property_cs);
	object_cache->remove_reference();

	this->pipelined_private_property_data = NULL;
	int x;
	for (x=0; x<this->pipelined_objects.len(); x++) {
	    IVP_Clustering_Visualizer_Shortrange_Objectdata *objectdata = this->pipelined_objects.element_at(x);
	    if ( objectdata->real_object == this->private_property ) {
		this->pipelined_private_property_data = objectdata;
		break;
	    }
	}
	if ( !this->pipelined_private_property_data ) {
	    this->pipelined_private_property_data = new IVP_Clustering_Visualizer_Shortrange_Objectdata();
	    this->pipelined_private_property_data->real_object = this->private_property;
	    this->pipelined_private_property_data->center.set(&intruder_matrix.vv);
	    this->pipelined_private_property_data->radius = this->intruder_radius;
	    this->pipelined_objects.add(this->pipelined_private_property_data);
	}

	this->recursively_traverse_cluster(private_property_ledgetree_rootnode);

	// *** temporary hack !!!
	//IVP_U_Float_Point center(&intruder_matrix.vv);
	//ivp_global_clustering_visualizer.longrange.add_object(this->intruder, &center, this->intruder_radius);
	// *** temporary hack !!!

    }

    return;
}


void IVP_Clustering_Visualizer_Shortrange::add_object_to_intruder_filter(IVP_Real_Object *obj, IVP_CLUSTERING_VISUALIZER_FILTER filter) {

    switch ( filter ) {
    case ACCEPT:
	{
	    if ( !this->accept_filter_intruder->find(obj) ) { // pair already stored?
		this->accept_filter_intruder->add(obj);       // no, then store now...
	    }
	}
	break;
    case REJECT:
	{
	    if ( !this->reject_filter_intruder->find(obj) ) { // pair already stored?
		this->reject_filter_intruder->add(obj);       // no, then store now...
	    }
	}
	break;
    }

    return;
}


void IVP_Clustering_Visualizer_Shortrange::remove_object_from_intruder_filter(IVP_Real_Object *obj, IVP_CLUSTERING_VISUALIZER_FILTER filter) {

    switch ( filter ) {
    case ACCEPT:
	{
	    if ( this->accept_filter_intruder->find(obj) ) { // pair stored in hash?
		this->accept_filter_intruder->remove(obj); // yes, remove...
	    }
	}
	break;
    case REJECT:
	{
	    if ( this->reject_filter_intruder->find(obj) ) { // pair stored in hash?
		this->reject_filter_intruder->remove(obj); // yes, remove...
	    }
	}
	break;
    }

    return;
}


void IVP_Clustering_Visualizer_Shortrange::add_object_to_private_property_filter(IVP_Real_Object *obj, IVP_CLUSTERING_VISUALIZER_FILTER filter) {

    switch ( filter ) {
    case ACCEPT:
	{
	    if ( !this->accept_filter_private_property->find(obj) ) { // pair already stored?
		this->accept_filter_private_property->add(obj);       // no, then store now...
	    }
	}
	break;
    case REJECT:
	{
	    if ( !this->reject_filter_private_property->find(obj) ) { // pair already stored?
		this->reject_filter_private_property->add(obj);       // no, then store now...
	    }
	}
	break;
    }

    return;
}


void IVP_Clustering_Visualizer_Shortrange::remove_object_from_private_property_filter(IVP_Real_Object *obj, IVP_CLUSTERING_VISUALIZER_FILTER filter) {

    switch ( filter ) {
    case ACCEPT:
	{
	    if ( this->accept_filter_private_property->find(obj) ) { // pair stored in hash?
		this->accept_filter_private_property->remove(obj); // yes, remove...
	    }
	}
	break;
    case REJECT:
	{
	    if ( this->reject_filter_private_property->find(obj) ) { // pair stored in hash?
		this->reject_filter_private_property->remove(obj); // yes, remove...
	    }
	}
	break;
    }

    return;
}


void IVP_Clustering_Visualizer_Shortrange::install_visualize_callback(IVP_Clustering_Visualizer_Shortrange_Callback *callback) {

    this->output_callbacks.add(callback);
    return;
}


void IVP_Clustering_Visualizer_Shortrange::remove_visualize_callback(IVP_Clustering_Visualizer_Shortrange_Callback *callback) {

    if ( this->output_callbacks.index_of(callback) == -1 ) return;
    
    this->output_callbacks.remove(callback);
    return;
}


void IVP_Clustering_Visualizer_Shortrange::enable() {

    this->enabled = IVP_TRUE;

    return;
}


void IVP_Clustering_Visualizer_Shortrange::disable() {

    this->enabled = IVP_FALSE;

    return;
}


IVP_BOOL IVP_Clustering_Visualizer_Shortrange::get_state() {

    return(this->enabled);
}


IVP_Clustering_Visualizer_Shortrange::IVP_Clustering_Visualizer_Shortrange() {

    this->enabled = IVP_FALSE; // disabled by default!

    this->accept_filter_intruder         = new IVP_Clustering_Visualizer_Object_Hash(64);
    this->reject_filter_intruder         = new IVP_Clustering_Visualizer_Object_Hash(64);
    this->accept_filter_private_property = new IVP_Clustering_Visualizer_Object_Hash(64);
    this->reject_filter_private_property = new IVP_Clustering_Visualizer_Object_Hash(64);

    this->use_intruder_accept_filter         = IVP_FALSE;
    this->use_intruder_reject_filter         = IVP_FALSE;
    this->use_private_property_accept_filter = IVP_FALSE;
    this->use_private_property_reject_filter = IVP_FALSE;

//    this->remove_longrange_visualization = IVP_TRUE; //IVP_TRUE

    this->boxmode = TRAVERSED_BOXES; //ABORTED_BOXES TERMINALS_ONLY TRAVERSED_BOXES

    this->stats_n_sphere_collisiontests = 0;
    this->stats_n_box_collisiontests = 0;

    return;
}


IVP_Clustering_Visualizer_Shortrange::~IVP_Clustering_Visualizer_Shortrange() {

    this->enabled = IVP_FALSE; // just in case...

    P_DELETE(this->accept_filter_intruder);
    P_DELETE(this->reject_filter_intruder);
    P_DELETE(this->accept_filter_private_property);
    P_DELETE(this->reject_filter_private_property);

    return;
}










/*******************************************************************************
 *******************************************************************************
 *
 *  LONGRANGE CLUSTERING VISUALIZER
 *
 *******************************************************************************
 ******************************************************************************/

IVP_Clustering_Visualizer_Longrange_Callback::~IVP_Clustering_Visualizer_Longrange_Callback() {

    return;
}


/********************************************************************************
 *	Class:	       	IVP_CFEP_Hash
 *	Description:	hash table needed for fast storage of object pairs in
 *					"Exclusive Pair" collision filter
 ********************************************************************************/
class IVP_Clustering_Visualizer_Longrange_Hash : protected IVP_VHash {
protected:
    IVP_BOOL          compare     (void *elem0, void *elem1) const;
    int               obj_to_index(IVP_Real_Object *obj);

public:
    void              add         (IVP_Real_Object *obj);
    IVP_Real_Object * remove      (IVP_Real_Object *obj);
    IVP_Real_Object * find        (IVP_Real_Object *obj);

    IVP_Clustering_Visualizer_Longrange_Hash(int create_size);
    ~IVP_Clustering_Visualizer_Longrange_Hash();
};

IVP_BOOL IVP_Clustering_Visualizer_Longrange_Hash::compare(void *elem0, void *elem1) const {

    IVP_Real_Object *obj0 = (IVP_Real_Object *)elem0;
    IVP_Real_Object *obj1 = (IVP_Real_Object *)elem1;
    
    if ( obj0 != obj1 ) return(IVP_FALSE);
    
    return(IVP_TRUE);
}

int IVP_Clustering_Visualizer_Longrange_Hash::obj_to_index(IVP_Real_Object *obj) {

    return hash_index( (char *)obj, sizeof(obj));
};

void IVP_Clustering_Visualizer_Longrange_Hash::add(IVP_Real_Object *obj) {

    add_elem(obj, obj_to_index(obj));
}

IVP_Real_Object *IVP_Clustering_Visualizer_Longrange_Hash::remove(IVP_Real_Object *obj) {

    return (IVP_Real_Object *)remove_elem(obj, obj_to_index(obj));
}

IVP_Real_Object *IVP_Clustering_Visualizer_Longrange_Hash::find(IVP_Real_Object *obj) {

    return (IVP_Real_Object *)find_elem(obj, obj_to_index(obj));
}

IVP_Clustering_Visualizer_Longrange_Hash::IVP_Clustering_Visualizer_Longrange_Hash(int create_size) : IVP_VHash(create_size) {

    return;
}

IVP_Clustering_Visualizer_Longrange_Hash::~IVP_Clustering_Visualizer_Longrange_Hash() {

    return;
}


/*******************************************************************************
 *******************************************************************************
 *
 *  LONGRANGE CLUSTERING: METHODS
 *
 *******************************************************************************
 ******************************************************************************/

class IVP_Clustering_Visualizer_Longrange_Data {
public:
    IVP_Real_Object *  real_object;
    IVP_U_Float_Point  center;
    IVP_DOUBLE         radius;
};


void IVP_Clustering_Visualizer_Longrange::visualize() {

    if ( !this->enabled ) return;

    int x;
    for (x=this->visualize_data.len()-1; x>=0; x--) {
	IVP_Clustering_Visualizer_Longrange_Data *data = this->visualize_data.element_at(x);
	IVP_Real_Object *real_object = data->real_object;

	if (  real_object->get_core()->physical_unmoveable && !this->visualize_unmoveable_objects) return;
	if ( !real_object->get_core()->physical_unmoveable && !this->visualize_moveable_objects)   return;

	if ( this->use_accept_filter ) {
	    if ( !this->accept_filter->find(real_object) ) return;
	}

	if ( this->use_reject_filter ) {
	    if ( this->reject_filter->find(real_object) ) return;
	}

	int i;
	for (i=0; i<this->graphics_callbacks.len(); i++) {
	    IVP_Clustering_Visualizer_Longrange_Callback *callback = this->graphics_callbacks.element_at(i);

	    callback->real_object = real_object;
	    callback->center.set(&data->center);
	    callback->radius = data->radius;

	    callback->visualize_request();
	}

	this->visualize_data.remove(data);
	P_DELETE(data);
    }

    return;
}


void IVP_Clustering_Visualizer_Longrange::remove_object(IVP_Real_Object *real_object) {

    if ( !this->enabled ) return;

    this->remove_objectdata(real_object);

    int i;
    for (i=0; i<this->graphics_callbacks.len(); i++) {
	IVP_Clustering_Visualizer_Longrange_Callback *callback = this->graphics_callbacks.element_at(i);

	callback->real_object = real_object;
	callback->devisualize_request();
    }

    return;
}


void IVP_Clustering_Visualizer_Longrange::add_object(IVP_Real_Object *real_object, IVP_U_Float_Point *center, IVP_DOUBLE radius) {

    IVP_Clustering_Visualizer_Longrange_Data *data = new IVP_Clustering_Visualizer_Longrange_Data();
    data->real_object = real_object;
    data->center.set(center);
    data->radius = radius;

    this->visualize_data.add(data);

    return;
}


void IVP_Clustering_Visualizer_Longrange::remove_objectdata(IVP_Real_Object *real_object) {

    int x;
    for (x=0; x<this->visualize_data.len(); x++) {
	IVP_Clustering_Visualizer_Longrange_Data *data = this->visualize_data.element_at(x);
	if ( data->real_object != real_object ) continue;
	this->visualize_data.remove(data);
	P_DELETE(data);
	return;
    }

    return;
}


void IVP_Clustering_Visualizer_Longrange::install_visualize_callback(IVP_Clustering_Visualizer_Longrange_Callback *callback) {

    this->graphics_callbacks.add(callback);
    return;
}


void IVP_Clustering_Visualizer_Longrange::remove_visualize_callback(IVP_Clustering_Visualizer_Longrange_Callback *callback) {

    if ( this->graphics_callbacks.index_of(callback) == -1 ) {
	return;
    }
    
    this->graphics_callbacks.remove(callback);
    return;
}


void IVP_Clustering_Visualizer_Longrange::add_object_to_filter(IVP_Real_Object *obj, IVP_CLUSTERING_VISUALIZER_FILTER filter) {

    switch ( filter ) {
    case ACCEPT:
	{
	    if ( !this->accept_filter->find(obj) ) { // pair already stored?
		this->accept_filter->add(obj);       // no, then store now...
	    }
	}
	break;
    case REJECT:
	{
	    if ( !this->reject_filter->find(obj) ) { // pair already stored?
		this->reject_filter->add(obj);       // no, then store now...
	    }
	}
	break;
    }

    return;
}


void IVP_Clustering_Visualizer_Longrange::remove_object_from_filter(IVP_Real_Object *obj, IVP_CLUSTERING_VISUALIZER_FILTER filter) {

    switch ( filter ) {
    case ACCEPT:
	{
	    if ( this->accept_filter->find(obj) ) { // pair stored in hash?
		this->accept_filter->remove(obj); // yes, remove...
	    }
	}
	break;
    case REJECT:
	{
	    if ( this->reject_filter->find(obj) ) { // pair stored in hash?
		this->reject_filter->remove(obj); // yes, remove...
	    }
	}
	break;
    }

    return;
}


void IVP_Clustering_Visualizer_Longrange::enable() {

    int i;
    for (i=0; i<this->graphics_callbacks.len(); i++) {
	IVP_Clustering_Visualizer_Longrange_Callback *callback = this->graphics_callbacks.element_at(i);
	callback->enable();
	callback->enabled = IVP_TRUE;
    }

    this->enabled = IVP_TRUE;

    return;
}


void IVP_Clustering_Visualizer_Longrange::disable() {

    int i;
    for (i=0; i<this->graphics_callbacks.len(); i++) {
	IVP_Clustering_Visualizer_Longrange_Callback *callback = this->graphics_callbacks.element_at(i);
	callback->disable();
	callback->enabled = IVP_FALSE;
    }

    this->enabled = IVP_FALSE;

    return;
}


IVP_BOOL IVP_Clustering_Visualizer_Longrange::get_state() {

    return(this->enabled);
}


IVP_Clustering_Visualizer_Longrange::IVP_Clustering_Visualizer_Longrange() {

    this->enabled = IVP_FALSE;
    this->visualize_moveable_objects = IVP_FALSE; //TRUE;
    this->visualize_unmoveable_objects = IVP_FALSE; //TRUE;

    this->accept_filter = new IVP_Clustering_Visualizer_Longrange_Hash(1024);
    this->reject_filter = new IVP_Clustering_Visualizer_Longrange_Hash(1024);

    return;
}


IVP_Clustering_Visualizer_Longrange::~IVP_Clustering_Visualizer_Longrange() {

    P_DELETE(this->accept_filter);
    P_DELETE(this->reject_filter);

    return;
}






#if 0

IVP_Clustering_Visualizer_Shortrange::IVP_Clustering_Visualizer_Shortrange(IVP_Real_Object *object) {

    // { statistical output
    this->statistics_sphere_collisions = new IVP_BetterStatisticsmanager_Data_Entity(INT_VALUE);
    this->statistics_sphere_collisions->set_text("# of sphere collision tests: ");
    this->statistics_sphere_collisions->set_position(10, 10);

    this->statistics_box_collisions = new IVP_BetterStatisticsmanager_Data_Entity(INT_VALUE);
    this->statistics_box_collisions->set_text("# of box collision tests: ");
    this->statistics_box_collisions->set_position(10, 20);

    this->statistics_true_collisions = new IVP_BetterStatisticsmanager_Data_Entity(INT_VALUE);
    this->statistics_true_collisions->set_text("# of true collisions: ");
    this->statistics_true_collisions->set_position(10, 30);

    this->statistics_terminals = new IVP_BetterStatisticsmanager_Data_Entity(INT_VALUE);
    this->statistics_terminals->set_text("# of terminal collisions: ");
    this->statistics_terminals->set_position(10, 40);

    IVP_BetterStatisticsmanager *stats_manager = this->environment->get_betterstatisticsmanager();
    stats_manager->install_data_entity(this->statistics_sphere_collisions);
    stats_manager->install_data_entity(this->statistics_box_collisions);
    stats_manager->install_data_entity(this->statistics_true_collisions);
    stats_manager->install_data_entity(this->statistics_terminals);
    // statistical output }

    return;
}


#endif



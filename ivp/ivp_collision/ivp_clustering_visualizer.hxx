// Copyright (C) Ipion Software GmbH 2000. All rights reserved.

// IVP_EXPORT_PROTECTED


class IVP_Clustering_Visualizer_Longrange_Data;
class IVP_Clustering_Visualizer_Longrange_Hash;
class IVP_Clustering_Visualizer_Object_Hash;
class IVP_Clustering_Visualizer_Shortrange_Objectdata;
class IVP_Compact_Ledgetree_Node;


/********************************************************************************
 * Filename:	ivp_clustering_visualizer.hxx
 * Description:	This file provides you with assisting classes to visualize
 *		the object-internal clustering of concave objects in the
 *		Ipion Virtual Physics Engine as well as the time-spheres
 *		around physical objects.
 * Note:	These are abstract classes, i.e. they do not perform any
 *		significant output at all. You can install an arbitrary
 *		number of callbacks (subclassed from
 *		'IVP_Clustering_XXXrange_Visualizer_Callback_Interface',
 *		that can handle whatever visualisation you might need.
 * Classes:	IVP_Clustering_Visualizer_Shortrange
 *		IVP_Clustering_Visualizer_Longrange
 ********************************************************************************/

#ifndef IVP_CLUSTERING_VISUALIZER_INCLUDED
#define IVP_CLUSTERING_VISUALIZER_INCLUDED

enum IVP_CLUSTERING_VISUALIZER_FILTER {
    ACCEPT  = 1,
    REJECT  = 2 
};


/*******************************************************************************
 *******************************************************************************
 *
 *  LONGRANGE CLUSTERING VISUALIZER
 *
 *******************************************************************************
 ******************************************************************************/

/********************************************************************************
 * Name:	IVP_Clustering_Visualizer_Longrange_Callback   	
 * Description:	A callback called whenever the clustering visualizer needs to
 *		update the time-sphere around an object.
 ********************************************************************************/

class IVP_Clustering_Visualizer_Longrange_Callback {
public:
    IVP_BOOL          enabled;

    IVP_Real_Object * real_object;
    IVP_U_Float_Point center;
    IVP_DOUBLE        radius;

    virtual void      visualize_request() = 0;
    virtual void      devisualize_request() = 0;
    virtual void      enable() = 0;
    virtual void      disable() = 0;

    virtual ~IVP_Clustering_Visualizer_Longrange_Callback();
};


/********************************************************************************
 * Class:	IVP_Clustering_Visualizer_Longrange
 * Description: Visualizer class to help you visualize the time-sphere of a
 *		supplied physical object.
 *******************************************************************************/

class IVP_Clustering_Visualizer_Longrange {
private:
    // global data
    IVP_BOOL                                                   enabled;
    IVP_U_Vector<IVP_Clustering_Visualizer_Longrange_Callback> graphics_callbacks; // list of user-definable callbacks
    IVP_Clustering_Visualizer_Longrange_Hash *                 accept_filter;
    IVP_Clustering_Visualizer_Longrange_Hash *                 reject_filter;
    IVP_U_Vector<IVP_Clustering_Visualizer_Longrange_Data>     visualize_data;

    void remove_objectdata(IVP_Real_Object *real_object);

public:
    // global data
    IVP_BOOL use_accept_filter;
    IVP_BOOL use_reject_filter;
    IVP_BOOL visualize_moveable_objects;
    IVP_BOOL visualize_unmoveable_objects;

    void add_object_to_filter     (IVP_Real_Object *obj, IVP_CLUSTERING_VISUALIZER_FILTER filter);
    void remove_object_from_filter(IVP_Real_Object *obj, IVP_CLUSTERING_VISUALIZER_FILTER filter);

    void add_object   (IVP_Real_Object *real_object, IVP_U_Float_Point *center, IVP_DOUBLE radius);
    void remove_object(IVP_Real_Object *real_object);

    /******************************************************************************
     * Method:		visualize
     * Description:	Call this function to initiate the visualization
     *			of the object's time-sphere.
     * Input:		<real_object>  the physical object
     *			<center>       the center of the time-sphere
     *			<radius>       the radius of the time-sphere
     *****************************************************************************/
    void visualize();
    //void devisualize(IVP_Real_Object *real_object);

    /******************************************************************************
     * Method:		install_visualize_callback / remove_visualize_callback
     * Description:	Used to install or remove a certain visualization callback.
     * Input:		<callback>  callback to install/remove
     *****************************************************************************/
    void install_visualize_callback(IVP_Clustering_Visualizer_Longrange_Callback *callback);
    void remove_visualize_callback (IVP_Clustering_Visualizer_Longrange_Callback *callback);

    /******************************************************************************
     * Method:		enable / disable / get_state
     * Description:	Used to enable or disable the visualizer or to retrieve
     *			its current state.
     *****************************************************************************/
    void enable();
    void disable();
    IVP_BOOL get_state();

    /******************************************************************************
     * Method:		Class constructor / destructor
     * Description:	Standard constructor / destructor.
     *****************************************************************************/
    IVP_Clustering_Visualizer_Longrange();
    ~IVP_Clustering_Visualizer_Longrange();
};




/*******************************************************************************
 *******************************************************************************
 *
 *  SHORTRANGE CLUSTERING VISUALIZER
 *
 *******************************************************************************
 ******************************************************************************/

/********************************************************************************
 * Name:	IVP_Clustering_Visualizer_Shortrange_Callback   	
 * Description:	A callback called whenever ...
 ********************************************************************************/

class IVP_Clustering_Visualizer_Shortrange_Callback {
public:

    IVP_Real_Object * private_property;

    // these values define one single node
    IVP_Compact_Ledgetree_Node * node; // address of IVP_Compact_Ledgetree_Node
    IVP_U_Point                  position;
    IVP_DOUBLE                   sphere_radius;
    IVP_U_Point                  box_extents;
    int                          n_touches; // only used when node is terminal!
    IVP_BOOL                     reported_ledge; // IVP_TRUE if node is terminal and the corresponding ledge has been passed back from hierarchy check!
    void *                       user_data;

    virtual void  visualize_request_for_node() = 0;
    virtual void  visualize_request_for_intruder_radius() = 0;

    IVP_Clustering_Visualizer_Shortrange_Callback() {

	this->user_data = NULL;
	return;
    };

    virtual ~IVP_Clustering_Visualizer_Shortrange_Callback();
};




enum IVP_CLUSTERING_SHORTRANGE_VISUALIZER_BOXMODE {
    TERMINALS_ONLY  = 1,	// report only the bounding boxes of terminals
    ABORTED_BOXES   = 2,	// report all boxes that have been aborted during traversal
    TRAVERSED_BOXES = 3		// report all visited boxes
};


class IVP_Clustering_Visualizer_Shortrange {
private:
    // global data
    IVP_BOOL                                                    enabled;
    IVP_Clustering_Visualizer_Object_Hash *                     accept_filter_intruder;
    IVP_Clustering_Visualizer_Object_Hash *                     reject_filter_intruder;
    IVP_Clustering_Visualizer_Object_Hash *                     accept_filter_private_property;
    IVP_Clustering_Visualizer_Object_Hash *                     reject_filter_private_property;
    IVP_U_Vector<IVP_Clustering_Visualizer_Shortrange_Callback> output_callbacks; // list of user-definable callbacks

    // temporary data
    IVP_Real_Object * private_property;
    IVP_Real_Object * intruder;
    IVP_DOUBLE        intruder_radius;
    IVP_U_Matrix      intruder_matrix;
    IVP_U_Point       intruder_position_in_private_property_cs;  // position of intruder-object in private property's object's core coordinate-system

    IVP_U_Vector<IVP_Clustering_Visualizer_Shortrange_Objectdata> pipelined_objects;
    IVP_Clustering_Visualizer_Shortrange_Objectdata *             pipelined_private_property_data;

    void     add_node_to_pipeline        (const IVP_Compact_Ledgetree_Node *node, IVP_BOOL reported_ledge);
    IVP_BOOL recursively_traverse_cluster(const IVP_Compact_Ledgetree_Node *);

public:
    IVP_CLUSTERING_SHORTRANGE_VISUALIZER_BOXMODE boxmode; // see above
    IVP_BOOL                                     use_intruder_accept_filter;
    IVP_BOOL                                     use_intruder_reject_filter;
    IVP_BOOL                                     use_private_property_accept_filter;
    IVP_BOOL                                     use_private_property_reject_filter;
    IVP_BOOL                                     remove_longrange_visualization; // do not display longrange visualization if object hierarchy is traversed!

    int stats_n_reported_ledges;
    int stats_n_sphere_collisiontests;
    int stats_n_box_collisiontests;

    void visualize_collisions();
    void analyze_collision(IVP_Real_Object *object0, IVP_DOUBLE radius0, IVP_Real_Object *object1, IVP_DOUBLE radius1);

    void add_object_to_intruder_filter             (IVP_Real_Object *obj, IVP_CLUSTERING_VISUALIZER_FILTER filter);
    void add_object_to_private_property_filter     (IVP_Real_Object *obj, IVP_CLUSTERING_VISUALIZER_FILTER filter);
    void remove_object_from_intruder_filter        (IVP_Real_Object *obj, IVP_CLUSTERING_VISUALIZER_FILTER filter);
    void remove_object_from_private_property_filter(IVP_Real_Object *obj, IVP_CLUSTERING_VISUALIZER_FILTER filter);

    /******************************************************************************
     * Method:		install_visualize_callback / remove_visualize_callback
     * Description:	Used to install or remove a certain visualization callback.
     * Input:		<callback>  callback to install/remove
     *****************************************************************************/
    void install_visualize_callback(IVP_Clustering_Visualizer_Shortrange_Callback *callback);
    void remove_visualize_callback (IVP_Clustering_Visualizer_Shortrange_Callback *callback);

    /******************************************************************************
     * Method:		enable / disable / get_state
     * Description:	Used to enable or disable the visualizer or to retrieve
     *			its current state.
     *****************************************************************************/
    void enable();
    void disable();
    IVP_BOOL get_state();

    /******************************************************************************
     * Method:		Class constructor / destructor
     * Description:	Standard constructor / destructor.
     * Input:		<object>    the physical object to which the visualizer
     *				    shall be attached.
     *****************************************************************************/
    IVP_Clustering_Visualizer_Shortrange();
    ~IVP_Clustering_Visualizer_Shortrange();
};




/*******************************************************************************
 *******************************************************************************
 *
 *  CLUSTERING VISUALIZER MANAGER
 *
 *******************************************************************************
 ******************************************************************************/

/********************************************************************************
 * Class:	IVP_Clustering_Visualizer
 * Description: Main visualizer class.
 *******************************************************************************/

class IVP_Clustering_Visualizer {
public:
    IVP_Clustering_Visualizer_Shortrange  shortrange;
    IVP_Clustering_Visualizer_Longrange   longrange;
};

#ifdef IVP_HOME_BUILD
extern IVP_Clustering_Visualizer ivp_global_clustering_visualizer;
#endif

#endif

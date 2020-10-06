// Copyright (C) Ipion Software GmbH 2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 * Filename:	ivp_betterstatisticsmanager.hxx
 * Description:	...
 * Classes:	IVP_BetterStatisticsmanager
 *		etc.
 ********************************************************************************/

#ifndef IVP_BETTERSTATISTICSMANAGER_INCLUDED
#define IVP_BETTERSTATISTICSMANAGER_INCLUDED


/********************************************************************************
 * Name:	IVP_BetterStatisticsmanager_Callback_Interface   	
 * Description:	...
 ********************************************************************************/

enum IVP_BETTERSTATISTICSMANAGER_DATA_ENTITY_TYPE {
    INT_VALUE    = 1,	// one single 'int' value
    DOUBLE_VALUE = 2,	// one single 'double' value
    INT_ARRAY    = 3,	// an array of 'int' values
    DOUBLE_ARRAY = 4,	// an array of 'double' values
    STRING       = 5
};


class IVP_BetterStatisticsmanager_Data_Int_Array {
public:
    int		size;
    int *	array;
    int		max_value;
    int		xpos, ypos; // upper, left corner
    int		width, height; // graphical size of array
    int         bg_color, border_color, graph_color;
};


class IVP_BetterStatisticsmanager_Data_Double_Array {
public:
    int		size;
    IVP_DOUBLE *array;
    IVP_DOUBLE	max_value;
    int		xpos, ypos; // upper, left corner
    int		width, height; // graphical size of array
    int         bg_color, border_color, graph_color;
};


class IVP_BetterStatisticsmanager_Data_Entity {
private:
    IVP_BOOL   enabled;
    
public:
    IVP_BETTERSTATISTICSMANAGER_DATA_ENTITY_TYPE type;
    union {
	int         int_value;
	IVP_DOUBLE  double_value;
	IVP_BetterStatisticsmanager_Data_Int_Array    int_array;
	IVP_BetterStatisticsmanager_Data_Double_Array double_array;
    } data;
    char *     text;
    int        text_color;
    int        xpos, ypos;
    
    void       enable();
    void       disable();
    IVP_BOOL   get_state();

    void       set_int_value(int value);
    void       set_double_value(IVP_DOUBLE value);
    void       set_array_size(int size); // changing the array size will clear all values!
    void       set_int_array_latest_value(int value);
    void       set_double_array_latest_value(IVP_DOUBLE value);

    //int        get_int_value();
    //IVP_DOUBLE get_double_value();
    //int *      get_int_array();
    //int        get_int_array_latest_value();

    void       set_text(const char *text);
    void       set_position(int x, int y);

    IVP_BetterStatisticsmanager_Data_Entity(IVP_BETTERSTATISTICSMANAGER_DATA_ENTITY_TYPE type);
    ~IVP_BetterStatisticsmanager_Data_Entity();
};


/********************************************************************************
 * Name:	IVP_BetterStatisticsmanager_Callback_Interface   	
 * Description:	A callback called whenever the clustering visualizer needs to
 *		update a certain box in the clustering hierarchy.
 ********************************************************************************/
class IVP_BetterStatisticsmanager_Callback_Interface {
public:
    virtual void output_request(IVP_BetterStatisticsmanager_Data_Entity *entity) = 0;
    virtual void enable() = 0;
    virtual void disable() = 0;
};


/********************************************************************************
 * Class:	IVP_Clustering_Shortrange_Visualizer
 * Description: Visualizer class to help you visualize the object-internal
 *		clustering of concave objects in the Ipion Virtual Physics
 *		Engine.
 * Details:	Each instance of this class attaches to exactly one physical
 *		object. You can install an arbitrary number of callbacks
 *		('IVP_Clustering_Shortrange_Visualizer_Callback_Interface')
 *		to which the actual data will be passed.
 *		This class provides two traversion methods:
 *		1.  'INTRUDER' TRAVERSION: you specify a certain 'intruder'
 *		    object (in this case a cube defined by its center and its
 *		    'radius'); for each branch in the clustering hierarchy you
 *		    will get the node at which this branch's traversion has been
 *		    aborted.
 *		2.  'MANUAL' TRVERSION: you can navigate through the clustering
 *		    hierarchy 'by hand' using the supplied navigation-methods.
 *******************************************************************************/

class IVP_BetterStatisticsmanager {
private:
    // global data
    IVP_BOOL                                                     enabled;
    IVP_U_Vector<IVP_BetterStatisticsmanager_Callback_Interface> output_callbacks; // list of user-definable callbacks
    IVP_U_Vector<IVP_BetterStatisticsmanager_Data_Entity>        data_entities;    // list of all statistical data
    IVP_DOUBLE                                                   simulation_time;

    // temporary data

    // methods

public:
    // temporary data
    IVP_BOOL update_delayed;

    // global data
    IVP_DOUBLE update_interval;

    /******************************************************************************
     * Method:		print
     * Description:	Call this function to initiate the output of all
     *                  statistical values
     *****************************************************************************/
    void print();

    /******************************************************************************
     * Method:		set_simulation_time
     * Description:	Call this function to update the statistics manager's
     *                  internal simulation time counter
     *****************************************************************************/
    void set_simulation_time(IVP_DOUBLE time);

    /******************************************************************************
     * Method:		install_data_entity / remove_data_entity
     * Description:	Used to install or remove one single data entity.
     * Input:		<entity>  entity to install/remove
     *****************************************************************************/
    void install_data_entity(IVP_BetterStatisticsmanager_Data_Entity *entity);
    void remove_data_entity(IVP_BetterStatisticsmanager_Data_Entity *entity);

    /******************************************************************************
     * Method:		install_output_callback / remove_output_callback
     * Description:	Used to install or remove a certain output callback.
     * Input:		<callback>  callback to install/remove
     *****************************************************************************/
    void install_output_callback(IVP_BetterStatisticsmanager_Callback_Interface *callback);
    void remove_output_callback(IVP_BetterStatisticsmanager_Callback_Interface *callback);

    /******************************************************************************
     * Method:		enable / disable / get_state
     * Description:	Used to enable or disable the manager or to retrieve
     *			its current state.
     *****************************************************************************/
    void enable();
    void disable();
    IVP_BOOL get_state();

    /******************************************************************************
     * Method:		Class constructor / destructor
     * Description:	Standard constructor / destructor.
     *****************************************************************************/
    IVP_BetterStatisticsmanager();
    ~IVP_BetterStatisticsmanager();
};

#endif


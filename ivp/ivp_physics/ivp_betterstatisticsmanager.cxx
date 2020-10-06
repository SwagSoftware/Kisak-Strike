// Copyright (C) Ipion Software GmbH 2000. All rights reserved.

// IVP_EXPORT_PROTECTED

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/ 

// IVP includes
#include <ivp_physics.hxx>
#include <string.h>

#include <ivp_betterstatisticsmanager.hxx>


/*******************************************************************************
 * DEFINES
 ******************************************************************************/ 

#if 0
#define IVP_CLUSTERING_SHORTRANGE_VISUALIZER_ASSERT(errortext) \
{ \
    if ( !this->valid_instance ) { \
	IVP_IF(1) { \
	    IVP_IFDEBUG(IVP_DM_CLUSTERING_SHORTRANGE_VISUALIZER) { \
		ivp_debugmanager.dprint(IVP_DM_CLUSTERING_SHORTRANGE_VISUALIZER, errortext); \
	    } \
	} \
	return; \
    } \
}
#endif


/*******************************************************************************
 *******************************************************************************
 *
 *  METHODS - IVP_BetterStatisticsmanager_Data_Entity
 *
 *******************************************************************************
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC METHODS
 ******************************************************************************/ 

void IVP_BetterStatisticsmanager_Data_Entity::set_int_value(int value) {

    this->data.int_value = value;
    return;
}


void IVP_BetterStatisticsmanager_Data_Entity::set_double_value(IVP_DOUBLE value) {

    this->data.double_value = value;
    return;
}


void IVP_BetterStatisticsmanager_Data_Entity::set_array_size(int array_size) {

    switch ( this->type ) {
    case INT_ARRAY:
	P_FREE(this->data.int_array.array);
	this->data.int_array.array = (int *)p_calloc(array_size+1, sizeof(int));
	this->data.int_array.size = array_size;
	break;
    case DOUBLE_ARRAY:
	P_FREE(this->data.double_array.array);
	this->data.double_array.array = (IVP_DOUBLE *)p_calloc(array_size+1, sizeof(IVP_DOUBLE));
	this->data.double_array.size = array_size;
	break;
    default:
	break;
    }
    return;
}


void IVP_BetterStatisticsmanager_Data_Entity::set_int_array_latest_value(int value) {

    if ( !this->data.int_array.array ) return;

    int x;
    for (x=0; x<this->data.int_array.size-1; x++) {
	this->data.int_array.array[x] = this->data.int_array.array[x+1];
    }
    this->data.int_array.array[x] = value;
    return;
}


void IVP_BetterStatisticsmanager_Data_Entity::set_double_array_latest_value(IVP_DOUBLE value) {

    if ( !this->data.double_array.array ) return;

    int x;
    for (x=0; x<this->data.double_array.size-1; x++) {
	this->data.double_array.array[x] = this->data.double_array.array[x+1];
    }
    this->data.double_array.array[x] = value;
    return;
}


//int IVP_BetterStatisticsmanager_Data_Entity::get_int_value() {
//
//    return(this->data.int_value);
//}


//IVP_DOUBLE IVP_BetterStatisticsmanager_Data_Entity::get_double_value() {
//
//    return(this->data.double_value);
//}


//    void       set_int_array(int size);
//    void       set_int_array_latest_value(int value);

void IVP_BetterStatisticsmanager_Data_Entity::enable() {

    this->enabled = IVP_TRUE;

    return;
}


void IVP_BetterStatisticsmanager_Data_Entity::disable() {

    this->enabled = IVP_FALSE;

    return;
}


IVP_BOOL IVP_BetterStatisticsmanager_Data_Entity::get_state() {

    return(this->enabled);
}


void IVP_BetterStatisticsmanager_Data_Entity::set_text(const char *text_in) {

    char *new_text = p_strdup(text_in);
    if ( !new_text ) return;

    P_FREE(this->text);
    this->text = new_text;

    return;
}


void IVP_BetterStatisticsmanager_Data_Entity::set_position(int x, int y) {

    this->xpos = x;
    this->ypos = y;

    return;
}


IVP_BetterStatisticsmanager_Data_Entity::IVP_BetterStatisticsmanager_Data_Entity(IVP_BETTERSTATISTICSMANAGER_DATA_ENTITY_TYPE type_in) {

    this->enabled    = IVP_TRUE;
    this->type       = type_in;
    this->text       = p_strdup("Value: ");
    this->text_color = 0;
    this->xpos       = 0;
    this->ypos       = 0;

    switch ( this->type ) {
    case INT_VALUE:
	this->data.int_value = 0;
	break;
    case DOUBLE_VALUE:
	this->data.double_value = 0.0;
	break;
    case INT_ARRAY:
	this->data.int_array.size = 10;
	this->data.int_array.array = (int *)p_calloc(11, sizeof(int));
	this->data.int_array.max_value = 10;
	this->data.int_array.width = 10;
	this->data.int_array.height = 10;
	this->data.int_array.xpos = 0;
	this->data.int_array.ypos = 0;
	this->data.int_array.bg_color = 0;
	this->data.int_array.border_color = 1;
	this->data.int_array.graph_color = 2;
	break;
    case DOUBLE_ARRAY:
	this->data.double_array.size = 10;
	this->data.double_array.array = (IVP_DOUBLE *)p_calloc(11, sizeof(IVP_DOUBLE));
	this->data.double_array.max_value = 10.0;
	this->data.double_array.width = 10;
	this->data.double_array.height = 10;
	this->data.double_array.xpos = 0;
	this->data.double_array.ypos = 0;
	this->data.int_array.bg_color = 0;
	this->data.int_array.border_color = 1;
	this->data.int_array.graph_color = 2;
	break;
    case STRING:
	break;
    }

    return;
}


IVP_BetterStatisticsmanager_Data_Entity::~IVP_BetterStatisticsmanager_Data_Entity() {

    P_FREE(this->text);

    return;
}




/*******************************************************************************
 *******************************************************************************
 *
 *  METHODS - IVP_BetterStatisticsmanager
 *
 *******************************************************************************
 ******************************************************************************/


/*******************************************************************************
 * PRIVATE METHODS
 ******************************************************************************/ 



/*******************************************************************************
 * PUBLIC METHODS
 ******************************************************************************/ 

void IVP_BetterStatisticsmanager::print() {

    if ( !this->enabled ) return;
    if ( this->update_delayed ) return;

#if 0    
    static IVP_DOUBLE time_of_last_update = 0.0;
    if ( (simulation_time - time_of_last_update) < this->update_interval ) {
	this->update_delayed = IVP_TRUE;
	return;
    }
    time_of_last_update = this->simulation_time;
    this->update_delayed = IVP_FALSE;
#endif
    
    int i;
    for (i=0; i<this->output_callbacks.len(); i++) {
	IVP_BetterStatisticsmanager_Callback_Interface *callback = this->output_callbacks.element_at(i);

	int k;
	for (k=0; k<this->data_entities.len(); k++) {
	    IVP_BetterStatisticsmanager_Data_Entity *entity = this->data_entities.element_at(k);

	    if ( entity->get_state() ) {
		callback->output_request(entity);
	    }
	}
    }
    
    return;
}


void IVP_BetterStatisticsmanager::set_simulation_time(IVP_DOUBLE time) {
    
    this->simulation_time = time;
    
    static IVP_DOUBLE time_of_last_update = 0.0;
    if ( (simulation_time - time_of_last_update) < this->update_interval ) {
	this->update_delayed = IVP_TRUE;
	return;
    }
    time_of_last_update = this->simulation_time;
    //printf("Time: %f\n", time_of_last_update);
    this->update_delayed = IVP_FALSE;

    return;
}


void IVP_BetterStatisticsmanager::install_data_entity(IVP_BetterStatisticsmanager_Data_Entity *entity) {

    this->data_entities.add(entity);
    return;
}


void IVP_BetterStatisticsmanager::remove_data_entity(IVP_BetterStatisticsmanager_Data_Entity *entity) {

    if ( this->data_entities.index_of(entity) == -1 ) {
	return;
    }
    
    this->data_entities.remove(entity);
    return;
}


void IVP_BetterStatisticsmanager::install_output_callback(IVP_BetterStatisticsmanager_Callback_Interface *callback) {

    this->output_callbacks.add(callback);
    return;
}


void IVP_BetterStatisticsmanager::remove_output_callback(IVP_BetterStatisticsmanager_Callback_Interface *callback) {

    if ( this->output_callbacks.index_of(callback) == -1 ) {
	return;
    }
    
    this->output_callbacks.remove(callback);
    return;
}


void IVP_BetterStatisticsmanager::enable() {

    this->enabled = IVP_TRUE;

    return;
}


void IVP_BetterStatisticsmanager::disable() {

    this->enabled = IVP_FALSE;

    return;
}


IVP_BOOL IVP_BetterStatisticsmanager::get_state() {

    return(this->enabled);
}


IVP_BetterStatisticsmanager::IVP_BetterStatisticsmanager() {

    this->enabled = IVP_TRUE;
    this->update_interval = 1.0;
    this->update_delayed = IVP_TRUE;

    return;
}


IVP_BetterStatisticsmanager::~IVP_BetterStatisticsmanager() {

    return;
}


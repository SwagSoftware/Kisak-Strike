// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.



#include <ivp_physics.hxx>

#include <stdarg.h>

#include <ivp_betterdebugmanager.hxx>


void IVP_BetterDebugmanager::enable_debug_output(IVP_DEBUG_CLASS class_id) {

    if (class_id >= IVP_DEBUG_MAX_N_CLASSES ) {
	return;
    }
    this->flag_list[class_id] = 1;

    return;
}

void IVP_BetterDebugmanager::disable_debug_output(IVP_DEBUG_CLASS class_id) {

    if (class_id >= IVP_DEBUG_MAX_N_CLASSES ) {
	return;
    }
    this->flag_list[class_id] = 0;

    return;
}


IVP_BOOL IVP_BetterDebugmanager::is_debug_enabled(IVP_DEBUG_CLASS class_id) {

    if (class_id >= IVP_DEBUG_MAX_N_CLASSES ) {
	return(IVP_FALSE);
    }
    if ( this->flag_list[class_id] == 0 || !this->initialized ) {
	return(IVP_FALSE);
    }
    return(IVP_TRUE);
}


void IVP_BetterDebugmanager::dprint(IVP_DEBUG_CLASS class_id, const  char *formatstring, ...) {

    char buffer[4096];
    va_list args;

    va_start(args, formatstring);	
    vsprintf(buffer, formatstring, args);
    va_end(args);

    this->output_function(class_id, buffer);

    return;
}


void IVP_BetterDebugmanager::output_function(IVP_DEBUG_CLASS , const char *string) {

    printf("%s", string);
    return;
}


IVP_BetterDebugmanager::IVP_BetterDebugmanager() {

    int x;
    for (x=0; x<IVP_DEBUG_MAX_N_CLASSES; x++) {
	this->flag_list[x] = 0;
    }
    this->initialized = 1;

#ifdef DEBUG
    //this->enable_debug_output(IVP_DM_SURBUILD_POINTSOUP);
    //this->enable_debug_output(IVP_DM_SURBUILD_HALFSPACESOUP);
    //this->enable_debug_output(IVP_DM_SURBUILD_Q12);
    //this->enable_debug_output(IVP_DM_CONVEX_DECOMPOSITOR);
    //this->enable_debug_output(IVP_DM_QHULL);
    //this->enable_debug_output(IVP_DM_GEOMPACK_LEVEL1);
    //this->enable_debug_output(IVP_DM_GEOMPACK_LEVEL2);
    //this->enable_debug_output(IVP_DM_GEOMPACK_LEVEL3);
    this->enable_debug_output(IVP_DM_CLUSTERING_SHORTRANGE_VISUALIZER);
    this->enable_debug_output(IVP_DEBUG_IPION_ERROR_MSG);
#endif

    return;
}


IVP_BetterDebugmanager::~IVP_BetterDebugmanager() {

    return;
}

IVP_BetterDebugmanager ivp_debugmanager;

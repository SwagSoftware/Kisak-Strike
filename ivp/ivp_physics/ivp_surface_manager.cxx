// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#ifndef WIN32
#	pragma implementation "ivp_surface_manager.hxx"
#endif

#include <ivp_physics.hxx>
#include <ivp_compact_ledge.hxx>
#include <ivp_surface_manager.hxx>



IVP_SurfaceManager::~IVP_SurfaceManager()
{
    return;
}


    /******************************************************************************
     *  Method:		set/get ledge_client_data
     *  Description:	Allows the user to set and get a ledge specific client_data
     *	Note:		Range of values [0..IVP_SURMAN_POLYGON_MAX_VALUE_CLIENTDATA]
     *****************************************************************************/
void IVP_SurfaceManager::set_ledge_specific_client_data( IVP_Compact_Ledge *cl, unsigned int value){
    // assert value < IVP_SURMAN_POLYGON_MAX_VALUE_CLIENTDATA
    cl->set_client_data(value);
}

unsigned int IVP_SurfaceManager::get_ledge_specific_client_data( const IVP_Compact_Ledge *cl){
    return cl->get_client_data();
}

// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

/********************************************************************************
 *	File:	       	ivp_listener_hull.hxx	
 *	Description:	???
 ********************************************************************************/

#ifndef _IVP_LISTENER_HULL_INCLUDED
#define _IVP_LISTENER_HULL_INCLUDED

#ifndef WIN32
#	pragma interface
#endif


class IVP_Hull_Manager;


enum IVP_HULL_ELEM_TYPE {
    IVP_HULL_ELEM_POLYGON,
    IVP_HULL_ELEM_ANCHOR,
    IVP_HULL_ELEM_OO_WATCHER,
    IVP_HULL_ELEM_OO_CONNECTOR
};

/********************************************************************************
 *	Name:	  	IVP_Listener_Hull    	
 *	Description:	super-class for IVP_Synapse and IVP_Anchor
 ********************************************************************************/
class IVP_Listener_Hull {

public:
    virtual IVP_HULL_ELEM_TYPE get_type()=0;
    virtual void hull_limit_exceeded_event(IVP_Hull_Manager *, IVP_HTIME hull_intrusion_value) = 0;
    virtual void hull_manager_is_going_to_be_deleted_event(IVP_Hull_Manager *) = 0;
            // implementations have to remove themselves from hull manager
    virtual void hull_manager_is_reset(IVP_FLOAT dt, IVP_FLOAT center_dt);  // hull_manager is reset
private:
    friend class IVP_Hull_Manager;
    unsigned int minlist_index;
};

#endif

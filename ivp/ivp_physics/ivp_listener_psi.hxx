// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef IVP_LISTENER_PSI_INCLUDED
#define IVP_LISTENER_PSI_INCLUDED

/********************************************************************************
 *	Name:	       	IVP_Event_PSI
 *	Description:	information given to the IVP_Listener_PSI callback
 ********************************************************************************/
class IVP_Event_PSI {
public:
    IVP_Environment	*environment;		// note: call environment->get_current_time() to get time
};

// 
/********************************************************************************
 *	Name:	    	IVP_Listener_PSI   	
 *	Description:	A callback called every psi
 *			Using this callback, an application may change the speed of objects
 *			by pushing them.
 *	Note:		event collision may remove itself from listener list
 ********************************************************************************/
class IVP_Listener_PSI {
public:
    virtual void event_PSI( IVP_Event_PSI *)=0;
    virtual void environment_will_be_deleted(IVP_Environment *) = 0;
};

#endif

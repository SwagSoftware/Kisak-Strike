// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef _IVP_LISTENER_OBJECT_INCLUDED
#define _IVP_LISTENER_OBJECT_INCLUDED

#ifndef WIN32
#	pragma interface
#endif


class IVP_Environment;
class IVP_Real_Object;

class IVP_Event_Object
{
public:
    IVP_Environment *environment; // call environment->get_current_time() to get current time
    IVP_Real_Object *real_object;   
};

class IVP_Listener_Object
{
public:
    virtual void event_object_deleted( IVP_Event_Object *)=0;
    virtual void event_object_created( IVP_Event_Object *)=0;
    virtual void event_object_revived( IVP_Event_Object *)=0;
    virtual void event_object_frozen ( IVP_Event_Object *)=0;

	virtual ~IVP_Listener_Object() {};
};

#endif

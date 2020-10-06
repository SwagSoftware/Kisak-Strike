// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


// IVP_EXPORT_PUBLIC

#ifndef IVP_TIME_EVENT_INCLUDED
#define IVP_TIME_EVENT_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

class IVP_Environment;

class IVP_Time_Event {
public:
    int index;    
    IVP_Time_Event(){;};
    virtual void simulate_time_event(IVP_Environment *){ CORE; }; // to be implemented by application
};

#endif

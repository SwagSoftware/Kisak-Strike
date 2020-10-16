// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


// IVP_EXPORT_PROTECTED

#ifndef _IVP_TIME_INCLUDED
#define _IVP_TIME_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

class IVP_U_Min_List;
class IVP_Environment;
class IVP_Mindist;

#ifndef IVP_TIME_EVENT_INCLUDED
#    include <ivp_time_event.hxx>
#endif

// spezielle Auspraegungen von IVP_Time_Event

class IVP_Time_Event_PSI : public IVP_Time_Event
{
public:
    IVP_Time_Event_PSI(){;};
    void simulate_time_event(IVP_Environment *env);
};

class IVP_Time_Event_D : public IVP_Time_Event
{
public:    
    int number_of_sons;
    void simulate_time_event(IVP_Environment *env);
    IVP_Time_Event_D(IVP_Time time);
};

class IVP_Time_Event_N : public IVP_Time_Event {
public:    
    void simulate_time_event(IVP_Environment *env);
    IVP_Time_Event_N(IVP_Time time);
};

class IVP_Event_Manager {
public:
	int mode;
	virtual void simulate_time_events(IVP_Time_Manager *tman,IVP_Environment *env,IVP_Time until) = 0;
	virtual void simulate_variable_time_step(IVP_Time_Manager *tman,IVP_Environment *env,IVP_Time_Event_PSI *psi_event, IVP_FLOAT delta);
};

class IVP_Event_Manager_Standard : public IVP_Event_Manager {
	virtual void simulate_time_events(IVP_Time_Manager *tman,IVP_Environment *env,IVP_Time until);
};

class IVP_Event_Manager_D : public IVP_Event_Manager {
	virtual void simulate_time_events(IVP_Time_Manager *tman,IVP_Environment *env,IVP_Time untilt);
};

// Time Manager

//void time_man_sim_until(IVP_Time_Manager *tman,IVP_Environment *env, IVP_Time time);
//void time_man_sim_until_d(IVP_Time_Manager *tman,IVP_Environment *env, IVP_Time time);

class IVP_Time_Manager
{
//private: // lwss change this to public.
public:
    int n_events; // num of events currently managed
    IVP_Time_Event *get_next_event(); // and remove event afterwards
    IVP_Time_Event *get_next_event(IVP_Time time); //get event until
public:
    IVP_Event_Manager *event_manager;
    IVP_U_Min_List *min_hash;
	IVP_Time_Event_PSI *psi_event;

    IVP_DOUBLE last_time; // relative basetime
    IVP_Time base_time; // value of elements are relativ to this
    // construct
    IVP_Time_Manager();
    
    void insert_event (IVP_Time_Event *event, IVP_Time time);
    void remove_event(IVP_Time_Event *event);
    void update_event(IVP_Time_Event *event, IVP_Time time);

    int get_event_count(); // num of events currently managed by IVP_Time_Manager
    void event_loop(IVP_Environment *env, IVP_Time time); 

	void simulate_variable_time_step( IVP_Environment *, IVP_FLOAT delta_time);
    void env_set_current_time(IVP_Environment *env, IVP_Time time);
    void reset_time( IVP_Time offset );
  
    // destruct
    ~IVP_Time_Manager();
};

#endif

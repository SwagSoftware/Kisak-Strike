// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef IVP_ANOMALY_MANAGER_INCLUDED
#define IVP_ANOMALY_MANAGER_INCLUDED

//IVP_EXPORT_PUBLIC

#ifndef WIN32
#	pragma interface
#endif

class IVP_Mindist;

/********************************************************************************
 *	Name:	       	IVP_Anomaly_Limits
 *	Description:	used to identify special situations occuring during
 *			the simulation
 ********************************************************************************/
class IVP_Anomaly_Limits {
    IVP_BOOL delete_this_if_env_is_deleted;
public:

    IVP_FLOAT max_velocity;
    int       max_collisions_per_psi;
    IVP_FLOAT max_angular_velocity_per_psi;

    /********************************************************************************
     *	Name:	       	IVP_Anomaly_Manager
     *	Description:	used to identify and solve special situations occuring during
     *			the simulation
     ********************************************************************************/
    inline IVP_FLOAT get_max_velocity() const { return max_velocity; }; // [m/s]**2
    inline IVP_FLOAT get_max_angular_velocity_per_psi() const { return max_angular_velocity_per_psi; }; // [radians/psi]**2
    inline int get_max_collisions_per_psi(){ return max_collisions_per_psi; };  // per core

    virtual void environment_will_be_deleted(IVP_Environment *);

    IVP_Anomaly_Limits(IVP_BOOL delete_this_if_env_is_deleted = IVP_TRUE);
    virtual ~IVP_Anomaly_Limits();
};

/********************************************************************************
 *	Name:	       	IVP_Anomaly_Manager
 *	Description:	used to solve special situations occuring during
 *			the simulation
 ********************************************************************************/
class IVP_Anomaly_Manager {
    IVP_BOOL delete_this_if_env_is_deleted;
public:
    /********************************************************************************
     *	Name:	       	IVP_Anomaly_Manager
     *	Description:	used to identify and solve special situations occuring during
     *			the simulation
     ********************************************************************************/
    
    virtual void max_velocity_exceeded(IVP_Anomaly_Limits *, IVP_Core *, IVP_U_Float_Point *velocity_in_out);
    virtual void max_angular_velocity_exceeded( IVP_Anomaly_Limits *, IVP_Core *, IVP_U_Float_Point *angular_velocity_in_out);
    virtual void inter_penetration( IVP_Mindist *mindist,IVP_Real_Object *, IVP_Real_Object *);
    virtual IVP_BOOL max_collisions_exceeded_check_freezing(IVP_Anomaly_Limits *, IVP_Core *); // return true if object should be temp. freezed

    virtual void environment_will_be_deleted(IVP_Environment *);

	// to solve penetration we push objects with 2* force of gravity
	// there might be situation in that this is not necessary 
	//  -> derive from IVP_Anomaly_Manager and have different implementation
    virtual IVP_FLOAT get_push_speed_penetration( IVP_Real_Object *, IVP_Real_Object *);

	void solve_inter_penetration_simple( IVP_Real_Object *, IVP_Real_Object *); //push mass centers in opposite directions

    IVP_Anomaly_Manager(IVP_BOOL delete_this_if_env_is_deleted = IVP_TRUE);
    virtual ~IVP_Anomaly_Manager();
};

#endif

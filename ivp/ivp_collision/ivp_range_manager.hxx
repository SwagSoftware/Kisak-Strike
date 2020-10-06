// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

class IVP_Range_Manager {
    IVP_BOOL bound_to_environment;
public:
    IVP_Environment *environment;

	// object vs object
    IVP_DOUBLE look_ahead_time_intra; 			// [s]
    IVP_DOUBLE look_ahead_max_radius_intra;   	// object radius factor
    IVP_DOUBLE look_ahead_min_distance_intra;	// [m]
    IVP_DOUBLE look_ahead_max_distance_intra;	// [m]
    IVP_DOUBLE look_ahead_min_seconds_intra; 	// [s] real minimum

	// objects vs world
    IVP_DOUBLE look_ahead_time_world; 			// see above
    IVP_DOUBLE look_ahead_max_radius_world;   	// object radius factor
    IVP_DOUBLE look_ahead_min_distance_world;	// [m]
    IVP_DOUBLE look_ahead_max_distance_world;	// distances
    IVP_DOUBLE look_ahead_min_seconds_world; 	// [s] real minimum

    // calculate the range for mindists
    virtual void get_coll_range_intra_objects( const IVP_Real_Object *obj0,const IVP_Real_Object *obj1,
			    IVP_DOUBLE *range0, IVP_DOUBLE *range1 );

    // calculate range for single object
    virtual IVP_DOUBLE get_coll_range_in_world( const IVP_Real_Object *obj);
    
    IVP_Range_Manager(IVP_Environment *env, IVP_BOOL delete_this_on_env_delete);
    virtual void environment_will_be_deleted(IVP_Environment *);
};






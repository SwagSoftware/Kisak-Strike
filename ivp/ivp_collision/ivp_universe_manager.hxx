// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

class IVP_Universe_Manager;


/********************************************************************************
 *	Class:	    	IVP_Universe_Manager
 *	Idea:		Each dynamic object in the IVP_Environment will dynamically
 *			request objects around it so that collisions may be detected
 *			and resolved. The IVP_Environment will pass a spherical
 *			radius within which all non moving objects should be in the
 *		        simulation.
 *	Description:	This class is responsible for adding (normally non-moving) objects
 *			into the IVP_Environment. The IVP_Environment will call
 *			IVP_Universe_Manger::ensure_objects_in_environment(
 *			IVP_U_Point *sphere_center, IVP_DOUBLE sphere_radius).
 *			Each IVP_Real_Object has a get_collision_check_reference_count(),
 *			which returns the number of objects which are checking this
 *			object for collision. If -1 is returned than there is no collision detection
 *			enabled for this object
 *			Or 
 *	Note:		The implementation of ensure_objects_in_environment should be
 *			fairly fast.
 *			Only objects which can collide the checked object have to 
 *			be thrown into the IVP_Environment
 ********************************************************************************/

class IVP_Universe_Manager_Settings{
public:
    /* first level check for unused not referenced objects */
    int num_objects_in_environment_threshold_0;  // if less objects exist, physics will not call object_no_longer_needed
    int check_objects_per_second_threshold_0;    // physics will only check this number of objects per second

    /* second level check for unused not referenced objects */
    /* assert( num_objects_in_environment_threshold_1 > num_objects_in_environment_threshold_0) */
    int num_objects_in_environment_threshold_1;  // if less objects exist, physics will use check_objects_per_second_threshold_0, otherwise check_objects_per_second_threshold_1
    int check_objects_per_second_threshold_1;

    IVP_Universe_Manager_Settings();	// sets some default values
};


class IVP_Universe_Manager {
private:
public:
    virtual void ensure_objects_in_environment(IVP_Real_Object *object, 
					       IVP_U_Float_Point *sphere_center,
					       IVP_DOUBLE sphere_radius) ;
    
    virtual void object_no_longer_needed(IVP_Real_Object *object) ;

    virtual void event_object_deleted(IVP_Real_Object *object) ;  // object will be deleted

    // called by the IVP_Environment to get the information when it should
    // send object_no_longer_needed events
    virtual const IVP_Universe_Manager_Settings *provide_universe_settings();
    IVP_Universe_Manager(void) { ; };
};

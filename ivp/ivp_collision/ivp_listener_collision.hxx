// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef WIN32
#	pragma interface
#endif

#ifndef _IVP_CONTACT_SITUATION_INCLUDED
#	include <ivp_contact_situation.hxx>
#endif

#include <ivp_friction.hxx>

class IVP_Environment;

class IVP_Contact_Point;      // a kind a (void *) pointer to identify friction points

/********************************************************************************
 *	Name:	       	IVP_Contact_Point_API
 *	Description:	Fast and shielded access to internal friction values
 ********************************************************************************/
class IVP_Contact_Point_API {
public:
    /********************************************************************************
     *	Name:	       	get_eliminated_energy
     *	Description:	returns the integrated kinetic energy, which is destroyed by this
     *			friction handle. The value may jitter, as the PSI and frame rate
     *			may differ
     ********************************************************************************/
    static IVP_FLOAT get_eliminated_energy(IVP_Contact_Point *friction_handle);

    /********************************************************************************
     *	Name:	       	reset_eliminated_energy
     *	Description:	the integrated destroyed energy is stored only as a float
     *			So from time to time you have to reset this value in order
     *			not to loose precision.
     ********************************************************************************/
    static void  reset_eliminated_energy(IVP_Contact_Point *friction_handle);

    /********************************************************************************
     *	Name:	       	get_vert_force
     *	Description:	returns the vertical force between the two involved objects
     *			at the point of contact
     ********************************************************************************/
    static IVP_FLOAT get_vert_force(IVP_Contact_Point *friction_handle);

    //lwss- add this missing func
    static void get_surface_normal_ws(IVP_Contact_Point *friction_handle, IVP_U_Float_Point *normal )
    {
        return friction_handle->get_contact_normal( normal );
    }
};



/********************************************************************************
 *	Name:	       	IVP_Event_Collision
 *	Description:	Condensed information about a collision
 ********************************************************************************/
class IVP_Event_Collision {
public:
	// diff in sec when same objs collided last 
    IVP_FLOAT		    d_time_since_last_collision;

    IVP_Environment	    *environment;

	// call environment->get_current_time() to get current time
    IVP_Contact_Situation   *contact_situation;
};

class IVP_Event_Friction {
public:
	// call environment->get_current_time() to get current time
    IVP_Environment* environment;

    IVP_Contact_Situation* contact_situation;

	// can be used to identify friction contact point
    IVP_Contact_Point* friction_handle;
};

enum IVP_LISTENER_COLLISION_CALLBACKS {
	IVP_LISTENER_COLLISION_CALLBACK_POST_COLLISION  = 0x01,
	IVP_LISTENER_COLLISION_CALLBACK_OBJECT_DELETED    = 0x02,
	IVP_LISTENER_COLLISION_CALLBACK_FRICTION   = 0x04,
	IVP_LISTENER_COLLISION_CALLBACK_PRE_COLLISION = 0x08
};

/********************************************************************************
 *	Name:	       	IVP_Listener_Collision
 *	Description:	callback for collision events,
 *			The user application should override these methods to
 *			their custom needs. 
 *
 *			see IVP_Environment::add_listener_collision
 *			or  IVP_Real_Object::add_listener_collision
 *	Note:		event_collision and event_friction_xxx 
 *			may remove itself from listener list 
 ********************************************************************************/
class IVP_Listener_Collision {
	int enabled_callbacks;	    // an or-ed IVP_LISTENER_COLLISION_CALLBACKS list
public:
	int get_enabled_callbacks()
	{ 
		return enabled_callbacks;
	}

	// collision: set the IVP_LISTENER_COLLISION_CALLBACK_COLLISION bit in the constructor 
	// if you want to use the next callback.

	// the user app sould override this
	// called just before a collision/contact is solved
	virtual void event_pre_collision(IVP_Event_Collision*);

	// the user app sould override this
	// called just after the collision/contact is solved
	virtual void event_post_collision(IVP_Event_Collision*);

	// only object private callbacks are called	
	virtual void event_collision_object_deleted(class IVP_Real_Object*);
	
	// friction: set the IVP_LISTENER_COLLISION_CALLBACK_FRICTION bit in the constructor 
	// if you want to use the next callback.

	// the user app sould override this
	virtual void event_friction_created(IVP_Event_Friction*);

	// the user app sould override this
	virtual void event_friction_deleted(IVP_Event_Friction*);
	// warning: not all data in IVP_Event_Friction is valid
	//          (like surf_normal)

	virtual void event_friction_pair_created( class IVP_Friction_Core_Pair *pair ) {}

	// the user app sould override this
	virtual void event_friction_pair_deleted( class IVP_Friction_Core_Pair *pair ) {}

	// constructor
	IVP_Listener_Collision(int /* IVP_LISTENER_COLLISION_CALLBACKS */ enable_callbacks = 1){
		enabled_callbacks = enable_callbacks;	// set flags for implemented callbacks
	};

	// virtual destructor: @@CB
	virtual ~IVP_Listener_Collision() {};
};



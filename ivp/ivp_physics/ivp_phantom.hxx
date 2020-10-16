// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef _IVP_PHANTOM_INCLUDED
#define _IVP_PHANTOM_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

#ifndef IVP_SET_INCLUDED
#	include <ivu_set.hxx>
#endif

class IVP_Mindist_Base;

/*********************************************************************
 *	Name:	    	IVP_Template_Phantom  	
 *	Description:	Parameters needed to construct a phantom
 *                      A phantom is just another name for an object
 *                      which does not collide but keeps track of all
 *                      objects intruding it
 *********************************************************************/
class IVP_Template_Phantom {
public:

  /*********************************************************************
   *	Name:	    	manage_intruding_objects  	
   *	Default:	IVP_FALSE
   *	Description:    if set to IVP_TRUE, phantom manages a set of intruding objects,
   *                    see IVP_Controller_Phantom::get_intruding_objects()
   *********************************************************************/
  IVP_BOOL manage_intruding_objects;         // default IVP_FALSE, if set to IVP_TRUE, phantom manages a set of intruding objects

  /*********************************************************************
   *	Name:	    	manage_intruding_cores  	
   *	Default:	IVP_FALSE
   *	Description:    if set to IVP_TRUE, phantom manages a set of intruding cores of objects,
   *                    see IVP_Controller_Phantom::get_intruding_cores()
   *********************************************************************/
  IVP_BOOL manage_intruding_cores;         // default IVP_FALSE, if set to IVP_TRUE, phantom manages a set of intruding objects

  //lwss add
  IVP_BOOL manage_sleeping_cores;
  //lwss end
    
  /*********************************************************************
   *	Name:	    	dont_check_for_unmoveables  	
   *	Default:	IVP_FALSE
   *	Description:    if set to IVP_TRUE, only moving objects are checked
   *********************************************************************/
  IVP_BOOL dont_check_for_unmoveables;
  
  /*********************************************************************
   *	Name:	    	exit_policy_extra_radius  	
   *	Default:	0.5f[m]
   *	Description:    This is an extra radius around each phantom.
   *			the callback object_left_volume() is only called if the object
   *			leaves the phantom plus this extra radius.
   *    Note:           Set this value to higher values to speed up the collision detection engine
   *********************************************************************/
  IVP_FLOAT exit_policy_extra_radius;

  /*********************************************************************
   *	Name:	    	exit_policy_extra_time  	
   *	Default:	0.5f[sec]
   *	Description:    This is the time interval used by the collision detection engine
   *                    to check for intruded objects to leave the phantom volume
   *    Note:           Set this value to higher values to speed up the collision detection engine
   *    Version Info:   Not implemented yet
   *********************************************************************/
  IVP_FLOAT exit_policy_extra_time;   // def 0.5f[s]
  
  IVP_Template_Phantom();
};

/*********************************************************************
 *	Name:	    	IVP_Listener_Phantom  	
 *	Description:	Listens to phantom events
 *********************************************************************/
class IVP_Listener_Phantom {
public:
  virtual void mindist_entered_volume(class IVP_Controller_Phantom *controller,class IVP_Mindist_Base *mindist)=0;
  virtual void mindist_left_volume(class IVP_Controller_Phantom *controller, class IVP_Mindist_Base *mindist)=0;
  virtual void core_entered_volume(class IVP_Controller_Phantom *controller,class IVP_Core *pCore)=0;
  virtual void core_left_volume(class IVP_Controller_Phantom *controller, class IVP_Core *pCore)=0;
  virtual void phantom_is_going_to_be_deleted_event(class IVP_Controller_Phantom *controller)=0;  // indicates either object is deleted or phantom is converted to object
  IVP_Listener_Phantom(){;};
};


/*********************************************************************
 *	Name:	    	IVP_Controller_Phantom  	
 *	Description:	Manages and stores parameters used by the 
 *			collision detection engine
 *	Note:		To access the IVP_Controller_Phantom call
 *			IVP_Real_Object::get_controller_phantom()
 *********************************************************************/
class IVP_Controller_Phantom {
  friend class IVP_Mindist_Manager;
protected:
  IVP_Real_Object *object;
  IVP_FLOAT    exit_policy_extra_radius;
  IVP_U_Vector<IVP_Listener_Phantom> listeners;
  IVP_U_Set_Active<IVP_Mindist_Base> set_of_mindists;
  IVP_U_Set_Active<IVP_Real_Object> *set_of_objects;   // optional set of objects
    IVP_VHash_Store *mindist_object_counter;
    IVP_VHash_Store *mindist_core_counter;
  IVP_U_Set_Active<IVP_Core> *set_of_cores;   // optional set of objects
  IVP_Time time_of_last_set_transformation;

  friend class IVP_Real_Object;
  IVP_Controller_Phantom(IVP_Real_Object *object, const IVP_Template_Phantom *templat); // use IVP_Real_Object::convert_to_phantom to create phantoms

  // functions called by the collision detection engine
  void mindist_entered_volume(class IVP_Mindist *mindist);
  void mindist_left_volume(class IVP_Mindist *mindist);
public:
  
  IVP_U_Set_Active<IVP_Real_Object> *get_intruding_objects() const { return set_of_objects ; };  // returns NULL if manage_set was IVP_FALSE 
  IVP_U_Set_Active<IVP_Core> *get_intruding_cores() const { return set_of_cores ; };  // returns NULL if manage_set was IVP_FALSE 
  IVP_U_Set_Active<IVP_Mindist_Base> *get_intruding_mindists(){ return &set_of_mindists ; };             // returns mindist
  IVP_Real_Object *get_object() const { return object; };
  
  void add_listener_phantom( IVP_Listener_Phantom *listener);
  void remove_listener_phantom( IVP_Listener_Phantom *listener);
  
  ~IVP_Controller_Phantom();   // Note: Has side effect, switches phantom object to real object

  //lwss add
  void wake_all_sleeping_objects();
  //lwss end

  void *client_data; // for use by game code
};



#endif

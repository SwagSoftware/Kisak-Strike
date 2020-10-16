// Copyright (C) Ipion Software GmbH 2000. All rights reserved.

//IVP_EXPORT_PUBLIC
#ifndef _IVP_COLLISION_INCLUDED
#define _IVP_COLLISION_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

class IVP_Compact_Ledge;
class IVP_Real_Object;
class IVP_Collision;

/********************************************************************************
 *	Name:	       	IVP_Collision_Delegator
 *	Description:	Super class of all collision delegators
 ********************************************************************************/
class IVP_Collision_Delegator {
public:
	/********************************************************************************
	 *	Name:	       	collision_is_going_to_be_deleted_event
	 *	Description:	for any reason a collision element is deleted,
	 *			a and b can also be calculated using the get_objects function
	 *			however it's much faster this way
	 ********************************************************************************/
  virtual void collision_is_going_to_be_deleted_event(class IVP_Collision *t) = 0;  // should remove t from its internal structures
  virtual ~IVP_Collision_Delegator(){;};
  // Note: in the destructor, it should not delete it's IVP_Collision children, but
  // call child->delegator_is_going_to_be_deleted_event(this)

  //@@CB
  virtual void change_spawned_mindist_count(int /*change*/) {;};
  virtual int get_spawned_mindist_count() {return -1;};
  //@@CB
};


/********************************************************************************
 *	Name:	       	IVP_Collision_Delegator_Root
 *	Description:	Super class of the root collision delegator,
 *			see IVP_Environment::add(/remove)_collision_delegator_root
 ********************************************************************************/
class IVP_Collision_Delegator_Root: public IVP_Collision_Delegator {
public:
  
	/********************************************************************************
	 *	Name:	       	object_is_removed_from_collision_detection
	 *	Description:	called if an object is taken out of collision detection.
	 ********************************************************************************/
  virtual void object_is_removed_from_collision_detection(IVP_Real_Object *) = 0;
  
	/********************************************************************************
	 *	Name:	       	delegate_collisions_for_object
	 *	Description:	create a subclass which checks collisions between the base_object and the colliding_element
	 *	Return:		NULL if type of objects cannot be handled
	 ********************************************************************************/
  virtual IVP_Collision *delegate_collisions_for_object(IVP_Real_Object *base_object, IVP_Real_Object *colliding_element)=0;

  virtual void environment_is_going_to_be_deleted_event(IVP_Environment *env) = 0;
};



/********************************************************************************
 *	Name:	       	IVP_Collision
 *	Description:	Super class of all collision checkers
 ********************************************************************************/
class IVP_Collision: public IVP_Time_Event {
public:
  IVP_Collision_Delegator *delegator;
    int fvector_index[2];                // for ov_tree backlink ( see IVP_Mindist::recheck_ov_element())
    int get_fvector_index(int i) const{ return fvector_index[i]; }
    void set_fvector_index(int old, int i){
	if (fvector_index[0] == old){
	    fvector_index[0] = i;
	} else {
	    IVP_ASSERT( fvector_index[1] == old);
	    fvector_index[1] = i;
	}
    }

  // to identify the collision
  virtual void get_objects( IVP_Real_Object *objects_out[2] ) = 0;
  virtual void get_ledges( const IVP_Compact_Ledge *ledges_out[2] ) = 0;

  virtual void delegator_is_going_to_be_deleted_event(IVP_Collision_Delegator *){
    P_DELETE_THIS(this);
  };
  virtual ~IVP_Collision(){;}; // implementations should call  delegator->collision_is_going_to_be_deleted_event(this);
  IVP_Collision( IVP_Collision_Delegator *d){ delegator = d; fvector_index[0] = fvector_index[1] = -1; };
};

#endif

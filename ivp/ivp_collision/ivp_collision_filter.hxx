// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_collision_filter.hxx
 *  Description:    This file provides you with a base class and two subclasses
 *		    for collision filters. Using them you can easily prevent
 *		    certain objects from physically colliding. Instead they will
 *		    simply pass through each other.
 *  Note:	    It is only possible to define one single collision filter
 *		    for each physical environment!
 *  Classes:	    IVP_Collision_Filter (base class)
 *		    IVP_Collision_Filter_Coll_Group_Ident
 *		    IVP_Collision_Filter_Exclusive_Pair
 ********************************************************************************/

#ifndef IVP_COLLISION_FILTER_INCLUDED
#define IVP_COLLISION_FILTER_INCLUDED


/********************************************************************************
 *  Class:	    IVP_Collision_Filter
 *  Description:    This is the base class for all collision filters. Simply
 *		    subclass it and implement your own filtering conditions
 *		    for certain object pairs.
 *  Note:	    It's recommended to heavily use collision filters to speed
 *		    up performance of the collision detection whenever you are
 *		    sure that certain moveable objects are definately unable
 *		    to collide (e.g. the wheels of a car)
 *******************************************************************************/

class IVP_Collision_Filter {
public:

    /******************************************************************************
     *  Method:		check_objects_for_collision_detection
     *  Description:    This method gets called for every pair of objects that
     *			is likely to collide within the next time step. It should
     *			return IVP_TRUE if collision between the two supplied
     *			objects is to be detected and processed
     *			accordingly.
     *	Input:		<object0> first collision partner
     *			<object1> second collision partner
     *	Output:		IVP_TRUE, if collision should be detected
     *			IVP_FALSE, if collision between objects shall be ignored
     *	Note:		Use this method to implement your own filter conditions.
     *****************************************************************************/
    virtual IVP_BOOL check_objects_for_collision_detection(IVP_Real_Object *object0, IVP_Real_Object *object1) = 0;

    /******************************************************************************
     *  Method:		environment_will_be_deleted
     *  Description:    This method gets called when deleting the physical
     *			environment. Use it to clean up any of your own personal
     *			filter-internal data.
     *	Input:		<environment> the soon-to-be-deleted environment
     *****************************************************************************/
    virtual void environment_will_be_deleted(IVP_Environment *environment) = 0;

    virtual ~IVP_Collision_Filter();
};


/********************************************************************************
 *  Class:	    IVP_Collision_Filter_Coll_Group_Ident
 *  Description:    This is a subclass of the base collision filter class
 *		    IVP_Collision_Filter (see above). It will filter out all
 *		    collisions between objects that share the same collision-
 *		    group-identification string.
 *  Note:	    You can set this coll_group_ident when creating the physical
 *		    object.
 *******************************************************************************/

class IVP_Collision_Filter_Coll_Group_Ident : public IVP_Collision_Filter {
private:
    // internal data
    IVP_BOOL delete_on_env_delete;

public:

    /******************************************************************************
     *  Method:		check_objects_for_collision_detection
     *  Description:    This method checks the coll_group_ident string of both
     *			supplied objects. If they are the same it will return
     *			IVP_FALSE and thus disable collision detection between
     *			those objects.
     *	Input:		<object0> first collision partner
     *			<object1> second collision partner
     *	Output:		IVP_TRUE, if collision should be detected
     *			IVP_FALSE, if collision between objects shall be ignored
     *****************************************************************************/
    IVP_BOOL check_objects_for_collision_detection(IVP_Real_Object *object0, IVP_Real_Object *object1);

    /******************************************************************************
     *  Method:		environment_will_be_deleted
     *  Description:    This method will delete the collision filter as soon as the
     *			corresponding environment gets deleted.
     *	Note:		You can avoid this by setting a special flag when calling
     *			the class constructor (see below).
     *****************************************************************************/
    virtual void environment_will_be_deleted(IVP_Environment *);

    /******************************************************************************
     *  Method:		constructor
     *	Input:		<delete_on_env_delete> set to IVP_TRUE, if you want the
     *			                       filter to be automatically destroyed
     *			                       when the corresponding environment
     *			                       gets deleted.
     *****************************************************************************/
    IVP_Collision_Filter_Coll_Group_Ident(IVP_BOOL delete_on_env_delete);

    ~IVP_Collision_Filter_Coll_Group_Ident();
};


class IVP_CFEP_Objectpair;
class IVP_CFEP_Hash;

/********************************************************************************
 *  Class:	    IVP_Collision_Filter_Exclusive_Pair
 *  Description:    This is a subclass of the base collision filter class
 *		    IVP_Collision_Filter (see above). It will filter out certain
 *		    (pre-defined) object pairs.
 *  Details:	    This filter keeps track of the object-pairs by using a fast
 *		    hash.
 *******************************************************************************/

class IVP_Collision_Filter_Exclusive_Pair : public IVP_Collision_Filter {
private:
    // internal methods & data
    IVP_CFEP_Hash *hash_table; // the "no-collision" hash list

    void generate_hash_entry(IVP_Real_Object *object0, IVP_Real_Object *object1, IVP_CFEP_Objectpair *entry);

public:

    /******************************************************************************
     *  Method:		disable_collision_between_objects
     *  Description:    Use this method to disable the collision detection for a
     *			certain pair of objects.
     *	Input:		<object0> first possible collision partner
     *			<object1> second possible collision partner
     *****************************************************************************/
    void disable_collision_between_objects(IVP_Real_Object *object0, IVP_Real_Object *object1);

    /******************************************************************************
     *  Method:		enable_collision_between_objects
     *  Description:    Use this method to enable again the collision detection for
     *			a certain pair of objects.
     *	Input:		<object0> first possible collision partner
     *			<object1> second possible collision partner
     *****************************************************************************/
    void enable_collision_between_objects(IVP_Real_Object *object0, IVP_Real_Object *object1);

    /******************************************************************************
     *  Method:		check_objects_for_collision_detection
     *  Description:    This method checks whether the supplied object pair is
     *			present in the filter's internal "no-collision" list. If
     *			the pair is found, the method will return IVP_FALSE and no
     *			collision occur.
     *	Input:		<object0> first collision partner
     *			<object1> second collision partner
     *	Output:		IVP_TRUE, if collision should be detected
     *			IVP_FALSE, if collision between objects shall be ignored
     *****************************************************************************/
    IVP_BOOL check_objects_for_collision_detection(IVP_Real_Object *object0, IVP_Real_Object *object1);

    /******************************************************************************
     *  Method:		environment_will_be_deleted
     *  Description:    This method will delete the collision filter as soon as the
     *			corresponding environment gets deleted.
     *****************************************************************************/
    void environment_will_be_deleted(IVP_Environment *);

    IVP_Collision_Filter_Exclusive_Pair();
    ~IVP_Collision_Filter_Exclusive_Pair();
};


/********************************************************************************
 *  Class:          IVP_Meta_Collision_Filter
 *  Description:    This is a subclass of the base collision filter class
 *                  IVP_Collision_Filter (see above). You can add multiple 
 *                  Collision_Filters and the meta collision filter will
 *                  filter out collisions that are filtered by any of them
 *******************************************************************************/
class IVP_Meta_Collision_Filter : public IVP_Collision_Filter {
private:
    IVP_BOOL delete_on_env_delete;
    IVP_U_Vector<IVP_Collision_Filter> filter_set;

public:
    void add_collision_filter(IVP_Collision_Filter *filter);
    void remove_collision_filter(IVP_Collision_Filter *filter);

    /******************************************************************************
     *  Method:         check_objects_for_collision_detection
     *  Description:    This method checks all collision_filters that are
     *                  in the set of collision_filters. If any collision filter
     *                  reports that collision should not take place, the meta
     *                  collision filter returns IVP_FALSE;
     *  Input:          <object0> first collision partner
     *                  <object1> second collision partner
     *  Output:         IVP_TRUE, if collision should be detected
     *                  IVP_FALSE, if collision between objects shall be ignored
     *****************************************************************************/
    IVP_BOOL check_objects_for_collision_detection(IVP_Real_Object *object0, IVP_Real_Object *object1);

    /******************************************************************************
     *  Method:         environment_will_be_deleted
     *  Description:    This method will delete the collision filter as soon as the
     *                  corresponding environment gets deleted.
     *  Note:           You can avoid this by setting a special flag when calling
     *                  the class constructor (see below).
     *****************************************************************************/
    virtual void environment_will_be_deleted(IVP_Environment *);

    /******************************************************************************
     *  Method:         constructor
     *  Input:          <delete_on_env_delete> set to IVP_TRUE, if you want the
     *                                         filter to be automatically destroyed
     *                                         when the corresponding environment
     *                                         gets deleted.
     *****************************************************************************/
    IVP_Meta_Collision_Filter(IVP_BOOL delete_on_env_delete);

    ~IVP_Meta_Collision_Filter();
};


#endif

// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

/********************************************************************************
 *	File:	       	ivp_object.hxx	
 *	Description:	???
 ********************************************************************************/

#ifndef _IVP_OBJECT_INCLUDED
#define _IVP_OBJECT_INCLUDED

#ifndef WIN32
#	pragma interface
#endif


class IVP_Environment;
class IVP_Template_Object;
class IVP_Template_Cluster;
class IVP_Polygon;
class IVP_Ball;
class IVP_Cluster;


/********************************************************************************
 *	Name:	       	IVP_OBJECT_TYPE
 *	Description:	Object types for collision detection.
 *	Note:		The type IVP_CLUSTER is not evaluated semantically, yet.
 ********************************************************************************/
enum IVP_OBJECT_TYPE {

    IVP_NONE,
    IVP_CLUSTER,		// a container for other objects
    IVP_POLYGON,		// a polyhedron
    IVP_BALL,			// a ball
    IVP_OBJECT
};


/************************************************************************************************
 *	Name:	    	IVP_Object   	
 *	Description:	An abstract object
 *	Note:		See IVP_Real_Object if you are looking for the physical side of an object
 ************************************************************************************************/
class IVP_Object {

    friend class IVP_Cluster;
    friend class IVP_Cluster_Manager;
    friend class IVP_Real_Object;
public:
    virtual ~IVP_Object();
private:
    void init(IVP_Environment *env);	// real constructor
    IVP_OBJECT_TYPE object_type;	// type of this object, e.g. used for correct casts...
protected:
    IVP_Object   *next_in_cluster;	// next object in this cluster
    IVP_Object   *prev_in_cluster;	
    IVP_Cluster  *father_cluster;	// the father cluster
    const char	 *name;			// the name of the object, helps debugging

    IVP_Object(IVP_Environment *env);		// necessary to create root cluster
    IVP_Object(IVP_Cluster *father, const IVP_Template_Object *templ);

    void assign_to_cluster(IVP_Cluster *cluster); // moves object to cluster
//    friend class IVP_Mindist;

    void set_type(IVP_OBJECT_TYPE type_in)		{ object_type = type_in; };
public:
    IVP_Environment *environment; 	// the environment to which this object belongs
    IVP_OBJECT_TYPE	get_type() const		{ return object_type; };

    const char		*get_name() const		{ return name; };
    IVP_Environment    	*get_environment() const    	{ return environment; };


    IVP_Polygon     *to_poly(){return (IVP_Polygon *)this;};	// cast
    IVP_Cluster     *to_cluster(){ IVP_ASSERT(get_type()==IVP_CLUSTER); return (IVP_Cluster *)this;};
    IVP_Real_Object *to_real(){ IVP_ASSERT(get_type()!=IVP_CLUSTER); return (IVP_Real_Object *)this;};
    IVP_Ball        *to_ball(){ IVP_ASSERT(get_type()==IVP_BALL); return (IVP_Ball *)this;};
};

/********************************************************************************
 *	Name:	  	IVP_Cluster    	
 *	Description:	An object used to group other objects.
 *	Attention:	Physical simulation may change the hierarchy of
 *    			objects without warning (so do not use this grouping).
 *			With physical simulation, grouping of objects in
 *			the physics engine does not make sense because all
 *			cluster matrices have to be 'I'.
 ********************************************************************************/
class IVP_Cluster : public IVP_Object {

    // INTERN_START
    friend class IVP_Cluster_Manager;
    friend class IVP_Mindist_Manager;
    // INTERN_END
protected:
    // INTERN_START
    friend class IVP_Object;
    friend class IVP_Real_Object;
    friend class IVP_Environment;
    // INTERN_END
    void add_object(IVP_Object *object);
    void remove_object(IVP_Object *object);
    IVP_Cluster(IVP_Environment *env);  // needed to create the root cluster
    IVP_Object *objects;		// linked list of children

public:
    IVP_Object *get_first_object_of_cluster() { return(objects); }
    IVP_Object *get_next_object_in_cluster(IVP_Object *object) { return(object->next_in_cluster); }

    IVP_Cluster(IVP_Cluster *father, IVP_Template_Cluster *templ);
    ~IVP_Cluster();
};


#endif

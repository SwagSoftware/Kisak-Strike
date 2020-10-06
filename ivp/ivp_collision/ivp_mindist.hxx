// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef _IVP_MINDIST_INCLUDED
#define _IVP_MINDIST_INCLUDED

#ifndef WIN32
#	pragma interface
#endif


class IVP_Mindist_Base;
class IVP_Compact_Edge;

#ifndef IVP_TIME_EVENT_INCLUDED
#    include <ivp_time_event.hxx>
#endif

#ifndef _IVP_LISTENER_HULL_INCLUDED
#	include <ivp_listener_hull.hxx>
#endif
#ifndef _IVP_COLLISION
#	include <ivp_collision.hxx>
#endif

//#define IVP_MINDIST_BEHAVIOUR_DEBUG	// do core as soon as possible


/* if IVP_HALFSPACE_OPTIMIZATION_ENABLED is defined, the mindist remembers a virtual plane
 * seperating both ledges. It tries to use that halfspace to optimize the hull */
#define IVP_HALFSPACE_OPTIMIZATION_ENABLED
//#define IVP_PHANTOM_FULL_COLLISION //@@CB - not default behaviour



/********************************************************************************
 *	Name:	       	IVP_SYNAPSE_POLYGON_STATUS
 *	Description:	How the synapse is connected to a geometry
 ********************************************************************************/
enum IVP_SYNAPSE_POLYGON_STATUS {
    IVP_ST_POINT = 0,
    IVP_ST_EDGE  = 1,
    IVP_ST_TRIANGLE =2,
    IVP_ST_BALL = 3,
    IVP_ST_MAX_LEGAL = 4,	// max legal status, should be 2**x
    IVP_ST_BACKSIDE = 5	        // unknown, intrusion
};



/********************************************************************************
 *	Name:	       	IVP_Synapse
 *	Description:	Synapses are attachments to objects
 ********************************************************************************/
class IVP_Synapse: public IVP_Listener_Hull {  // sizeof() == 32
public:
  IVP_Synapse 	 *next, *prev;		       // per object, only for exact/invalid synapses
  IVP_Real_Object  *l_obj;                     // back link to object
  const IVP_Compact_Edge 	*edge;		// Note: all balls share one dummy edge

protected:
    short mindist_offset;             // back link to my controlling mindist
    short status;                     // IVP_SYNAPSE_POLYGON_STATUS point, edge, tri, ball ....
public:
   
protected:
    // hull manager
  IVP_HULL_ELEM_TYPE get_type(){ return IVP_HULL_ELEM_POLYGON; };
  virtual void hull_limit_exceeded_event(IVP_Hull_Manager *hull_manager, IVP_HTIME hull_intrusion_value);
  virtual void hull_manager_is_going_to_be_deleted_event(IVP_Hull_Manager *hull_manager);
  virtual   void hull_manager_is_reset(IVP_FLOAT dt,IVP_FLOAT center_dt);
public:

    IVP_Real_Object *get_object(){ return l_obj; };
    IVP_SYNAPSE_POLYGON_STATUS get_status()const{ return (IVP_SYNAPSE_POLYGON_STATUS) status; };

    virtual ~IVP_Synapse(){;};			// dummy, do not call
    const IVP_Compact_Ledge *get_ledge() const;
    const IVP_Compact_Edge *get_edge() const { return edge; };

    IVP_Mindist_Base *get_synapse_mindist()const{ return  (IVP_Mindist_Base *)(mindist_offset + (char *)this);} ;
    void set_synapse_mindist( IVP_Mindist_Base *md ) { mindist_offset = ((char *)md) - (char *)this; };

    void init_synapse_real( IVP_Mindist_Base *min, IVP_Real_Object *object_to_link ){
	set_synapse_mindist(min);
	l_obj = object_to_link;
	IVP_IF(1){      next = prev = this;    }
    }
};

// flag telling how the mindist is linked
enum IVP_MINIMAL_DIST_STATUS {
    IVP_MD_UNINITIALIZED = 0,
    IVP_MD_INVALID = 2,		// invalid mindist, eg endless loop, collision
    IVP_MD_EXACT = 3,
    IVP_MD_HULL_RECURSIVE = 4,   // invalid recursive mindists which spawned childs
    IVP_MD_HULL = 5		// -> synapses = hull synapses
};

// result of last recalc_mindist
enum IVP_MINIMAL_DIST_RECALC_RESULT {
  IVP_MDRR_OK = 0,
  IVP_MDRR_INTRUSION = 1
};


// result of the last recalc_next_event
enum IVP_COLL_TYPE {	// if last 4 bits == 0 then collision
    IVP_COLL_NONE	=0x00,
    IVP_COLL_PP_COLL	=0x10,
    IVP_COLL_PP_PK	=0x11,

    IVP_COLL_PF_COLL	=0x20,
    IVP_COLL_PF_NPF	=0x21,
    // PF_PK nicht noetig, da Flaeche Objekt abschirmt

    IVP_COLL_PK_COLL	=0x30,
    IVP_COLL_PK_PF	=0x31,
    IVP_COLL_PK_KK	=0x32,
    IVP_COLL_PK_NOT_MORE_PARALLEL = 0x33,
    
    IVP_COLL_KK_COLL	=0x40,
    IVP_COLL_KK_PARALLEL=0x41,
    IVP_COLL_KK_PF	=0x42
};

// type of mindist
enum IVP_MINDIST_FUNCTION {
  IVP_MF_COLLISION = 0,
  IVP_MF_PHANTOM =1
};


class IVP_Mindist_Base : public IVP_Collision {
protected:
  IVP_COLL_TYPE coll_type:8;
  unsigned int synapse_sort_flag:2;
public:  // Read only public !!!!
  IVP_BOOL             is_in_phantom_set:2;   // mindist is in phantom set_of_mindists already
  IVP_MINDIST_FUNCTION mindist_function:2;                      // collision / phantom
  IVP_MINIMAL_DIST_RECALC_RESULT recalc_result:2;               // intrusion/ok
#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED    
  IVP_BOOL disable_halfspace_optimization:2;
#endif    
  IVP_MINIMAL_DIST_STATUS mindist_status:4;
    unsigned int coll_dist_selector:8;

  IVP_Synapse         synapse[2];
  IVP_FLOAT           sum_extra_radius;   // sum of extra radius around each object

  // calculated by IVP_Mindist::recalc_mindist
  IVP_FLOAT 	    len_numerator; 
#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED
    IVP_FLOAT             contact_dot_diff_center; // contact plane * ( core0->pos - core1->pos),
    IVP_DOUBLE            sum_angular_hull_time; // sum of center hull time, needs IVP_DOUBLE 
#endif    

  IVP_U_Float_Point contact_plane;	//  needed for halfspace optimization, normized surface normal, only valid if recalc_result == IVP_MDRR_OK  

        IVP_Synapse *get_mindist_synapse(int i)     { return &synapse[i]; };
  const IVP_Synapse *get_mindist_synapse(int i)const{ return &synapse[i]; };
  
  IVP_FLOAT get_length() const { return len_numerator; }; // distance, only valid if recalc_result == IVP_MDRR_OK

  // IVP_Collision implementations
  virtual void get_objects( IVP_Real_Object *objects_out[2] );
  virtual void get_ledges( const IVP_Compact_Ledge *ledges_out[2] );

  
  IVP_Mindist_Base(IVP_Collision_Delegator *del);
  virtual ~IVP_Mindist_Base(){;};
};


#endif














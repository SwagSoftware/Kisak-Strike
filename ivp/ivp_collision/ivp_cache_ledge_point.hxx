// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef _IVP_CACHE_LEDGE_POINT_INCLUDED
#define _IVP_CACHE_LEDGE_POINT_INCLUDED
#ifndef WIN32
#	pragma interface
#endif


class IVP_Compact_Poly_Point;
class IVP_Synapse_Ball;


class IVP_Cache_Ball {
public:
    IVP_Cache_Object 	*cache_object;		// backlink to matrix
    IVP_Ball 	*object;		// backlink to object
    union  {
	IVP_Synapse_Real *synapse; // used to identify synapses
    } tmp;
};

class IVP_Cache_Ledge_Point {
    friend class IVP_Cache_Object_Manager;
    friend class IVP_Real_Object;
    friend class IVP_Compact_Ledge_Solver;
    
    // temporary section
    const IVP_Compact_Poly_Point *compact_poly_points; // backlink to float[3] array
    const IVP_Compact_Ledge *compact_ledge;  
public:
    IVP_Cache_Object 	*clp_cache_object;		// backlink to matrix
    IVP_Real_Object 	*clp_object;		// backlink to object

    union  {
	IVP_Synapse_Real *synapse; // used to identify synapses
	IVP_Synapse_Friction *synapse_friction; // used to identify synapses
    } tmp;
private:
public:
    const IVP_Compact_Ledge *get_compact_ledge() { return this->compact_ledge; };
    IVP_Cache_Object *get_object_cache() { return this->clp_cache_object; };
    IVP_Core *get_core() { return this->clp_object->get_core(); };

    inline void init_cache_ledge_point(IVP_Real_Object *obj, const IVP_Compact_Ledge *ledge){
	compact_ledge = ledge;
	clp_cache_object = obj->get_cache_object();
	clp_object = obj;
	compact_poly_points = ledge->get_point_array();
    }
    
    void remove_reference(){
	clp_cache_object->remove_reference();
    }
    IVP_Cache_Ledge_Point(IVP_Real_Object *obj, const IVP_Compact_Ledge *ledge){
	this->init_cache_ledge_point(obj,ledge);
    }
    IVP_Cache_Ledge_Point(){};
};



#endif

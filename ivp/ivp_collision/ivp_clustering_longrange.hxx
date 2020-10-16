// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

typedef int IVP_OV_TIME_STAMP;

class IVP_OV_Node;



#ifndef _IVP_LISTENER_HULL_INCLUDED
#	include <ivp_listener_hull.hxx>
#endif
#ifndef IVP_FVECTOR_INCLUDED
#	include <ivu_fvector.hxx>
#endif
class IVP_Collision;

class IVP_OV_Element : public IVP_Listener_Hull {

protected:
    IVP_HULL_ELEM_TYPE get_type();
    virtual void hull_limit_exceeded_event(IVP_Hull_Manager *mgr, IVP_HTIME);
    virtual void hull_manager_is_going_to_be_deleted_event(IVP_Hull_Manager *mgr);
public:
    IVP_OV_Node *node;
    IVP_Hull_Manager *hull_manager;	// set if in hull manager
    IVP_U_Float_Point center;            // xyz // #+# check alignment
    IVP_FLOAT radius;
    IVP_Real_Object     *real_object;
    IVP_U_FVector<IVP_Collision> collision_fvector;	// all oo connectors used by this object

    void add_oo_collision(IVP_Collision *connector);
    void remove_oo_collision(IVP_Collision *connector);

    void add_to_hull_manager(IVP_Hull_Manager *hm, IVP_DOUBLE hull_time);
    
    IVP_OV_Element(IVP_Real_Object *obj);
    virtual ~IVP_OV_Element();
};



struct IVP_OV_Node_Data {
    int x; // coordinates to be interpreted as points on a grid with
    int y; // below stored 2^rasterlevel as raster distance
    int z; // -> (2^rasterlevel) * x/y/z = world coordinates of upper-left-front corner of cube
    int rasterlevel; // exponent for raster grid
    int sizelevel; // sizelevel = rasterlevel+1 -> size of cube is twice the
                   // grid's raster distance
};

class IVP_OV_Node {
    friend class IVP_OV_Tree_Manager;
    ~IVP_OV_Node();
public:
    struct IVP_OV_Node_Data data;

    IVP_OV_Node                  *parent;
    IVP_U_Vector<IVP_OV_Node>     children;
    IVP_U_Vector<IVP_OV_Element>  elements;

    IVP_OV_Node();
};


class IVP_ov_tree_hash;

/********************************************************************************
 *	Name:	    	IVP_OV_Tree_Manager  	
 *	Description:	Tree-like datastructure, used to store spheres (around objects)
 ********************************************************************************/
class IVP_OV_Tree_Manager
{
  friend class IVP_Ray_Solver;
  friend class IVP_Ray_Solver_Group;
  friend class IVP_Sphere_Solver;

    IVP_DOUBLE powerlist[81]; // 2 power -40   to  2 power 40
    
    IVP_OV_Node                   search_node;
    IVP_U_Vector<IVP_OV_Element> *collision_partners;
    
    IVP_ov_tree_hash  *hash_table;
    IVP_Environment   *environment;       // see IVP_Collision_Filter
    IVP_OV_Node       *root;

    int          log_base2(IVP_DOUBLE x) const;
    inline IVP_DOUBLE       power2(int x) const{
	return(this->powerlist[x+40]);
    }
    

    IVP_DOUBLE       calc_optimal_box(const IVP_OV_Element *element, IVP_DOUBLE min_radius, IVP_DOUBLE max_radius);
    IVP_BOOL     box_contains_box(const struct IVP_OV_Node_Data *master_data, const IVP_OV_Node *sub_node, int const rasterlevel_diff) const;
    IVP_BOOL     box_overlaps_with_box(const IVP_OV_Node *largenode, const IVP_OV_Node *smallnode, int const rasterlevel_diff) const;
    IVP_OV_Node *find_smallest_box(const IVP_OV_Node *master_node, const IVP_OV_Node *sub_node) const;
    void         connect_boxes(IVP_OV_Node *node, IVP_OV_Node *new_node);
    void         expand_tree(const IVP_OV_Node *new_node);
    void         collect_subbox_collision_partners(const IVP_OV_Element *center_element, const IVP_OV_Node *node);
    void         collect_collision_partners(const IVP_OV_Element *center_element, const IVP_OV_Node *masternode, const IVP_OV_Node *new_node);

    void get_luf_coordinates_ws   (const IVP_OV_Node *node, IVP_U_Float_Point *p, IVP_FLOAT *cubesize); // fills vars with left-upper-front corner's coordinates and cube's size
    void get_center_coordinates_ws(const IVP_OV_Node *node, IVP_U_Float_Point *p, IVP_FLOAT *cubesize); // fills vars with center's coordinates and cube's size

    IVP_OV_Node *cleanup_node(IVP_OV_Node *node);
    
public:
    IVP_OV_Tree_Manager();
    ~IVP_OV_Tree_Manager();

    IVP_DOUBLE insert_ov_element(IVP_OV_Element *element,
                            IVP_DOUBLE min_radius,
                            IVP_DOUBLE max_radius,
			    IVP_U_Vector<IVP_OV_Element> *colliding_balls); // pass NULL if you are not interested in collisions

    void remove_ov_element(IVP_OV_Element *element);
};






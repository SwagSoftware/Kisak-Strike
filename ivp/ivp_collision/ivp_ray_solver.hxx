// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC
#ifndef IVP_RAY_SOLVER_INCLUDED
#define IVP_RAY_SOLVER_INCLUDED


#ifndef WIN32
#	pragma interface
#endif


class IVP_Environment;
class IVP_Real_Object;
class IVP_Ball;
class IVP_Polygon;
class IVP_Compact_Surface;
class IVP_Compact_Ledge;
class IVP_Compact_Triangle;

class IVP_OV_Node;
class IVP_OV_Tree_Manager;
class IVP_Compact_Ledgetree_Node;

#if !defined(_IVP_U_MINHASH_INCLUDED)
#    include <ivu_min_hash.hxx>
#endif

/********************************************************************************
 *	Name:	     	IVP_Ray_Solver	
 *	Description:	
 ********************************************************************************/

enum IVP_RAY_SOLVER_FLAGS
{
  IVP_RAY_SOLVER_ALL = 0,
  IVP_RAY_SOLVER_IGNORE_PHANTOMS = 1,
  IVP_RAY_SOLVER_IGNORE_MOVINGS = 2,
  IVP_RAY_SOLVER_IGNORE_STATICS = 4
};

#define IVP_MAX_NUM_RAY_HITS 256



/********************************************************************************
 *	Name:	     	IVP_Ray_Hit	
 *	Description:	The result of a ray cast
 ********************************************************************************/
class IVP_Ray_Hit {
  // used to describe ray contact situation
public:
    IVP_U_Float_Point             hit_surface_direction_os; // not valid for ball objects! For balls, you can easily compute this direction at a later point of time.

    IVP_Real_Object *             hit_real_object;
    const IVP_Compact_Ledge *     hit_compact_ledge;
    const IVP_Compact_Triangle *  hit_compact_triangle;
    IVP_FLOAT                     hit_distance;
};



/********************************************************************************
 *	Name:	     	IVP_Ray_Solver_Template	
 *	Description:	The template needed to create a ray solver
 ********************************************************************************/
class IVP_Ray_Solver_Template {
public:
  IVP_U_Point ray_start_point;
  IVP_U_Float_Point ray_normized_direction;  
  IVP_FLOAT   ray_length;
  IVP_RAY_SOLVER_FLAGS ray_flags;
};



/********************************************************************************
 *	Name:	     	IVP_Ray_Hit_Listener	
 *	Description:	The listener, all hits made by the IVP_Ray_Solver_Os are
 *			sent too.
 *	Note:		Internal use only
 ********************************************************************************/
class IVP_Ray_Hit_Listener {
public:
    virtual void add_hit_object(IVP_Real_Object *object, const IVP_Compact_Ledge *compact_ledge, const IVP_Compact_Triangle *compact_triangle, IVP_DOUBLE hit_dist, IVP_U_Point *hit_sur_vec_os) = 0;
    virtual ~IVP_Ray_Hit_Listener(){;};
};

/********************************************************************************
 *	Name:	     	IVP_Ray_Solver_Os	
 *	Description:	A ray solver needed to cast a ray for one object
 *	Note:		all hits are sent to the father_ray_solver
 ********************************************************************************/
class IVP_Ray_Solver_Os {
//protected:
public: // lwss- hack this to public
    friend class IVP_SurfaceManager_Grid;
	friend class IVP_SurfaceManager_Mopp;
	friend class hkMoppLongRayVirtualMachine;
    IVP_U_Point ray_start_point;
    IVP_U_Float_Point ray_center_point;
    IVP_U_Point ray_end_point;
    IVP_U_Float_Point ray_direction;

    IVP_Ray_Hit_Listener *hit_listener;
    IVP_Real_Object *object;

    IVP_FLOAT ray_length;

    inline IVP_BOOL check_ray_against_sphere_os(const IVP_U_Float_Point *sphere_center_os,   IVP_FLOAT sphere_radius);
public:
    void check_ray_against_ledge_tree_node_os(const IVP_Compact_Ledgetree_Node *);
    void check_ray_against_compact_surface_os(const IVP_Compact_Surface *);
    IVP_BOOL check_ray_against_compact_ledge_os(const IVP_Compact_Ledge *ledge_to_compare);
    
    IVP_Ray_Solver_Os( class IVP_Ray_Solver *father_ray_solver, IVP_Real_Object *);
};


/********************************************************************************
 *	Name:	     	IVP_Ray_Solver	
 *	Description:	solve a ray
 *	Note:		Internal use only
 ********************************************************************************/
class IVP_Ray_Solver: public IVP_Ray_Hit_Listener {
public:
//protected:
    friend class IVP_Ray_Solver_Os;
    friend class IVP_Ray_Solver_Group;    

    // set with the constructor
    IVP_U_Point ray_start_point;
    IVP_U_Point ray_end_point;
    IVP_U_Float_Point ray_center_point;
    IVP_U_Float_Point ray_direction;

    IVP_FLOAT ray_length;
    IVP_RAY_SOLVER_FLAGS ray_flags;


    /* Insert hit info into class member 'output_min_hash'  */
    void check_ray_against_compact_ledge_os(const IVP_Compact_Ledge *compact_ledge_to_compare, IVP_Real_Object *object);
    inline IVP_BOOL check_ray_against_sphere(const IVP_U_Float_Point *sphere_center_ws,   IVP_FLOAT sphere_radius);

public:
    
    void check_ray_against_all_objects_in_sim(const IVP_Environment *environment);
		 //output_min_hash is filled with hit IVP_Real_Objects

    void check_ray_against_node(IVP_OV_Node *node, IVP_OV_Tree_Manager *ov_tree_man);
    
    void check_ray_against_ball(IVP_Ball *ball_to_compare);
    void check_ray_against_object(IVP_Real_Object *object_to_compare);

    // the following functions return IVP_TRUE if ray hits
    IVP_BOOL check_ray_against_cube(const IVP_U_Float_Point *luf_point,  const IVP_U_Float_Point *rlb_point);
    IVP_BOOL check_ray_against_square(IVP_FLOAT pos_dist, IVP_FLOAT pos_axis_len, const IVP_U_Float_Point *min_coords,
				      const IVP_U_Float_Point *max_coords,    int coord_0,   int coord_1);  

    IVP_Ray_Solver(const IVP_Ray_Solver_Template *templ); // inits class members

    ~IVP_Ray_Solver(){;};
};


/********************************************************************************
 *	Name:	       	IVP_Ray_Solver_Group
 *	Description:	Optimized raycaster for a group of short rays
 *	Example application: raycast for car wheels
 *	Note:		uses filter (e.g. IVP_RAY_SOLVER_FLAGS) of first ray
 ********************************************************************************/
class IVP_Ray_Solver_Group {
    IVP_U_Float_Point center_ws;
    IVP_FLOAT radius;
    
    int n_ray_solvers;
    IVP_Ray_Solver **ray_solvers;
    inline IVP_BOOL check_ray_group_against_sphere(const IVP_U_Float_Point *sphere_center_ws,   IVP_FLOAT sphere_radius);
public:
    void check_ray_group_against_object(IVP_Real_Object *object_to_compare);

    // next functions return IVP_TRUE if ray hits
    IVP_BOOL check_ray_group_against_cube(const IVP_U_Float_Point *cube_center_ws, IVP_FLOAT cube_size);

    void check_ray_group_against_node(IVP_OV_Node *node, IVP_OV_Tree_Manager *ov_tree_man);
    void check_ray_group_against_all_objects_in_sim(const IVP_Environment *environment);

    IVP_Ray_Solver_Group( int n_ray_solvers, IVP_Ray_Solver **ray_solvers );
};

/********************************************************************************
 *	Name:	       	IVP_Ray_Solver_Min_Hash
 *	Description:	Find all hits of the ray
 ********************************************************************************/
class IVP_Ray_Solver_Min_Hash: public IVP_Ray_Solver {
private:
    IVP_U_Min_Hash output_min_hash;    // holds results
    IVP_Ray_Hit hit_info[IVP_MAX_NUM_RAY_HITS];
    virtual void add_hit_object(IVP_Real_Object *object, const IVP_Compact_Ledge *compact_ledge, const IVP_Compact_Triangle *compact_triangle, IVP_DOUBLE hit_dist, IVP_U_Point *hit_sur_vec_os);
 public:

  IVP_U_Min_Hash *get_result_min_hash() { return &output_min_hash; };
  
  IVP_Ray_Solver_Min_Hash(const IVP_Ray_Solver_Template *templ): IVP_Ray_Solver(templ), output_min_hash(8){
      ; // inits class members
  }
  ~IVP_Ray_Solver_Min_Hash(){;};
};


/********************************************************************************
 *	Name:	       	IVP_Ray_Solver_Min_Hash
 *	Description:	Find shortest hit only
 ********************************************************************************/
class IVP_Ray_Solver_Min: public IVP_Ray_Solver {
 protected:
    IVP_FLOAT min_dist;
    IVP_Ray_Hit ray_hit;
 public:
    IVP_Ray_Hit *get_ray_hit(){
	if (min_dist == P_FLOAT_MAX) return NULL;
	return &ray_hit;
    };
    IVP_FLOAT get_ray_dist(){ return min_dist; };
    
    virtual void add_hit_object(IVP_Real_Object *object, const IVP_Compact_Ledge *compact_ledge, const IVP_Compact_Triangle *compact_triangle, IVP_DOUBLE hit_dist, IVP_U_Point *hit_sur_vec_os);

  
  IVP_Ray_Solver_Min(const IVP_Ray_Solver_Template *templ):IVP_Ray_Solver(templ){ // inits class members
      min_dist = P_FLOAT_MAX;
  }
    
  ~IVP_Ray_Solver_Min(){;};
};

#endif // IVP_INCLUDED

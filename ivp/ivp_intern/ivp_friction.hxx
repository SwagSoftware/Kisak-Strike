// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//Datastructures for friction systems:
//    Vector of all objects (IVP_Real_Object) belonging to fr sys, objects are counted: friction_obj_number
//        every object has Vector of IVP_Friction_Info_For_Obj for every friction system it belongs to (one object can have more then one fr sys,but now used only for fixed objects)
//            in one IVP_Friction_Info_For_Obj is Vector of IVP_Contact_Point and a backlink to friction system
//    Vector IVP_Friction_Core_Pair for every pair of objects that share mindists (for easing friction forces and speedup of union find)
//        every pair has backlinks to objects and Vector of IVP_Contact_Point that have that object pair as anchors
//    double linked list of all IVP_Contact_Point belonging to fr sys, dists are counted: friction_dist_number
//        IVP_Contact_Point has backlink to both objects (via snapses ...)
//        IVP_Contact_Point are numbered (but counted oppositely the way they occur in the list)
//    function test_hole_fr_system_data checks consistency of most of this infos 
#pragma once // lwss: these includes are hell

class IVP_Contact_Point;
class IVP_Friction_Solver_Long_Term;
class IVP_Mindist_Manager;
class IVP_Core;
class IVP_Friction_Info_For_Core;
class IV_U_Point_PCore;
class IVP_Environment;
class IVP_Cache_Ledge_Point;
class IVP_Compact_Edge;
class IVP_Mindist;

#define IVP_SLOWLY_TURN_ON_KEEPER 20

#define IVP_MAX_FRICTION_MATRIX 150
#define IVP_MAX_ADHESION_GAUSS 0.01f //in percent of weight

#define IVP_MINIMAL_REAL_FRICTION_LEN 0.005f
//#define NOFRICTIONFORCE
//#define NOEASING 
//#define NO_MUTUAL_ENERGYDESTROY
//#define INTEGRATE_ENERGY_DAMP 0.9f //leave 
#define MAX_ENERGY_DESTROY 0.1f
#define IVP_EASE_EVERY_NTH_PSI 5
//#define IVP_USE_S_VALS_FOR_PRETENSION

//#define IVP_DISTANCE_KEEPERS
//#define IVP_PANELTY_SOLVER


#ifndef IVP_CONTROLLER_INCLUDED
#	include <ivp_controller.hxx>
#endif

#ifndef _IVP_CONTACT_SITUATION_INCLUDED
#	include <ivp_contact_situation.hxx>
#endif

#ifndef _IVP_MINDIST_INCLUDED
#   include <ivp_mindist.hxx>//lwss add
#endif


// set completely (except union) by recalc friction s_vals
class IVP_Impact_Solver_Long_Term : public IVP_Contact_Situation {
public:
    short    index_in_fs;
    short    impacts_while_system;   
    IVP_BOOL coll_time_is_valid:8;
    IVP_BOOL friction_is_broken:2;
    // next block is mantained by calc_coll_dist and needed by impact

    union {
	struct { // impact data
	    IVP_FLOAT rescue_speed_addon;
	    IVP_FLOAT distance_reached_in_time;
	    IVP_FLOAT percent_energy_conservation; // #+# maybe move to IVP_Contact_Point, maybe short 1.0f subtract energy destroyed
	} impact;
	// next block is used for friction and set by IVP_Friction_Solver::setup_coords_mindists
	struct { // friction data
	    IVP_Friction_Info_For_Core *friction_infos[2]; //two friction infos for one distance, to get all distances of a object  x2
	    int has_negative_pull_since;                   // copy of contact point to avoid cache misses
	    IVP_FLOAT dist_len;                            // copy of distance of two objects now    
	} friction;
    };
    IVP_FLOAT  virtual_mass;             // in direction of surface normal
    IVP_FLOAT  inv_virtual_mass;         // in direction of surface normal
    
    IVP_Core *contact_core[2];                    // == 0 for unmoveable cores

    IVP_U_Float_Point span_friction_v[2]; // orthogonal vectors in world coords
    // vector of v[0]*s[0] + v[1]*s[1] is real world force vector of force that has to be done on first obj (is pointing to second obj)
 
    IVP_U_Float_Point contact_point_cs[2];   // impact/friction position in core coordinates
    IVP_U_Float_Point contact_cross_nomal_cs[2]; // crossproduct of contact_point_cs and normal_cs


  // *******************
  // ******** functions:
  // *******************    
    void init_tmp_contact_info() { impacts_while_system=0; coll_time_is_valid=IVP_FALSE; friction_is_broken = IVP_FALSE; };
  
    IVP_Impact_Solver_Long_Term() { init_tmp_contact_info(); };

    inline IVP_DOUBLE get_closing_speed() const ;  // based on current core speed
    
    void do_impact_long_term(IVP_Core *pushed_cores[2], IVP_FLOAT rescue_speed_val, IVP_Contact_Point *cp); // returns 0 if O.K. , returns 1 if false impact
    static IVP_Core *find_second_critical_impact_core(IVP_Core *core1,IVP_Core *core0);
    
    // the real start of an impact
    static void do_impact_of_two_objects(IVP_Mindist *mindist, IVP_Real_Object *obj0,IVP_Real_Object *obj1);
    static IVP_Core *get_best_merge_core(IVP_Core *core0,IVP_Core *core1,IVP_Core *core2);
};


class IVP_Synapse_Friction {  // sizeof() == 32
public:
  IVP_Synapse_Friction 	 *next, *prev;		        // per object list
  IVP_Real_Object  *l_obj;				// back link to object
protected:
  short contact_point_offset;               // back link to my controlling mindist
  short status:8;			    // type IVP_SYNAPSE_POLYGON_STATUS  point, edge, tri, ball ....
public:
    
  const IVP_Compact_Edge 	*edge;		// Note: all balls share one dummy edge


    // has to be transfered to synapse_friction_polygon:

    IVP_Real_Object *get_object(){ return l_obj; };
    IVP_Synapse_Friction       *get_next(){ return next;};
    IVP_Synapse_Friction       *get_prev(){ return prev;};
    IVP_Contact_Point *get_contact_point()const{ return  (IVP_Contact_Point *)(contact_point_offset + (char *)this);};
    void set_contact_point( IVP_Contact_Point *cp ) { contact_point_offset = ((char *)cp) - (char *)this; };

    int                        get_material_index() const;
    IVP_SYNAPSE_POLYGON_STATUS get_status()const { return (IVP_SYNAPSE_POLYGON_STATUS)status; }
    inline void init_synapse_friction(IVP_Contact_Point *friction,
	    		    IVP_Real_Object *object,
			    const IVP_Compact_Edge 	*edge,
			    IVP_SYNAPSE_POLYGON_STATUS status);

    IVP_BOOL is_same_as(const IVP_Synapse_Real *syn_comp) const;
    inline void remove_friction_synapse_from_object();  // @@@OS insert fr ????
};

//  now_friction_pressure: force done by complex SI:  t*m / s*s
//     This force is: push * inv_delta_PSI_time
//     SI of push: t*m / s
//  real_fr_now_len is length of horizontal friction spring
//     Real friction force done on objects horizontal: (now_friction_pressure * inv_max_len * friction_factor)  *  real_fr_now_len    .SI:  t*m / s*s
//     Corresponding D of horizontal spring: now_friction_pressure * inv_max_len * friction_factor     .SI:  t / s*s
//     Energy stored in horizontal spring:  0.5f * D * real_fr_now_len*real_fr_now_len   . SI:  t*m*m / s*s
//  Pot Energy won by complex: now_friction_pressure * delta Distance.    SI:  t*m*m / s*s 

// fast static area
class IVP_Contact_Point_Fast_Static {
protected:

    IVP_Contact_Point *next_dist_in_friction; // all friction springs in a friction system
    IVP_Contact_Point *prev_dist_in_friction;

    IVP_Synapse_Friction       synapse[2];

    //for real friction
    IVP_FLOAT inv_virt_mass_mindist_no_dir; 

    IVP_BOOL two_friction_values:8;
};

enum IVP_CONTACT_POINT_BREAK_STATUS {
    IVP_CPBS_NORMAL,
    IVP_CPBS_NEEDS_RECHECK,
    IVP_CPBS_BROKEN
};

// add fast dynamics
class IVP_Contact_Point_Fast: public IVP_Contact_Point_Fast_Static {
protected:
#ifdef IVP_USE_S_VALS_FOR_PRETENSION
    IVP_FLOAT s_coords[2];	    //
#endif    
    IVP_FLOAT span_friction_s[2];

    IVP_Impact_Solver_Long_Term *tmp_contact_info;

    IVP_FLOAT real_friction_factor;          // the friction factor
    
    IVP_FLOAT integrated_destroyed_energy;   // for watching a contact point over time, e.g. to drive the sound engine
    IVP_FLOAT inv_triangle_det;              // to optimize pF cases
    // for energy difference calculations
    IVP_FLOAT old_energy_dynamic_fr;
    IVP_FLOAT now_friction_pressure; // needed to add values  distance_keeper pressure + static friction
    IVP_FLOAT last_gap_len; //length of gap at last PSI (to calc gained energy)
    
    IVP_FLOAT get_gap_length() { return last_gap_len; };
    
    short slowly_turn_on_keeper;     // is initialized with IVP_SLOWLY_TURN_ON_KEEPER
    IVP_CONTACT_POINT_BREAK_STATUS  cp_status:8;
    int has_negative_pull_since;   //0 means inactive, n>1 means negative since n-1 PSIs, -n>0 means has positive value since n PSIs
    IVP_Time last_time_of_recalc_friction_s_vals; // move to friction system
    IVP_U_Float_Point last_contact_point_ws;
};

class IVP_Contact_Point: public IVP_Contact_Point_Fast {
    friend class IVP_Real_Object;
    friend class IVP_Core;
    friend class IVP_Friction_Solver;
    friend class IVP_Impact_Solver_Long_Term;
    friend class IVP_Impact_Solver;
    friend class IVP_Friction_System;
    friend class IVP_Friction_Sys_Static;
    friend class IVP_Impact_System;
    friend class IVP_Friction_Core_Pair;
    friend class IVP_Contact_Point_API;

    friend class IVP_Mindist;
    friend class IVP_Friction_Manager;

    IVP_Friction_System *l_friction_system; //backlink for deleting
    
    IVP_BOOL is_same_as(const IVP_Mindist *md2) const;

    // impact system
    void get_cos_sin_for_impact(IVP_FLOAT friction_val,IVP_FLOAT *cos_val,IVP_FLOAT *sin_val);
    void read_materials_for_contact_situation(IVP_Impact_Solver_Long_Term *info); // #+# check calls to this function when material is changed on the fly -> recalc some values

    void calc_coll_distance(); // returns the distance reached in one PSI and the rescue speed val and stores result in impact.distance_reached_in_time

    IVP_FLOAT get_rot_speed_uncertainty();
    IVP_FLOAT get_rescue_speed_impact(IVP_Environment *);
    
    inline IVP_FLOAT get_possible_friction_slide_way(const IVP_Event_Sim *es);
   
    void debug_situation_after_impact();    
  
    void calc_spring_constant_mindist_distance_keeper();
    void calc_virtual_mass_of_mindist();
    void update_distance_keepers_both_cores();
    
    /*** for friction mode ***/
    // for friction mode
    void leave_friction_mode();
    void reset_time(IVP_Time offset);
    
    //for easing friction forces
    void ease_the_friction_force(IVP_U_Float_Point *diff_vec);  // #+#, call only randomly, create flag for sliding

    //for doing real friction
    void friction_force_local_constraint_1d(const IVP_Event_Sim *);
    IVP_FLOAT friction_force_local_constraint_2d(const IVP_Event_Sim *);
	bool friction_force_local_constraint_2d_wheel( IVP_Core *core_a, IVP_Impact_Solver_Long_Term *info, const IVP_Event_Sim *es, IVP_FLOAT &flEnergy );
    void static_friction_single(const IVP_Event_Sim *es,IVP_FLOAT dg,IVP_FLOAT speedup_factor);
    IVP_BOOL get_world_direction_second_friction(IVP_U_Float_Point *ret_second_friction_vec,IVP_FLOAT *sin_second_friction);
    IVP_DOUBLE two_values_friction(IVP_U_Float_Point *real_world_friction_vec);
    IVP_DOUBLE get_and_set_real_friction_len(IVP_U_Float_Point *real_world_friction_vec);

    inline void calc_pretension(IVP_FLOAT len);

    IVP_Impact_Solver_Long_Term *get_lt(){ return tmp_contact_info; };
    
    ~IVP_Contact_Point();
    // some friction checks:

    void p_calc_friction_s_PP(const IVP_U_Point *pp0,const IVP_U_Point *pp1,IVP_Impact_Solver_Long_Term *info, IVP_U_Float_Point *diff_contact_vec);
    void p_calc_friction_qr_PF(const IVP_U_Point *pp, const  IVP_Compact_Edge *F, IVP_Cache_Ledge_Point *m_cache_F,IVP_Impact_Solver_Long_Term *info, IVP_U_Float_Point *diff_contact_vec);
    void p_calc_friction_ss_KK(const IVP_Compact_Edge *K,const  IVP_Compact_Edge *L, IVP_Cache_Ledge_Point *m_cache_K, IVP_Cache_Ledge_Point *m_cache_L, IVP_Impact_Solver_Long_Term *info, IVP_U_Float_Point *diff_contact_vec);
    void p_calc_friction_s_PK(const IVP_U_Point *pp, const IVP_Compact_Edge *K, IVP_Cache_Ledge_Point *m_cache_K, IVP_Impact_Solver_Long_Term *info, IVP_U_Float_Point *diff_contact_vec);
public:
	const IVP_U_Float_Point *get_contact_point_ws() const { return &last_contact_point_ws; }
    IVP_Synapse_Friction *get_synapse(int i){ return &synapse[i];};
    const IVP_Synapse_Friction *get_synapse(int i) const { return &synapse[i];};
    void recalc_friction_s_vals(); // move on surfaces but keep topologie
    IVP_Contact_Point(IVP_Mindist *mindist);
    void set_friction_to_neutral();
    void get_material_info(IVP_Material *mtl[2]);

    //lwss add
    void get_contact_normal( IVP_U_Float_Point *pOutNormal )
    {
        // lwss - I am unsure if this is correct! But it's just a simple 3x float assign
        pOutNormal->set( &last_contact_point_ws );
        //floats(DWORD*)
        // +44
        // +45
        // +43
    }
    float get_friction_factor()
    {
        return real_friction_factor;
    }
    void recompute_friction();
    //lwss end
};



// store information of pairs of objects existing
// used for union find and easing of friction forces
class IVP_Friction_Core_Pair {
    friend class IVP_Friction_System;
    friend class IVP_Friction_Sys_Static;
    friend class IVP_Impact_System;
    friend class IVP_Friction_Solver;
  
    IVP_U_Vector<IVP_Contact_Point> fr_dists;

  
    IVP_DOUBLE destroy_mutual_energy(IVP_DOUBLE d_e); //returns amount of energy that was actually reduced, rest is accumulated in integrated_anti_energy

    IVP_U_Float_Point span_vector_sum; // only for debug
public:
    int next_ease_nr_psi;
    IVP_Time last_impact_time_pair;
    IVP_FLOAT integrated_anti_energy;
    IVP_Core *objs[2];
  
    IVP_Friction_Core_Pair();
    ~IVP_Friction_Core_Pair();
    
    void remove_energy_gained_by_real_friction();
    int check_all_fr_mindists_to_be_valid(IVP_Friction_System *fs); //returns number of still valid mindists; warning, maybe this function deletes whole structure 'this'
    void get_average_friction_vector(IVP_U_Float_Point *average_friction);
    void set_friction_vectors(IVP_U_Float_Point *average_friction);
    inline void pair_calc_friction_forces(const IVP_Event_Sim *);
    inline IVP_FLOAT get_sum_slide_way(const IVP_Event_Sim *es);
    
    void add_fr_dist_obj_pairs(IVP_Contact_Point *dist);
    void del_fr_dist_obj_pairs(IVP_Contact_Point *dist);
    int number_of_pair_dists();

    void debug_store_vector_before_ease();
    void debug_read_vector_after_ease();
    void debug_printf_pair();
};

class IVP_Friction_Sys_Energy : public IVP_Controller_Independent {
public:  
    IVP_Friction_System *l_friction_system;
  
    IVP_CONTROLLER_PRIORITY get_controller_priority() { return IVP_CP_ENERGY_FRICTION; };
    void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list);

    virtual ~IVP_Friction_Sys_Energy() { ; };
    void core_is_going_to_be_deleted_event(IVP_Core *core);
    IVP_DOUBLE get_mimumum_simulation_frequency() { return 1.0f; };
};

class IVP_Friction_Sys_Static : public IVP_Controller_Independent {
    void do_simulation_single_friction(IVP_Event_Sim *es);    
public:
    IVP_Friction_System *l_friction_system;
  
    IVP_CONTROLLER_PRIORITY get_controller_priority() { return IVP_CP_STATIC_FRICTION; };
    void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list);

    virtual ~IVP_Friction_Sys_Static(){ ; };
    
    void core_is_going_to_be_deleted_event(IVP_Core *del_core);
    IVP_DOUBLE get_minimum_simulation_frequency() { return 1.0f; };
};

// a physical unmovable object can have more than one friction system and for every friction systems several distances
class IVP_Friction_Info_For_Core {
public: 
    IVP_U_Vector<IVP_Contact_Point> friction_springs;
    IVP_Friction_System *l_friction_system;

    void set_all_dists_of_obj_neutral();
	
    void friction_info_insert_friction_dist(IVP_Contact_Point *dist);
    void friction_info_delete_friction_dist(IVP_Contact_Point *dist);
    int dist_number();
};

class IVP_Friction_System : public IVP_Controller_Dependent {
    friend class IVP_Friction_Core_Pair;
    friend class IVP_Friction_Solver;
    friend class IVP_Friction_Hash;
    friend class IVP_Contact_Point;
    friend class IVP_Core;
    friend class IVP_Mindist;
    friend class IVP_Friction_Sys_Static;
    friend class IVP_Real_Object;
    friend class IVP_Friction_Sys_Energy;
    
    IVP_Environment *l_environment;

    IVP_Friction_Sys_Static static_fs_handle;
    IVP_Friction_Sys_Energy energy_fs_handle;
  
    IVP_Friction_System *next_friction_system;
    IVP_Friction_System *prev_friction_system;
    IVP_Contact_Point *first_friction_dist;
public:    
    IVP_U_Vector<IVP_Core> cores_of_friction_system;
    IVP_U_Vector<IVP_Core> moveable_cores_of_friction_system;
    IVP_U_Vector<IVP_Friction_Core_Pair> fr_pairs_of_objs;
private:
    short	friction_obj_number; 
    short	friction_dist_number; 
    short	complex_not_necessary_number; //the first of this number mindists are not recognized in complex
public:
    // some flags/constants
    IVP_BOOL union_find_necessary:8; // flag set when mindist is removed and a split into two systems may be possible
    IVP_BOOL fr_sys_simulated:8;

    IVP_Friction_System(IVP_Environment *env);
    virtual ~IVP_Friction_System();
        
    void add_core_to_system(IVP_Core *core);
    void remove_core_from_system(IVP_Core *obj);


    void add_fr_pair(IVP_Friction_Core_Pair *pair);
    void del_fr_pair(IVP_Friction_Core_Pair *pair);
    IVP_Friction_Core_Pair *get_pair_info_for_objs(IVP_Core *core0,IVP_Core *core1);    
    IVP_Friction_Core_Pair *find_pair_of_cores(IVP_Core *core0,IVP_Core *core1);
    
    //dist datastructures
    int get_fr_dist_number() { return friction_dist_number; };
    IVP_Contact_Point *get_first_friction_dist() { return first_friction_dist; };
    IVP_Contact_Point *get_next_friction_dist(IVP_Contact_Point *old) { return old->next_dist_in_friction; };
    IVP_Contact_Point *get_prev_friction_dist(IVP_Contact_Point *old) { return old->prev_dist_in_friction; };
    void exchange_friction_dists(IVP_Contact_Point *first,IVP_Contact_Point *second);
    void add_dist_to_system(IVP_Contact_Point *dist);
    void remove_dist_from_system(IVP_Contact_Point *dist);
    void delete_friction_distance(IVP_Contact_Point *old_dist); //has a call to remove_dist_from_system
    IVP_BOOL dist_removed_update_pair_info(IVP_Contact_Point *old_dist); //returns IVP_TRUE if union find has to be called
    void dist_added_update_pair_info(IVP_Contact_Point *new_dist); 

    //operations
  //void fs_remove_gained_energy();
  //void fs_sim_static_friction();
  //void do_simulation_core_variables();
    void static_fr_oversized_matrix_panic();
    IVP_BOOL core_is_terminal_in_fs(IVP_Core *test_core);
    void fs_recalc_all_contact_points();
    void reorder_mindists_for_complex();
    void do_friction_system(const IVP_Event_Sim *es_in); 
    void confirm_complex_pushes();
    void undo_complex_pushes();
    int get_num_supposed_active_frdists();
    void apply_real_friction(const IVP_Event_Sim *es); // does real friction according to pressure calculated by 'do_friction_system' in last PSI
    void clear_integrated_anti_energy();
    void remove_energy_gained_by_real_friction();
    void ease_friction_forces();
    void calc_friction_forces(const IVP_Event_Sim *);
    IVP_BOOL core_is_found_in_pairs(IVP_Core *test_core); //for union find
    void bubble_sort_dists_importance();
    void do_pushes_distance_keepers(const IVP_Event_Sim *es);
    IVP_DOUBLE get_max_energy_gain();
    void fusion_friction_systems(IVP_Friction_System *second_sys);
    IVP_Core *union_find_fr_sys();
    void split_friction_system(IVP_Core *split_father);

    //fs is controller
    void reset_time( IVP_Time offset);
    void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list);
    IVP_CONTROLLER_PRIORITY get_controller_priority() { return IVP_CP_DYNAMIC_FRICTION; };  
    void core_is_going_to_be_deleted_event(IVP_Core *core);  
    void get_controlled_cores(IVP_U_Vector<IVP_Core> *vectr);
    IVP_DOUBLE get_minimum_simulation_frequency();
    IVP_U_Vector<IVP_Core> *get_associated_controlled_cores();
  
    //for debugging
    void debug_clean_tmp_info();
    void debug_check_system_consistency();
    IVP_DOUBLE sum_energy_destroyed;
    void test_hole_fr_system_data(); 
    void print_all_dists();
    static int friction_global_counter;
    IVP_DOUBLE kinetic_energy_of_hole_frs();
    void debug_fs_out_ascii();
    void ivp_debug_fs_pointers();
    void debug_fs_after_complex();
};

class IVP_Mutual_Energizer {
    IVP_U_Float_Point trans_vec_world; //direction of push done to core0, to core1 negative push is given
    IVP_U_Float_Point rot_vec_obj[2]; //direction of pushes done to core0 and core1 in obj coords
    IVP_DOUBLE trans_speed_potential; //difference between both objects
    IVP_DOUBLE trans_inertia[2]; //normally mass
    IVP_DOUBLE inv_trans_inertia[2]; //normally inv_mass
    IVP_DOUBLE rot_speed_potential; //difference in rotation
    IVP_DOUBLE rot_inertia[2]; //resistence against rot pushes; resistance against trans pushes are masses
    IVP_DOUBLE inv_rot_inertia[2]; // inverses

    IVP_DOUBLE rot_energy_potential;
    IVP_DOUBLE trans_energy_potential;

    static inline void get_rot_inertia(IVP_Core *core,IVP_U_Float_Point *rot_vec_obj,IVP_DOUBLE *rot_inertia,IVP_DOUBLE *inv_rot_inertia);
    IVP_DOUBLE calc_impulse_to_reduce_energy_level(IVP_DOUBLE speed_pot,IVP_DOUBLE inv_mass0,IVP_DOUBLE inv_mass1,IVP_DOUBLE delta_e);
    
public:
    IVP_Core *core[2];
    IVP_DOUBLE whole_mutual_energy;

    static IVP_DOUBLE calc_energy_potential(IVP_DOUBLE speed_pot,IVP_DOUBLE mass0,IVP_DOUBLE mass1,IVP_DOUBLE inv_mass0,IVP_DOUBLE inv_mass1);
    
    void init_mutual_energizer(IVP_Core *core0,IVP_Core *core1);
    void calc_energy_potential();
    void destroy_percent_energy(IVP_DOUBLE percent_energy_to_destroy);    
};

class IVP_Friction_Manager {
public:
    static IVP_Contact_Point *generate_contact_point(IVP_Mindist *orig_mindist,IVP_BOOL *was_success);
    static IVP_Contact_Point *get_associated_contact_point(IVP_Mindist *mindist);
    static void delete_all_contact_points_of_object(IVP_Real_Object *obj0);
};


IVP_DOUBLE IVP_Impact_Solver_Long_Term::get_closing_speed() const{
    IVP_DOUBLE result = 0.0f;
    IVP_Core *c0;
    if ( (c0 = this->contact_core[0])){
	result = ( c0->speed.dot_product( & this->surf_normal ) +
		     c0->rot_speed.dot_product( &this->contact_cross_nomal_cs[0] ));
    }

    IVP_Core *c1;
    if ((c1 = this->contact_core[1])){
	result -= c1->speed.dot_product( & this->surf_normal ) +
	          c1->rot_speed.dot_product( &this->contact_cross_nomal_cs[1] );
    }
    return result;
}

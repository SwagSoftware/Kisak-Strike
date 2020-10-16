// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivu_list.hxx>

#define P_OBJECT_BLUR 0.01 // some other eps depend on it
                           // distance in m for physical collision epsilon

extern IVP_DOUBLE P_Pop_Eps;
extern IVP_DOUBLE P_Pop_Scal_Eps;
extern IVP_DOUBLE P_Pop_Too_Flat_Eps;

class IVP_Triangle;
class IVP_Poly_Point;
class IVP_Poly_Surface;
class IVP_Polygon;
class P_Sur_2D_Line;
class IVP_Object_Polygon_Tetra;
class IVP_Template_Surface;
class IVP_Template_Polygon;
class P_Object_Polygon;
class IVP_Tetra_Point;
class IVP_Tetra_Edge;
class IVP_Tetra_Point;
class P_Surface;
class IVP_Material;
class P_Level;

enum P_CONVEXIFY_STATE
{
    P_CONVEXIFY_STATE_NONE,
    P_CONVEXIFY_STATE_INIT,
    P_CONVEXIFY_STATE_LINK,
    P_CONVEXIFY_STATE_NORM_POP,
    P_CONVEXIFY_STATE_PROBLEM_POP
};

enum P_HASH_CLASS
{
    P_HASH_CLASS_NONE,
    P_HASH_CLASS_NORMAL,
    P_HASH_CLASS_PROBLEM,
    P_HASH_CLASS_EPSILON,
    P_HASH_CLASS_CONVEX_INTRUDE,
    P_HASH_CLASS_MAX
};


class IVP_Tri_Edge;


/** Surface used for physical textures */
class IVP_Poly_Surface {
public:
    IVP_Material *l_material; // may be shared
    IVP_Object_Polygon_Tetra	*tetras;
	P_Surface	*l_p_debug_surface;		// for debugging
    void set(IVP_Template_Surface *temp_sur, IVP_Object_Polygon_Tetra *i_tetras);
    int get_surface_index();
};

class IVP_Real_Object;

class IVP_Poly_Point : public IVP_U_Point
{
    friend class IVP_Tri_Edge;
public:
    IVP_Poly_Point() { tmp.tetra_point = NULL; };    
    inline IVP_Real_Object *get_real_object2() const; // @@@@@ returns 0
    IVP_Object_Polygon_Tetra *l_tetras;
    union {			// temporary pointers
	IVP_Tetra_Point *tetra_point;
	int compact_index; // for compact gen
    } tmp;
    
    int point_num();
    void print(const char *text=0);
    int p();    
};

class IVP_Tri_Edge	// Triangle Edge
{
public:
    
    IVP_Poly_Point  *start_point; // only save one point per edge
	
    IVP_Triangle *triangle;	// corresponding triangle
    IVP_Tri_Edge *next;		// next edge
    IVP_Tri_Edge *prev;		// previous edge
    IVP_Tri_Edge *behind;       // falls kein blatt die direkt dahinterliegende Kannte des darunterliegenden Dreiecks
    IVP_Tri_Edge *opposite;	// opposite edge2 of neighbor-triangle

    union {
	struct {
	    char checked_in;		// checked in tetra_intrude
	    char hash_class;		// P_HASH_CLASS
	    char concav_flag; // see check_concavity()
	    IVP_Tetra_Point *tetra_point; // temporarily for tetraedification
	} gen;
    } tmp;

    IVP_DOUBLE concavity; // <0 =convex, else measure for volume

    
  IVP_Tri_Edge *search_nearest_edge_to(IVP_U_Point *reference,IVP_DOUBLE *quad_old_dist);
    
    int check_concavity(IVP_Tri_Edge *other_edge);
    
    IVP_Tri_Edge *other_side();
    IVP_BOOL is_concav(){ return (IVP_BOOL)(concavity < -1E-10f); };
    void print(const char *text = 0);
    

    int p(); /// for debugger
};

class IVP_Triangle { //
private:
public:    
    IVP_Triangle *next; // all triangles in a linked list
    IVP_Triangle *prev;
    IVP_Triangle *other_side; // 'backside' triangle in a tetraeder
    IVP_Triangle *pierced_triangle; // for mindist minimize

    struct {
	char is_terminal;	// leaf of triangle hierarchy?
	char is_hidden; // some triangles are above (by pop e.g.)
	ushort ledge_group; // used for piercing, indicates pierce 'time' 
    } flags;
    
    IVP_Poly_Surface *ivp_surface;
    IVP_Tri_Edge three_edges[3];
    
    struct ivp_triangle_tmp{
	struct ivp_triangle_gen{
	    IVP_U_Hesse hesse;			// normized hesse
		ivp_triangle_gen(){;};
	} gen;
	ivp_triangle_tmp(){;};
    } tmp;    
    int index; // used for compact ledge generation
    
    void calc_hesse(); // calc hesse out of edges
    IVP_DOUBLE calc_areasize(); // calc size of triangle area
    
    IVP_Triangle();
    virtual ~IVP_Triangle();

    int print(const char *comment = NULL);
};

class IVP_U_Min_Hash;
class IVP_Hash;

class IVP_Tetra_Intrude;
class IVP_Intrusion_Status;
class IVP_Template_Polygon;

class IVP_Extra_Point : public IVP_Poly_Point {
public:
    IVP_Extra_Point *next;
};

class IVP_Object_Polygon_Tetra {
    friend class P_Sur_2D;
    friend class IVP_Mindist;
    friend class IVP_Tri_Edge;
    
    /**** basic internal data **/
    // see public section

    /**** intermediate data during creation ****/
public:
    IVP_Template_Polygon *template_polygon;	// used for generation !!!!
private:
    IVP_U_Min_Hash *min_hash[P_HASH_CLASS_MAX];	// needed to find the best element for tetraification
public:
    IVP_Hash *points_to_edge_hash;
private:    
    IVP_Tetra_Intrude *tetra_intrude;

    int			n_tetra_points;
    int			n_tetra_points_malloced;
    IVP_Tetra_Point	*tetra_points; // array of tetra points, temp

    
    /**** intermediate data during simulation ****/
    
/* Some Sub Constructors */
public:
    IVP_ERROR_STRING make_triangles();
private:
    void free_triangles();

         
    void calc_concavities(); // calcs and counts all concav edges
    IVP_ERROR_STRING total_breakivp_edge(IVP_Tri_Edge *edge);
    IVP_ERROR_STRING breakivp_triangle(IVP_Triangle *triangle, IVP_Triangle *no_break_a=NULL,
			     IVP_Triangle *no_break_b = NULL, IVP_Triangle *no_break_c = NULL);
    IVP_ERROR_STRING breakivp_random_triangle(IVP_Tri_Edge *edge);
    IVP_DOUBLE rate_tri_edge(IVP_Tri_Edge *edge);
    void move_edge_to_problem_hash(IVP_Tri_Edge *edge);
    void move_edge_to_epsilon_hash(IVP_Tri_Edge *edge);
    void move_edge_to_normal_hash(IVP_Tri_Edge *edge);
    void move_edge_to_convex_intrude_hash(IVP_Tri_Edge *edge);
    void remove_edge_from_min_list(IVP_Tri_Edge *edge);
    void add_edge_to_min_list(IVP_Tri_Edge *edge,P_HASH_CLASS hash_class, IVP_DOUBLE rating);
    void reveal_triangle(IVP_Triangle *tri);
    void reveal_triangles_behind(IVP_Triangle *tri);
    void hide_triangle(IVP_Triangle *tri);
    int check_concavity_and_manage(IVP_Tri_Edge *edge, P_CONVEXIFY_STATE);
    void add_edge_into_point_to_edge_hash(IVP_Tri_Edge *edge);
    void record_intruding_convex_edges(IVP_Intrusion_Status *status);
    IVP_Tri_Edge *get_an_edge_with_points(IVP_Poly_Point *p1, IVP_Poly_Point *p2);
    void calc_extrusion_point(const IVP_Tri_Edge *edge, IVP_U_Point *point_out);
    void mark_not_connectable(IVP_Poly_Point *p1, IVP_Poly_Point *p2);
   
    IVP_BOOL p_link_edge(IVP_Tri_Edge *edge, IVP_Tri_Edge *neighbor);	// returns true if problems
    void pop_concav_edge(IVP_Tri_Edge *edge);   
    void pop_problematic_edge(IVP_Tri_Edge *edge);   
    int link_triangle_couple(IVP_Triangle *tri_a,
			       IVP_Tri_Edge *neighbor_0,
			       IVP_Tri_Edge *neighbor_1,
			       IVP_Tri_Edge *neighbor_2);
    
    void make_double_triangle_permanent(IVP_Triangle *triangle);
    
    void link_existing_pop_edge(IVP_Tri_Edge *pop_edge);

    IVP_ERROR_STRING final_convexify_check();
    
    IVP_DOUBLE calc_rot_inertia_for_optimization( IVP_Template_Polygon *tpop, IVP_FLOAT angle, short rot_axis, short value);
    void maximize_one_physic_axis( IVP_Template_Polygon *tpop, IVP_U_Matrix *m_axis, short max, short rot_axis, short start) ;
	
    /*********** Public Section *************/
public:

    /**** basic internal data **/
    class IVP_Real_Object *real_object;
    
    P_List<IVP_Triangle> triangles;	// pointer to first triangle

    IVP_Poly_Point *points;		// points
    int	n_points;
    
    IVP_Poly_Surface *surfaces;
    int		n_surfaces;
    
    /** Extra points generated by deadlock in make_tetraeder */
    
    IVP_Extra_Point *extra_points; // invisible, extra for tetraeders
    int n_extra_points;
    
        
    
    IVP_Object_Polygon_Tetra(IVP_Template_Polygon *i_temp_pop);
    
    ~IVP_Object_Polygon_Tetra();

    /* build check and test */
    void convexify();			// inserts new triangles to get a convex result
    IVP_ERROR_STRING check_konsistency_of_triangles();

    /* Object shift and rotate */
    void calc_main_physic_axis(IVP_Template_Polygon *tpop,IVP_U_Matrix *m_core_f_pop_out);


    /* Generate an object polygon from trianges */
    void create_surface_from_triangles(P_Object_Polygon *out_object);

    /* etc */
    void insert_pierce_info();
    static IVP_Triangle *generate_double_triangle(IVP_Poly_Point *p1, IVP_Poly_Point *p2, IVP_Poly_Point *p3);
};

inline IVP_Real_Object *IVP_Poly_Point::get_real_object2() const { return NULL; } // l_tetras->real_object
//inline IVP_Polygon *IVP_Poly_Point::get_polygon() const { return (IVP_Polygon *)l_tetras->real_object; };




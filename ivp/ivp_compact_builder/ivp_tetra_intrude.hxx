// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


class IVP_Tri_Edge;
class IVP_Hash;
class IVP_Poly_Point;
#define P_INVALID_TETRA_EDGE  ((char *)-1)

enum IVP_INTRUSION_CHECK_RESULTS {
    IVP_INTRUSION_CHECK_NONE = 0,
    IVP_INTRUSION_CHECK_EPSILON = 1,
    IVP_INTRUSION_CHECK_OVERLAP = 2,
    IVP_INTRUSION_CHECK_POINT = 3,
    IVP_INTRUSION_CHECK_LINE
};

class IVP_Tetra_Intrude;


/*	Tetra Points and Edges are used to find intrusion problems during
 *	the creation phase of tetraeders,
 *	Each axis is diveded into 32 parts, each part represented by one bit
 */

  
class IVP_Tetra_Point { // just temporarily used for tetraedification
public:
    int		cover_area_bits[3]; // includes all edges
    IVP_Poly_Point	*opoint;
    // int		clock;
    // temporary variables
    int		tmp_side_of_triangle_bits;
    void init(IVP_Tetra_Intrude *ti);		// call after opoint is set
    void print(const char *text = 0);
    int p();
};

class IVP_Tetra_Edge {
public:
    int		reference_count;
    int		cover_area_bits[3];
    IVP_Tetra_Point	*tetra_points[2];
    IVP_Tri_Edge		*source_tri_edge;
    void print(const char *text = 0);
    int p();
};


enum IVP_POP_TYPE {
    IVP_POP_TYPE_NORMAL,
    IVP_POP_TYPE_CLOSING
};

struct IVP_Intrusion_Included_Points {
    IVP_Intrusion_Included_Points *next;
    IVP_Tetra_Point *tetra_point;
    void print(const char *text);
    int p();
};

struct IVP_Intrusion_Intersection {
    IVP_INTRUSION_CHECK_RESULTS type;
    IVP_Intrusion_Intersection *next;
    IVP_U_Point 	intersect_point;
    IVP_Poly_Point	*line_endpoints[2];
    IVP_Tri_Edge	*tri_edge;	// of the tetra-eder
    IVP_Intrusion_Intersection *pairing_intersection;
    IVP_Intrusion_Included_Points *included_point;
    void print(const char *text);
    int p();
};

class IVP_Intrusion_Status {
public:
    IVP_Intrusion_Included_Points *intruded_points;
    IVP_Intrusion_Intersection *intersections;
    IVP_Intrusion_Status();
    ~IVP_Intrusion_Status();
    void print(const char *text);
    int p();
};

class IVP_Extra_Point;

/** Manager Class to find intrusion of tetraeders */
class IVP_Tetra_Intrude {
    friend class IVP_Tetra_Point;
    IVP_U_Point min_koord;
    IVP_U_Point max_koord;
    int		memsize_of_tetra_edges;
    int		n_tetra_edges;
    IVP_Tetra_Edge *tetra_edges;
    IVP_Hash	*twop_2_tetra_edge_hash;
    void	point_2_bits(IVP_U_Point *lpos,IVP_U_Point *rpos,int *result_bitmasp);
    IVP_INTRUSION_CHECK_RESULTS check_point_intrusion(int bitmask[3], IVP_Tetra_Point *t_points[4],
						    IVP_U_Hesse hesse_of_t[4], IVP_DOUBLE p_pop_eps[4],
						    IVP_U_Hesse edge_hesses[6], IVP_Tetra_Point *tp,
						    IVP_Intrusion_Status *status);
public:
    IVP_Tetra_Intrude(IVP_Tetra_Point *i_tetra_points, int orig_pnt_anz);
    ~IVP_Tetra_Intrude();

    int			n_tetra_points;
    int			n_tetra_points_malloced;
    IVP_Tetra_Point	*tetra_points; // array of tetra points
    void init_tetra_edge(IVP_Tetra_Edge *edge, IVP_Tetra_Point *p0, IVP_Tetra_Point *p1,IVP_Tri_Edge *e);
    void checkin_edge(IVP_Tri_Edge *edge);
    void checkout_edge(IVP_Tri_Edge *edge);
				// if type == IVP_POP_TYPE_CLOSING -> pop_edge_b must be the new triangle
    IVP_INTRUSION_CHECK_RESULTS check_intrusion(IVP_Tri_Edge *old_pop_edge_a, IVP_Tri_Edge *old_pop_edge_b,
			IVP_Tri_Edge *pop_edge_a, IVP_Tri_Edge *pop_edge_b,
			IVP_Extra_Point *first_extra_point,int n_new_triangles,
			IVP_Intrusion_Status *status = 0);
    IVP_Intrusion_Status *calc_intrusion_status(
	IVP_Tri_Edge *old_pop_edge_a, IVP_Tri_Edge *old_pop_edge_b,
	IVP_Tri_Edge *pop_edge_a, IVP_Tri_Edge *pop_edge_b,
	IVP_Extra_Point *first_extra_point,int n_new_triangles);

    
    void print(const char *text);
    int pe(const int pnt_num);	// print all edges including pnt_num
    int p();
};

// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#include <ivp_physics.hxx>
#include <string.h>
#include <ivu_float.hxx>

#include <ivu_list.hxx>
#include <ivu_hash.hxx>
#include <ivu_geometry.hxx>

#ifndef WIN32
#	pragma implementation "ivp_object_polygon_tetra.hxx"
#endif

#include "ivp_triangle_gen.hxx"
#include <ivp_object_polygon_tetra.hxx>
#include <ivp_templates_intern.hxx>
#include <ivu_min_hash.hxx>
#include <ivp_tetra_intrude.hxx>

/* for debugging stop after some pops */
int	   n_physical_pops = -1;

/*** DEFINES ***/

//#define SUR_DEBUG
//#define CONVEX_DEBUG

#define P_POP_SURFACE_CONVEX_BLUR (0.1f * P_OBJECT_BLUR)		
#define P_POP_EPS_INIT 0.001f
#define P_POP_EPS_REDUCE_FACTOR 0.125f
#define P_POP_TOO_FLAT_EPS_FACTOR 2.1f // factor of P_Pop_Eps
#define P_POP_SCAL_EPS_INIT 0.01f
                        // [m] safety distance for collisions
#define P_CROSS_EPS 0.0001f
#define P_CROSS_EPS_QUAD (P_CROSS_EPS*P_CROSS_EPS)


/*** GLOBALS ***/
IVP_DOUBLE P_Pop_Eps;
IVP_DOUBLE P_Pop_Scal_Eps;
IVP_DOUBLE P_Pop_Too_Flat_Eps;

/*** METHODS AND SUBS ***/
int IVP_Poly_Point::point_num()
{
    if (!l_tetras) return -1;
    return this - this->l_tetras->points;
}

void IVP_Poly_Point::print(const char *text){
    printf("num %i: ",this->point_num());
    IVP_U_Point::print(text);
}

int IVP_Poly_Point::p(){
    this->print();
    return 0;
}

void p_del_double_triangles(IVP_Triangle **tri_a, IVP_Triangle **tri_b=NULL, IVP_Triangle **tri_c=NULL);
void p_del_double_triangles(IVP_Triangle **tri_a, IVP_Triangle **tri_b, IVP_Triangle **tri_c)
{
    if(tri_a) P_DELETE((*tri_a)->other_side);
    if(tri_b) P_DELETE((*tri_b)->other_side);
    if(tri_c) P_DELETE((*tri_c)->other_side);
    if(tri_a) P_DELETE((*tri_a));
    if(tri_b) P_DELETE((*tri_b));
    if(tri_c) P_DELETE((*tri_c));
}

int p_check_for_flat(IVP_Tri_Edge *a0, IVP_Tri_Edge *b0, IVP_DOUBLE eps)
{
	IVP_DOUBLE dist1 = -a0->triangle->tmp.gen.hesse.get_dist(b0->prev->start_point);
	IVP_DOUBLE dist2 = -b0->triangle->tmp.gen.hesse.get_dist(a0->prev->start_point);
	if( (dist1<eps) || (dist2<eps)){
	    //printf("%f %f\n",dist1,dist2);
	    return 1;
	}
	return 0;
}

void IVP_Triangle::calc_hesse()
{
    IVP_U_Point *p0, *p1, *p2;

    p0 = this->three_edges[0].start_point;
    p1 = this->three_edges[0].next->start_point;
    p2 = this->three_edges[0].prev->start_point;

    this->tmp.gen.hesse.calc_hesse(p0, p2, p1);
    this->tmp.gen.hesse.normize();
}

IVP_DOUBLE IVP_Triangle::calc_areasize()
{
    IVP_U_Hesse norm;
    IVP_U_Point *p0, *p1, *p2;

    p0 = this->three_edges[0].start_point;
    p1 = this->three_edges[0].next->start_point;
    p2 = this->three_edges[0].prev->start_point;

    norm.calc_hesse(p0, p2, p1);
    return norm.real_length() * 0.5f;
}

int IVP_Triangle::print(const char *comment)
{
    // OG problems with sun
    return 0;
    
    const char *obj_name = "unknown name";
    printf("%s '%s' T(%d %d %d) \n",
	   (comment)?comment:"",
	   (obj_name)?obj_name:"",
	   this->three_edges[0].start_point->point_num(), 
	   this->three_edges[0].next->start_point->point_num(),
	   this->three_edges[0].prev->start_point->point_num()
	);
    return 0;
}

IVP_Tri_Edge *IVP_Tri_Edge::other_side()
{
    int i;
    if(!this->triangle->other_side) return NULL;
    IVP_Tri_Edge *e = &this->triangle->other_side->three_edges[0];
    for(i=2; i>=0; i--){
	if(e->next->start_point == this->start_point) return e;
	e++;
    }
    return NULL;
}


void IVP_Tri_Edge::print(const char *text){
    if (!text) text = "";
    const char *name = start_point->get_real_object2()->get_name();
    if (!name) name = "noname";
    printf("%s '%s'	start_%lX: %i	",text,name, 0xff & ( ( (long)this->start_point->l_tetras  ) >>8), this->start_point->point_num());
    this->triangle->print("\n");
}

int IVP_Tri_Edge::p(){
    this->print();
    return 0;
}


int IVP_Object_Polygon_Tetra::check_concavity_and_manage(IVP_Tri_Edge *edge, P_CONVEXIFY_STATE )
{
    // prepo perfect links and perfect hidden flags
    // uses/sets opposite edge

    int flag = edge->check_concavity(edge->opposite);
    edge->opposite->concavity = edge->concavity;
    edge->tmp.gen.concav_flag = (char)flag;
    edge->opposite->tmp.gen.concav_flag = (char)flag;

    if((flag==-2)){
	move_edge_to_epsilon_hash(edge);
    }else{
	move_edge_to_normal_hash(edge);
    }
    return flag;
}

		       
int IVP_Tri_Edge::check_concavity(IVP_Tri_Edge *other_edge)
{
    // calcs concavity and test if edge could be popped
    // sets ggf. concavity and unsolvable of this
    // values for other_edge have to be copied manually afterwards

    // -2: partly overlapping
    // -1: identical
    // 0: convex
    // 1: concav
    
    if(!this){
	printf("NULL edge in check_concavity.\n");
    }

    
    // set concavity as measure for concavity volume (+ -> convex)
    IVP_DOUBLE concav = -this->triangle->tmp.gen.hesse.get_dist(other_edge->prev->start_point);
    {
	IVP_DOUBLE concav2 = -other_edge->triangle->tmp.gen.hesse.get_dist(this->prev->start_point);
	if (concav2 > concav){
	    concav = concav2;
	}
    }
    
    IVP_DOUBLE scal = this->triangle->tmp.gen.hesse.dot_product(&other_edge->triangle->tmp.gen.hesse);


    if(scal > 0.0f){
	// angle greater than 90 degrees or greater 270
	
	if( scal<1.0f-P_Pop_Scal_Eps || IVP_Inline_Math::fabsd(concav) >= P_POP_SURFACE_CONVEX_BLUR){
	    // different planes -> everything normal
	    
	    this->concavity = concav;

	    if(concav < 0){
		return 1; // real concav
	    }else{
		return 0; // real convex
	    }
	}else{
	    // same plane
	    concavity = 0.0f; // is convex (flat)
	    return 0;	
	}
    }else{
	// angle is less than 90 degrees
	this->concavity = concav;
	if (concav > 0){
		return 0; // real convex
	}
	// are triangles in the same plane ?
	if(concav < -P_Pop_Eps){
	    // different planes -> everything normal
		return 1; // real concav
	}else{
	    if(this->prev->start_point == other_edge->prev->start_point){
		CORE;
		return -1; // identical points
	    }
	    return -2;		// windschief
	}
    }
    CORE; // should not have happened
    return -12345; // dummy
}

P_Sur_2D_Triangle::P_Sur_2D_Triangle(int pn0, int pn1, int pn2)
{
    P_MEM_CLEAR(this);
    point_nums[0] = pn0;
    point_nums[1] = pn1;
    point_nums[2] = pn2;

#ifdef SUR_DEBUG
    printf("TRIANGLE GENERATED: %d, %d, %d\n",pn0, pn1, pn2);
#endif
}

P_Sur_2D_Point::P_Sur_2D_Point(int i_point_num)
{
    P_MEM_CLEAR(this);
    point_num = i_point_num;
}

P_Sur_2D_Point::~P_Sur_2D_Point()
{
    ;
}

P_Sur_2D_Line::P_Sur_2D_Line(P_Sur_2D_Point *sp, P_Sur_2D_Point *ep)
{
    P_MEM_CLEAR(this);
    start_point = sp;
    end_point = ep;

    if(sp && ep){
	this->delta_x = ep->k[0] - sp->k[0];
	this->delta_y = ep->k[1] - sp->k[1];
    }
#ifdef SUR_DEBUG
    printf("    Line generated: %d, %d\n", sp->point_num, ep->point_num);
#endif
}

P_Sur_2D_Line::~P_Sur_2D_Line()
{
    ; // end/start_points remain!
}

int P_Sur_2D_Line::point_lies_to_the_left(IVP_U_Point *i_point)
{
    IVP_DOUBLE hesse_dist = this->hesse_dist_to_point(i_point);
    if(hesse_dist <= P_DOUBLE_EPS) return 0; // on the other side
    if(hesse_dist <= P_Pop_Too_Flat_Eps) return 2; // not enough distance for triangle
    return 1;
}

int P_Sur_2D_Line::has_points(P_Sur_2D_Point *point_a, P_Sur_2D_Point *point_b)
{
    if(!point_b){
	if( (start_point==point_a) || (end_point==point_a) ) return 1;
	return 0;
    }
    
    if( (start_point==point_a)&&(end_point==point_b) ) return 1;
    if( (end_point==point_a)&&(start_point==point_b) ) return 1;
    return 0;
}

IVP_DOUBLE P_Sur_2D_Line::dist_to_point(IVP_U_Point *i_point)
{
    // att: maybe negative!
    
    IVP_DOUBLE norm = delta_y*delta_y + delta_x*delta_x;
    if(norm < P_DOUBLE_EPS){
	// line is just a point
	IVP_DOUBLE dx = this->start_point->k[0] - i_point->k[0];
	IVP_DOUBLE dy = this->start_point->k[1] - i_point->k[1];
	return dx*dx + dy*dy;
    }
    
    IVP_DOUBLE hesse_dist = delta_y * (start_point->k[0] - i_point->k[0]) +
	delta_x * (i_point->k[1] - start_point->k[1]); // not normized!

    return hesse_dist / IVP_Inline_Math::sqrtd(norm);
}

IVP_DOUBLE P_Sur_2D_Line::hesse_dist_to_point(IVP_U_Point *i_point)
{
    // att: maybe negative!
    
    IVP_DOUBLE hesse_dist = delta_y * (start_point->k[0] - i_point->k[0]) +
	delta_x * (i_point->k[1] - start_point->k[1]); // not normized!

    return hesse_dist;
}

int P_Sur_2D_Line::point_lies_in_interval(IVP_U_Point *i_point)
{
    // ATT: vorr: point lies on the extension of the line

    // try delta_x
    IVP_DOUBLE line_abs_delta = IVP_Inline_Math::fabsd(delta_x);
    if(line_abs_delta > P_DOUBLE_EPS){
	int dx = (delta_x > 0)? 1: -1;
	IVP_DOUBLE point_delta = (i_point->k[0] - start_point->k[0]) * dx;
	if(point_delta>P_CROSS_EPS && point_delta<line_abs_delta-P_CROSS_EPS){
	    return 1; // definitely inside
	}
	return 0; // outside or exactly on start/end point!!
    }

    // try delta_y
    line_abs_delta = IVP_Inline_Math::fabsd(delta_y);
    if(line_abs_delta > P_DOUBLE_EPS){
	int dy = (delta_y > 0)? 1: -1;
	IVP_DOUBLE point_delta = (i_point->k[1] - start_point->k[1]) * dy;
	if(point_delta>P_CROSS_EPS && point_delta<line_abs_delta-P_CROSS_EPS){
	    return 1; // definitely inside
	}
	return 0; // outside or exactly on start/end point!!
    }

    // line actually is a point
    return 0; // not inside
}

int P_Sur_2D_Line::overlaps_with_line(P_Sur_2D_Line *line_v)
{
#define P_OVERLAP_EPS 1E-8f   
    // ATT: vorr: lines are in a common line
    P_Sur_2D_Line *line_u = this;
    
    // try delta_x
    IVP_DOUBLE line_abs_delta = IVP_Inline_Math::fabsd(delta_x);
    if(line_abs_delta > P_OVERLAP_EPS){
	IVP_DOUBLE u_lower, u_higher;
	if(delta_x > 0.0f){
	    u_lower = line_u->start_point->k[0];
	    u_higher = line_u->end_point->k[0];
	}else{
	    u_lower = line_u->end_point->k[0];
	    u_higher = line_u->start_point->k[0];	    
	}
	u_lower += P_OVERLAP_EPS;
	u_higher -= P_OVERLAP_EPS;
	IVP_DOUBLE v_start = line_v->start_point->k[0];
	IVP_DOUBLE v_end = line_v->end_point->k[0];

	if( ((v_start<=u_lower) && (v_end<=u_lower)) ||
	    ((v_start>=u_higher) && (v_end>=u_higher)) ){
	    return 0; // no overlapping (but maybe 1 common point!)
	}
	return 1; // definitely overlapping
    }

    // try delta_y
    line_abs_delta = IVP_Inline_Math::fabsd(delta_y);
    if(line_abs_delta > P_OVERLAP_EPS){
	IVP_DOUBLE u_lower, u_higher;
	if(delta_y > 0.0f){
	    u_lower = line_u->start_point->k[1];
	    u_higher = line_u->end_point->k[1];
	}else{
	    u_lower = line_u->end_point->k[1];
	    u_higher = line_u->start_point->k[1];	    
	}
	u_lower += P_OVERLAP_EPS;
	u_higher -= P_OVERLAP_EPS;
	IVP_DOUBLE v_start = line_v->start_point->k[1];
	IVP_DOUBLE v_end = line_v->end_point->k[1];

	if( ((v_start<=u_lower) && (v_end<=u_lower)) ||
	    ((v_start>=u_higher) && (v_end>=u_higher)) ){
	    return 0; // no overlapping (but maybe 1 common point!)
	}
	return 1; // definitely overlapping
    }

    // line_u actually is a point
    return line_v->point_lies_in_interval(line_u->start_point);
#undef P_OVERLAP_EPS    
}

int P_Sur_2D_Line::is_crossing_line(P_Sur_2D_Line *line_v)
{
    // returns 1 if line_u (=this) crosses line_v, otherwise 0.

    // it doesn't count as a crossing, if lines have a
    // common end/start point.

   
    P_Sur_2D_Line *line_u = this;
#ifdef SUR_DEBUG
    printf("    crossing (%d, %d) with (%d, %d): ", line_u->start_point->point_num,
	   line_u->end_point->point_num, line_v->start_point->point_num,
	   line_v->end_point->point_num);
#endif

    // delta values for quickness and readability
#define u_dx (line_u->delta_x)
#define u_dy (line_u->delta_y)
#define v_dx (line_v->delta_x)
#define v_dy (line_v->delta_y)
#define DET_EPS 0.000000001f



    // calc determinante of (inhomogene) linear system
    IVP_DOUBLE D = v_dx * u_dy - u_dx * v_dy;
    if(IVP_Inline_Math::fabsd(D) < DET_EPS){
#ifdef SUR_DEBUG
	printf("det < det_eps: ");
#endif	
	// linear dependency: on the line or just parallel?
	if(IVP_Inline_Math::fabsd(line_u->dist_to_point(line_v->start_point)) < P_CROSS_EPS_QUAD){
#ifdef SUR_DEBUG
	    printf("on the line: ");
#endif
	    if( line_u->overlaps_with_line(line_v) ){
#ifdef SUR_DEBUG
		printf("overlapping (crossing)!\n");
#endif
		return 1; // lines are overlapping
	    }else{
#ifdef SUR_DEBUG
		printf("not overlapping (no crossing)!\n");
#endif		    
		return 0;
	    }
	}else{
#ifdef SUR_DEBUG
	    printf("really parallel.\n");
#endif	
	    return 0; // really parallel
	}
    }   
 
    // check whether lines have common end/start points
    if( (line_u->start_point == line_v->end_point)   ||
        (line_u->start_point == line_v->start_point) ||
        (line_u->end_point   == line_v->start_point) ||
        (line_u->end_point   == line_v->end_point) ){
#ifdef SUR_DEBUG
	printf("common end/start point -> no crossing\n");
#endif
	return 0; // shall not count as crossing
    }

    // calc delta of start_points ('origins')
    IVP_DOUBLE o_dx = line_v->start_point->k[0] - line_u->start_point->k[0];
    IVP_DOUBLE o_dy = line_v->start_point->k[1] - line_u->start_point->k[1];
    
    IVP_DOUBLE left_border, right_border;
    if(D>0.0f){
	left_border = 0.0f;
	right_border = D;
    }else{
	left_border = D;
	right_border = 0.0f;	
    }
 
    // check crossing on line_u
    IVP_DOUBLE u_t = v_dx*o_dy - v_dy*o_dx; // u_sp + u_t*D*(u_ep-u_sp) = crossing
#ifdef SUR_DEBUG
	printf("u_t %g: ", u_t/D);
#endif	
    if(u_t < left_border || u_t > right_border){
#ifdef SUR_DEBUG
	printf("no crossing.\n");
#endif	
	return 0; // crosses beyond line interval
    }
    // check crossing on line_v
    IVP_DOUBLE v_t = u_dx*o_dy - o_dx*u_dy; // v_sp + v_t*D*(v_ep-v_sp) = crossing
#ifdef SUR_DEBUG
	printf("v_t %g: ", v_t/D);
#endif	
    if(v_t < left_border || v_t > right_border){
#ifdef SUR_DEBUG
	printf("no crossing.\n");
#endif	
	return 0; // crosses beyond line interval
    }
    // real crossing
#ifdef SUR_DEBUG
    printf("CROSSING.\n");
#endif	
    return 1;
    
#undef u_dx  
#undef u_dy  
#undef v_dx  
#undef v_dy  
#undef DET_EPS    
}

P_Sur_2D::P_Sur_2D(IVP_Object_Polygon_Tetra *tetras_, IVP_Template_Surface *sur)
{
    P_MEM_CLEAR(this);
    orig_tetras = tetras_;
    orig_surface = sur;
}

P_Sur_2D::~P_Sur_2D()
{
    P_FREE(line_array);
    P_FREE(point_array);
    while(lines.first){
	P_Sur_2D_Line *l = lines.first;
	P_DELETE(l->start_point);
	P_DELETE(l->end_point);
	lines.remove(l);
	P_DELETE(l);
    }
    P_Sur_2D_Triangle *tri;
    while((tri = triangles.first)){
	triangles.remove(tri);
	P_DELETE(tri);
    }
}

IVP_ERROR_STRING P_Sur_2D::calc_line_representation()
{
    IVP_Template_Surface *sur = this->orig_surface;
    if(!sur){
	return "calc_line_representation: no orig_surface specified!\n";
    }
    

    IVP_Object_Polygon_Tetra *pop = orig_tetras;
    
#ifdef SUR_DEBUG    
    printf("\n\n\n***********************\n"
	   "POP: %d, SURFACE_NUM %d with %d points\n\n", (int)pop, sur->get_surface_index(),
	   sur->n_lines);
#endif    
    // 3d->2d trafo: skip koord with greatest ohesse extent
    IVP_U_Point *hesse = &sur->normal;
    int max_koord = 0;
    IVP_DOUBLE max_koord_val = IVP_Inline_Math::fabsd(hesse->k[0]);
    if(IVP_Inline_Math::fabsd(hesse->k[1]) > max_koord_val){
	max_koord_val = IVP_Inline_Math::fabsd(hesse->k[1]);
	max_koord = 1;
    }
    if(IVP_Inline_Math::fabsd(hesse->k[2]) > max_koord_val){
	max_koord_val = IVP_Inline_Math::fabsd(hesse->k[2]);
	max_koord = 2;
    }

    
    int td_k0, td_k1;
    switch(max_koord){
    case 0:
	td_k0 = 1; td_k1 = 2;
	break;
    case 1:
	td_k0 = 0; td_k1 = 2;
	break;
    case 2:
	td_k0 = 0; td_k1 = 1;
	break;
    default:
	td_k0 = td_k1 = 0; // compiler lullaby
	CORE;
    }
    
    // is trafo mirrored ?
    int trafo_is_mirrored = 0;
    if(hesse->k[max_koord] < 0.0f) trafo_is_mirrored = 1;

#ifdef SUR_DEBUG
    printf("hesse: %g %g %g; max_koord: %d; take koords %d %d; mirrored: %d\n",
	   hesse->k[0], hesse->k[1], hesse->k[2], max_koord, td_k0, td_k1, trafo_is_mirrored);
#endif
    if(max_koord==1) trafo_is_mirrored = 1-trafo_is_mirrored;

    // transfer line and point info
    P_Sur_2D_Point **point_hash_array = (P_Sur_2D_Point**)p_calloc(pop->n_points,
							    sizeof(P_Sur_2D_Point*));
    P_FREE(this->line_array);
    this->line_array = (P_Sur_2D_Line **)p_calloc(sur->n_lines,
							    sizeof(P_Sur_2D_Line*));
    P_FREE(this->point_array);
    this->point_array = (P_Sur_2D_Point **)p_calloc(sur->n_lines,
							    sizeof(P_Sur_2D_Point*));
    int point_count = 0;
    int i;
    for(i=sur->n_lines-1; i>=0; --i){

	// read line info
	int line_num = sur->lines[i];
	IVP_Template_Line *line = &pop->template_polygon->lines[line_num];

	int revert = sur->revert_line[i];
	if(trafo_is_mirrored) revert = 1 - revert;
	int start_point_num = line->p[revert];
	int end_point_num = line->p[1-revert];

	// gen 2D line and points
	P_Sur_2D_Point *start_point = point_hash_array[start_point_num];
	if(!start_point){
	    start_point = new P_Sur_2D_Point(start_point_num);
	    point_hash_array[start_point_num] = start_point;
	    point_array[point_count++] = start_point;
	}
	start_point->set(pop->points[start_point_num].k[td_k0],
		       pop->points[start_point_num].k[td_k1], 0.0f);
	
	P_Sur_2D_Point *end_point = point_hash_array[end_point_num];
	if(!end_point){
	    end_point = new P_Sur_2D_Point(end_point_num);
	    point_hash_array[end_point_num] = end_point;
	    point_array[point_count++] = end_point;
	}
	end_point->set(pop->points[end_point_num].k[td_k0],
		       pop->points[end_point_num].k[td_k1], 0.0f);
	
	P_Sur_2D_Line *td_line = new P_Sur_2D_Line(start_point, end_point);
	start_point->line_ref = td_line;
	IVP_ASSERT(td_line);
	this->lines.insert(td_line);
	this->line_array[i] = td_line;// bookmark for posterior deletion
    }

    IVP_ASSERT(point_count == sur->n_lines);
    
    // clean up
    P_FREE(point_hash_array);
    
    return IVP_NO_ERROR;
}

int p_count_reachable(P_Sur_2D_Point *i_point)
{
    // counts (and marks) all points not yet reached
    int reached_counter = 0;

    P_Sur_2D_Point *point;
    for(point=i_point; point && !point->was_reached; point=point->line_ref->end_point){
	point->was_reached = 1;
	reached_counter++;
    }
    return reached_counter;
}

IVP_DOUBLE p_ab_quad_length(P_Sur_2D_Point *pa, P_Sur_2D_Point *pb, P_Sur_2D_Point *pc)
{
    // return added lengths of both triangle sides
    return pc->quad_distance_to(pa) + pc->quad_distance_to(pb);
}

IVP_ERROR_STRING P_Sur_2D::calc_triangle_representation()
{
    // opt: check whether there are any islands in the surface
    int having_islands = 0;
    int triangle_count = 0;
    int n_lines = this->orig_surface->n_lines;
    int reached_points_counter = p_count_reachable(this->lines.first->start_point);

    IVP_U_Min_Hash ab_min_hash(16);
    P_Sur_2D_Line *td_base_line;
   
    if(reached_points_counter < n_lines){
	having_islands = 1;
    }
#ifdef SUR_DEBUG
    printf("   Having islands would be: %d\n", having_islands);
#endif
    having_islands = 1; // forced

    P_Sur_2D_Line *td_ca_line = NULL;
    P_Sur_2D_Line *td_bc_line = NULL;
    
    // for all lines
    int wrong_flag = 0;
    int loop_counter = 0;
    while((td_base_line=this->lines.first)){
#ifdef SUR_DEBUG
	printf("TAKE BASE LINE: (%d %d)\n", td_base_line->start_point->point_num,
	       td_base_line->end_point->point_num);
#endif
	if (loop_counter++ > 100){ //@@@@SF: changed from 100000 to 100 [05.03.2000]
	    //printf("Degenerated polygon found:\n");
//	    P_Sur_2D_Point *point_a = td_base_line->start_point; // a b c form a triangle
//	    P_Sur_2D_Point *point_b = td_base_line->end_point;
//	    printf("  %f %f %f\n", point_a->k[0], point_a->k[1], point_a->k[2]);
//	    printf("  %f %f %f\n", point_b->k[0], point_b->k[1], point_b->k[2]);
	    P_Sur_2D_Line *td_point_line;
 	    for(td_point_line=this->lines.first; td_point_line; td_point_line=td_point_line->next){
		P_Sur_2D_Point *point_c = td_point_line->start_point;
		printf("  %f %f %f\n", point_c->k[0], point_c->k[1], point_c->k[2]);
	    }
	    return "Cannot convert";
	}
	// put points into min_hash
	while (ab_min_hash.find_min_elem()){
	    ab_min_hash.remove_min();
	}
	P_Sur_2D_Point *point_a = td_base_line->start_point; // a b c form a triangle
	P_Sur_2D_Point *point_b = td_base_line->end_point;
	P_Sur_2D_Line *td_point_line;
 	for(td_point_line=this->lines.first; td_point_line; td_point_line=td_point_line->next){
	    P_Sur_2D_Point *point_c = td_point_line->start_point;
	    ab_min_hash.add((void *)td_point_line,p_ab_quad_length(point_a, point_b, point_c));    
	}

	// now try triangles in ascending side length order
	for(td_point_line=(P_Sur_2D_Line *)ab_min_hash.find_min_elem();
	    td_point_line;
	    ab_min_hash.remove_min(), td_point_line=(P_Sur_2D_Line *)ab_min_hash.find_min_elem()){
	    if(td_point_line == td_base_line) continue;	    
	    if(td_point_line->start_point == td_base_line->end_point) continue;
#ifdef SUR_DEBUG
	    printf("  Trying point %d: ", td_point_line->start_point->point_num);
#endif		

	    // set koords
	    // point_a and point_b are already set
	    P_Sur_2D_Point *point_c = td_point_line->start_point;
	    
	    // check side: point_c must be on the left!!
	    int dist_flag;
	    if(! (dist_flag=td_base_line->point_lies_to_the_left(point_c)) ){
#ifdef SUR_DEBUG
		if(dist_flag==2){
		    printf("lies to near for triangle!");
		}else{
		    printf("lies on wrong side.\n");			    
		}
#endif		
		continue;
	    }
#ifdef SUR_DEBUG
	    printf("side OK.\n");
#endif		

	    int skip_this_point = 0;

	    // provide the two other triangle sides as lines (bc & ca)
#ifdef SUR_DEBUG
		printf("    check sides of triangle:\n");
#endif		
	    delete td_ca_line;
	    td_ca_line = new P_Sur_2D_Line(point_c, point_a);	    
	    delete td_bc_line;
	    td_bc_line = new P_Sur_2D_Line(point_b, point_c);

            // for all lines: check crossing
	    P_Sur_2D_Line *td_cross_line;
	    for(td_cross_line=this->lines.first; td_cross_line; td_cross_line=td_cross_line->next){
		// crosses ca ?
		if(!td_cross_line->has_points(point_c, point_a)){ // !'identical'
		    if(td_cross_line->is_crossing_line(td_ca_line)){
			skip_this_point = 1;
			break;
		    }
		}
		
		// crosses bc ?
		if(!td_cross_line->has_points(point_b, point_c)){ // !'identical'
		    if(td_cross_line->is_crossing_line(td_bc_line)){
			skip_this_point = 1;
			break;
		    }
		}
	    }
	    if(skip_this_point){
		if(!ab_min_hash.find_min_elem()){
		    printf("Couldn't find a matching point to baseline!\n");
		    CORE;
		}
#ifdef SUR_DEBUG
		printf("    ...giving up point %d because of crossing. next one!\n",
		       td_point_line->start_point->point_num);
#endif		
		continue;
	    }

	    // something inside?
	    if(having_islands){
		P_Sur_2D_Line *td_inside_line;
		for(td_inside_line=this->lines.first; td_inside_line; td_inside_line=td_inside_line->next){
		    // start_point inside triangle?
		    P_Sur_2D_Point *sp = td_inside_line->start_point;
		    if( td_base_line->point_lies_to_the_left(sp) &&
			td_bc_line->point_lies_to_the_left(sp)   &&
			td_ca_line->point_lies_to_the_left(sp)){
			// alas, triangle not empty...
			skip_this_point = 1;
			break;
		    }
		}
	    }
	    if(skip_this_point){
		if(!td_point_line->next){
		    printf("Couldn't find a matching point to baseline!\n");
		    CORE;
		}
#ifdef SUR_DEBUG
		printf("    ...giving up point because of point(s) inside. next one!\n");
#endif		
		continue;
	    }

	    // take triangle
	    P_Sur_2D_Triangle *triang = new P_Sur_2D_Triangle(point_a->point_num,
							      point_c->point_num,
							      point_b->point_num);
	    this->triangles.insert(triang);
#ifdef SUR_DEBUG	    
	    printf("Triangle %d: %d, %d, %d\n",triangle_count,
		   point_a->point_num, point_c->point_num, point_b->point_num);
#endif
	    triangle_count++;
	    
	    // delete used lines      , maybe faster with hash
	    this->lines.remove(td_base_line);
	    
#ifdef SUR_DEBUG
	    printf("    line removed: %d, %d\n", td_base_line->start_point->point_num,
		   td_base_line->end_point->point_num);
#endif		    
	    P_Sur_2D_Line *td_ident_line;
	    P_Sur_2D_Line *td_ident_line_next;
	    int ca_removed = 0; // flags
	    int bc_removed = 0;
	    P_DELETE(td_base_line);
	    
	    for(td_ident_line=this->lines.first; td_ident_line; td_ident_line=td_ident_line_next){
		td_ident_line_next = td_ident_line->next;
		// if( td_ident_line->has_points(point_c, point_a) && td_ident_line->has_points(point_c, point_b) ){
		//    printf("scheibenkleister!\n");
		// }
		if( td_ident_line->has_points(point_c, point_a) ){
		    this->lines.remove(td_ident_line);
#ifdef SUR_DEBUG
		    printf("    line removed: %d, %d\n", td_ident_line->start_point->point_num,
			   td_ident_line->end_point->point_num);
#endif		    
		    ca_removed = 1;
		    P_DELETE(td_ident_line);
		}
		if( td_ident_line && td_ident_line->has_points(point_c, point_b) ){
		    this->lines.remove(td_ident_line);
#ifdef SUR_DEBUG
		    printf("    line removed: %d, %d\n", td_ident_line->start_point->point_num,
			   td_ident_line->end_point->point_num);
#endif		    
		    bc_removed = 1;
		    P_DELETE(td_ident_line);
		}
	    }

	    // insert remaining triangle as new lines
	    if(!ca_removed){
		P_Sur_2D_Line *td_new_line = new P_Sur_2D_Line(point_a, point_c);
		this->lines.insert(td_new_line); // append is better
	    }
	    if(!bc_removed){
		P_Sur_2D_Line *td_new_line = new P_Sur_2D_Line(point_c, point_b);
		this->lines.insert(td_new_line); // append is better
	    }

	    // proceed to next base line for next triangle
	    break;
	} // end for all points
    } // end for all base_lines

    // clean up
    P_DELETE(td_ca_line);
    P_DELETE(td_bc_line);

    // lines and points are no longer needed
    int i;
    for(i=this->orig_surface->n_lines-1; i>=0; --i){
//	delete this->line_array[i];
	delete this->point_array[i];
    }
    
    if(wrong_flag){
	return "something went wrong in calc_triangle_representation! break...\n";
    }
    return IVP_NO_ERROR;
}

void ivp_check_for_opposite(IVP_Hash *hash, IVP_Poly_Point *p0, IVP_Poly_Point *p1, IVP_Tri_Edge *edge)
{
    IVP_Poly_Point *hashval[2];
    if(p0<p1){
	hashval[0] = p0;
	hashval[1] = p1;
    }else{
	hashval[0] = p1;
	hashval[1] = p0;
    }
    IVP_Tri_Edge *edge2;
    edge2 = (IVP_Tri_Edge *)hash->find((char *)(&hashval[0]));
    if(!edge2){
	// add
	hash->add((char *)(&hashval[0]), (void *)edge);	
    }else{
	// opposites now known -> add
	edge->opposite = edge2;
	edge2->opposite = edge;	
    }
}

IVP_Object_Polygon_Tetra::IVP_Object_Polygon_Tetra(IVP_Template_Polygon *i_temp_pop)
{
    P_MEM_CLEAR(this);
    this->template_polygon = i_temp_pop;

    this->n_points = i_temp_pop->n_points;
    this->points = (IVP_Poly_Point *)p_calloc(sizeof(IVP_Poly_Point), this->n_points);
    int i;
    for (i=0;i<this->n_points;i++){
	IVP_U_Point *p = &i_temp_pop->points[i];
	this->points[i].set( p );
	this->points[i].l_tetras = this;
    }


    this->n_surfaces = i_temp_pop->n_surfaces;
    this->surfaces = (IVP_Poly_Surface *)p_calloc(sizeof(IVP_Poly_Surface), this->n_surfaces);
    for (i=0;i<this->n_surfaces;i++){
	this->surfaces[i].set( &i_temp_pop->surfaces[i], this );
    }
    
    template_polygon = NULL;
}

void IVP_Poly_Surface::set(IVP_Template_Surface *templ_sur,
			  IVP_Object_Polygon_Tetra *i_tetras)
{
    this->tetras = i_tetras;
    IVP_USE(templ_sur);
}

int IVP_Poly_Surface::get_surface_index(){
    return this - tetras->surfaces;
}


IVP_Object_Polygon_Tetra::~IVP_Object_Polygon_Tetra()
{
    this->free_triangles();
    P_FREE(this->surfaces);
    P_FREE(this->points);
}


IVP_ERROR_STRING IVP_Object_Polygon_Tetra::make_triangles()
{
    int n_edges;
    IVP_Template_Surface *sur;
    IVP_Poly_Point *po, *po2, *po3;
    IVP_Triangle *tri;
    IVP_Hash *hash;
    IVP_Template_Polygon *templ = template_polygon;
    int num_of_edges;
    
    n_edges = 0;
    num_of_edges = 6 * (templ->n_points-2); // accurately calculated
    //num_of_edges = 6 * templ->n_points - 12;    //lwss - from debug bins
    //hash = new IVP_Hash(num_of_edges/*size*/, 8 /*keylen*/, 0/*notfound*/);
    hash = new IVP_Hash(num_of_edges/*size*/, sizeof(void*)*2 /*keylen*/, 0/*notfound*/); //lwss - from debug bins
    {
	int i;    
	IVP_Tri_Edge *edge;
	for(i=templ->n_surfaces-1, sur=&templ->surfaces[0]; i>=0; i--, sur++){
	    // split plane into triangles and insert them
	    P_Sur_2D *td_sur = new P_Sur_2D(this,sur);
	    IVP_ERROR_STRING error = td_sur->calc_line_representation();
	    if(error){
		printf("make_triangles:calc_line_representation: %s\n", error);
		return "No 2d representation";
	    }
	    error = td_sur->calc_triangle_representation();
	    if(error){
		printf("make_triangles:calc_triangle_representation: %s\n", error);
		return "no 3d representation";
	    }
	    
	    P_Sur_2D_Triangle *td_tri;
	    
	    for(td_tri=td_sur->triangles.first; td_tri; td_tri=td_tri->next){
		po = &this->points[td_tri->point_nums[2]];
		po2 = &this->points[td_tri->point_nums[1]];
		po3 = &this->points[td_tri->point_nums[0]];

		// clockwise ?
		{
		    IVP_U_Point *hesse = &sur->normal;
		    IVP_U_Point v0, v1;
		    v0.subtract(po2, po);
		    v1.subtract(po3, po);
		    IVP_U_Point cross;
		    cross.calc_cross_product(&v0, &v1);
		    IVP_DOUBLE scal = cross.dot_product(hesse);
		    if(scal<0.0f){
			IVP_Poly_Point *h=po3;
			po3 = po2;
			po2 = h;
			//printf("Points swapped for clockwise.\n");		    
		    }
		}
		
		// make physical triangle
		tri = new IVP_Triangle();
		this->triangles.insert(tri);
		tri->flags.is_terminal = 1; // might be changed later on
		int sur_index = sur->get_surface_index();
		tri->ivp_surface = &surfaces[sur_index];
		tri->flags.is_hidden = 0;
		
		// 3 edges in uzs
		edge = &tri->three_edges[0];
		n_edges += 3;
		
		// 1. edge
//		tri->edge = edge;
		edge->start_point = po;
		edge->triangle = tri;
		edge->next = edge+1;
		edge->prev = edge+2;
		edge->behind = NULL; // terminal!
		ivp_check_for_opposite(hash, po, po2, edge);
		
		// 2. edge
		edge++;
		edge->start_point = po2;
		edge->triangle = tri;
		edge->next = edge+1;
		edge->prev = edge-1;
		edge->behind = NULL; // terminal!
		ivp_check_for_opposite(hash, po2, po3, edge);
		
		// 3. edge
		edge++;
		edge->start_point = po3;
		edge->triangle = tri;
		edge->next = edge-2;
		edge->prev = edge-1;
		edge->behind = NULL; // terminal!
		ivp_check_for_opposite(hash, po3, po, edge);
	    }
	    P_DELETE(td_sur);
	}
	P_DELETE(hash);
    }
//	this->check_konsistency_of_triangles();

	IVP_Triangle *otri;
	for(otri=this->triangles.first; otri; otri=otri->next){
	    // make physical triangle - backside
	    tri = new IVP_Triangle();
	    this->triangles.insert(tri);
	    tri->flags.is_terminal = 1;
	    tri->ivp_surface = NULL;
	    tri->flags.is_hidden = 1;
	    tri->other_side = otri;
	    otri->other_side = tri;

	    n_edges += 3;

	    int i;
	    for(i=2; i>=0; i--){
		IVP_Tri_Edge *edge = &tri->three_edges[i];
		edge->start_point = otri->three_edges[i].next->start_point;
		edge->triangle = tri;
		edge->next = &tri->three_edges[(i+2)%3];
		edge->prev = &tri->three_edges[(i+1)%3];
		edge->behind = otri->three_edges[i].opposite;
		IVP_Tri_Edge *opp = edge->other_side()->opposite->other_side();
		if(opp){
		    edge->opposite = opp;
		    opp->opposite = edge;
		}
		otri->three_edges[i].opposite->behind = &tri->three_edges[i];
	    }
	    for(i=2; i>=0; i--){
		IVP_Tri_Edge *edge = &tri->three_edges[i];
		this->add_edge_into_point_to_edge_hash(edge);
	    }
	}
	return NULL;
	// this->check_konsistency_of_triangles();
}


IVP_ERROR_STRING IVP_Object_Polygon_Tetra::final_convexify_check()
{
    // checks wether the concavity of edges is correctly defined
    IVP_ERROR_STRING error_flag = 0;
    
    IVP_Object_Polygon_Tetra *ph = this;
    IVP_Triangle *tri;
    int i;
    int n_triangles_check = 0;
    for(tri=ph->triangles.first; tri; tri=tri->next){
	n_triangles_check++;

        // check triangle features
	// ...
	
	// check edges
	IVP_Tri_Edge *e = &tri->three_edges[0];
	for(i=0; i<3; i++, e=e->next){
	    int conc_flag = e->check_concavity(e->opposite);
	    if(conc_flag == -2){
		error_flag = "error";
		printf("convex_test: tri->edge: tmp.gen.concav_flag == -2 (partly overlapping)"); tri->print("\n");	    		
	    }
	    if(conc_flag == -1){
		error_flag = "error";
		printf("convex_test: tri->edge: tmp.gen.concav_flag == -1 (identic)"); tri->print("\n");	    		
	    }
	    if(conc_flag == 0){
		// convex: are neighbour and myself unhidden?
		if(!tri->flags.is_terminal && (e->opposite->triangle->flags.is_hidden ||
		   e->triangle->flags.is_hidden)){
		    printf("convex_test:  warning edge is convex, but neighbours are hidden."); tri->print("\n");
		}		
	    }
	    if(conc_flag == 1){
		// concav: are neighbour and myself hidden?
		if(!e->opposite->triangle->flags.is_hidden ||
		   !e->triangle->flags.is_hidden){
		    if (e->concavity < - P_POP_SURFACE_CONVEX_BLUR){
			printf("convex_test:  edge is concav %f, but neighbours are unhidden.",e->concavity); tri->print("\n");
			error_flag = "error";
		    }
		}		
	    }
	}
    }
    if(!error_flag){
	//printf("final_convexify_check: OK: %i triangles, %i extrapoints.\n", n_triangles_check, n_extra_points);
    }else{
	printf("final_convexify_check: ERRORS: %i triangles, %i extrapoints.\n", n_triangles_check, n_extra_points);
    }
    return error_flag;
}


IVP_ERROR_STRING IVP_Object_Polygon_Tetra::check_konsistency_of_triangles()
{
    // tests consistency of object's physical elements

    IVP_ERROR_STRING error_flag = 0;
    
    IVP_Object_Polygon_Tetra *ph = this;
    IVP_Triangle *tri;
    IVP_Tri_Edge *edge;
    int i;
    int n_triangles_check = 0;
    for(tri=ph->triangles.first; tri; tri=tri->next){
	n_triangles_check++;

	if(tri != tri->other_side->other_side){
		error_flag = "error";
		printf("physic_test: tri != tri->other_side->other_side in triangle "); tri->print("\n");	    
	}
	if( (tri->three_edges[0].next != &tri->three_edges[1] ||
	     tri->three_edges[1].next != &tri->three_edges[2] ||
	     tri->three_edges[2].next != &tri->three_edges[0] ) &&

	    (tri->three_edges[0].prev != &tri->three_edges[1] ||
	     tri->three_edges[1].prev != &tri->three_edges[2] ||
	     tri->three_edges[2].prev != &tri->three_edges[0] )
	    ){
		error_flag = "error";
		printf("physic_test: edges do not properly form a ring in triangle "); tri->print("\n");
	}
	if(tri->calc_areasize() < 1E-6f){
		error_flag = "error";
		printf("pysic_test: tri->calc_areasize() < 1E-6f in triangle: %g", tri->calc_areasize()); tri->print("\n");	    	    
	}

	// check tetraeder, if hidden and
	// check consistency of hidden flag in a tetraeder
	// (tetraeder -> all hidden, no tetraeder -> all unhidden)
	IVP_Tri_Edge *e = &tri->three_edges[0];
	if(tri->flags.is_hidden){
	    // check wether the neighbors do form a tetraeder
	    if( !tri->flags.is_terminal && ( (e->opposite->prev->start_point != e->next->opposite->prev->start_point)||
		(e->opposite->prev->start_point != e->prev->opposite->prev->start_point))   )
	    {
		error_flag = "error";
		printf("physic_test:  triangle is hidden, but neighbours don't form a tetraeder."); tri->print("\n");		
	    }
		// are all triangles inside this tetraeder hidden?
	    if(!(e->opposite->triangle->flags.is_hidden &&
	       e->next->opposite->triangle->flags.is_hidden &&
	       e->prev->opposite->triangle->flags.is_hidden)){
		error_flag = "error";
		printf("physic_test:  triangle is hidden, but not all neighbours are hidden."); tri->print("\n");
	    }
	}else{
	    if( e->opposite->triangle->flags.is_hidden ||
	        e->next->opposite->triangle->flags.is_hidden ||
		e->prev->opposite->triangle->flags.is_hidden){
		error_flag = "error";
		printf("physic_test:  triangle is not hidden, but a neighbour is hidden."); tri->print("\n");
	    }
	}

	
	// check edges
	edge = &tri->three_edges[0];
	for(i=0; i<3; i++, edge=edge->next){

	    // check if any hidden edge is in any hash
	    if(tri->flags.is_hidden && edge->tmp.gen.hash_class){
		error_flag = "error";
		printf("physic_test: tri->is_hidden && in hash"); tri->print("\n");
	    }
	    
	    if(edge->opposite->opposite != edge){
		error_flag = "error";
		printf("physic_test: edge->opposite->opposite != edge in triangle "); edge->print("\n");
	    }
	    if(edge->start_point != edge->opposite->next->start_point){
		error_flag = "error";
		printf("physic_test: edge->start_point != edge->opposite->next->startpoint in triangle "); edge->print("\n");
	    }
	    if(edge != edge->other_side()->other_side()){
		error_flag = "error";
		printf("physic_test: edge != edge->other_side()->other_side() in triangle "); edge->print("\n");
	    }
	    if(edge->prev->start_point == edge->opposite->prev->start_point){
		error_flag = "error";
		printf("physic_test: edge->prev->sp == edge->opp->prev->sp in triangle "); edge->print("\n");
	    }
	    if(edge->triangle == edge->opposite->triangle){
		error_flag = "error";
		printf("physic_test: edge->triangle == edge->opposite->triangle in triangle "); edge->print("\n");
	    }
	    if(edge->behind->other_side()->behind != edge->other_side()){
		error_flag = "error";
		printf("physic_test: edge->behind->other_side()->behind = edge->other_side in triangle "); edge->print("\n");
	    }
	    if(edge->triangle != tri){
		error_flag = "error";
		printf("physic_test: edge->triangle != tri in triangle "); edge->print("\n");
	    }
	    if((edge->behind->start_point != edge->start_point) || (edge->behind->next->start_point != edge->next->start_point)){
		error_flag = "error";
		printf("physic_test: (edge->behind->sp != edge->sp) || (edge->behind->next->sp != edge->next->sp) in triangle "); tri->print("\n");
	    }
	}
    }
    
    if(ph->triangles.len != n_triangles_check){
	printf("physik_test: Anzahl der Triangles (%d) != n_triangles (%d)!\n",
	       n_triangles_check, ph->triangles.len);
		error_flag = "error";
    }
    
    return error_flag;
}

void IVP_Object_Polygon_Tetra::free_triangles()
{
    /* Warning: Does not check references to tri edge !! */
    IVP_Triangle *tri;
    for (tri = triangles.first; tri; tri = triangles.first){
	triangles.remove(tri);
	delete tri;
    }
    IVP_Extra_Point *ep,*nep;
    for (ep = extra_points; ep ;ep = nep){
	nep = ep->next;
	delete ep;
    }
    extra_points= 0;
}

void IVP_Object_Polygon_Tetra::calc_concavities()
{
    // check for concav edges, count and calc them

    // calc concavity values
    IVP_Triangle *tri;
    const IVP_DOUBLE uninit = 123456.0f;
    for (tri = this->triangles.first; tri; tri = tri->next){
	IVP_Tri_Edge *edge = &tri->three_edges[0];
	for(int i=2; i>=0; edge=edge->next, i--){
	    edge->concavity = uninit;
	}
    }
	
    for (tri = this->triangles.first; tri; tri = tri->next){
	if(tri->flags.is_hidden) continue;
	int i;
	IVP_Tri_Edge *edge = &tri->three_edges[0];
	for(i=2; i>=0; edge=edge->next, i--){
	    if((edge->concavity == uninit)
	       &&(!edge->triangle->flags.is_hidden)
	       &&(!edge->opposite->triangle->flags.is_hidden)) {
		int flag = this->check_concavity_and_manage(edge, P_CONVEXIFY_STATE_INIT);
		if(flag == -1){
		    printf("Terminal object has identical triangles!!!\n");
		    CORE;
		}
	    }
	}
    }
}

IVP_DOUBLE IVP_Object_Polygon_Tetra::rate_tri_edge(IVP_Tri_Edge *edge){
    IVP_U_Point *p0 = edge->prev->start_point;
    IVP_U_Point *p1 = edge->opposite->prev->start_point;
    return p0->quad_distance_to(p1);
}

void IVP_Object_Polygon_Tetra::move_edge_to_problem_hash(IVP_Tri_Edge *edge){
    IVP_Tri_Edge *opp = edge->opposite;
    this->remove_edge_from_min_list(edge);

    this->tetra_intrude->checkout_edge(edge);
    this->tetra_intrude->checkout_edge(edge->opposite);

    if (opp<edge) edge = opp;
    if (edge->triangle->flags.is_hidden) return;
    this->tetra_intrude->checkin_edge(edge); // intrusion check with visible edges only

    IVP_DOUBLE rating = -edge->start_point->quad_distance_to(edge->next->start_point); // length
    add_edge_to_min_list(edge,P_HASH_CLASS_PROBLEM,rating);
}

void IVP_Object_Polygon_Tetra::move_edge_to_convex_intrude_hash(IVP_Tri_Edge *edge){
    IVP_Tri_Edge *opp = edge->opposite;
    this->remove_edge_from_min_list(edge);

    if (opp<edge) edge = opp;

    IVP_DOUBLE rating = -edge->start_point->quad_distance_to(edge->next->start_point); // length
    add_edge_to_min_list(edge, P_HASH_CLASS_CONVEX_INTRUDE, rating);
}

void IVP_Object_Polygon_Tetra::move_edge_to_epsilon_hash(IVP_Tri_Edge *edge){
    IVP_Tri_Edge *opp = edge->opposite;
    this->remove_edge_from_min_list(edge);

    this->tetra_intrude->checkout_edge(edge);
    this->tetra_intrude->checkout_edge(edge->opposite);

    if (opp<edge) edge = opp;
    if (edge->triangle->flags.is_hidden) return;
    this->tetra_intrude->checkin_edge(edge); // intrusion check with convex edges only

    IVP_DOUBLE rating = this->rate_tri_edge(edge);
    add_edge_to_min_list(edge,P_HASH_CLASS_EPSILON,rating);
}

void IVP_Object_Polygon_Tetra::move_edge_to_normal_hash(IVP_Tri_Edge *edge){
    this->remove_edge_from_min_list(edge);
    this->tetra_intrude->checkout_edge(edge);
    this->tetra_intrude->checkout_edge(edge->opposite);

    IVP_ASSERT(edge->tmp.gen.concav_flag != -2);
    IVP_Tri_Edge *opp = edge->opposite;
    if (opp<edge) edge = opp;
    if (edge->triangle->flags.is_hidden) return;
    this->tetra_intrude->checkin_edge(edge);
    if (edge->tmp.gen.concav_flag == 0)	return;
    
    IVP_DOUBLE rating = this->rate_tri_edge(edge);
    add_edge_to_min_list(edge,P_HASH_CLASS_NORMAL,rating);
}

void IVP_Object_Polygon_Tetra::add_edge_to_min_list(IVP_Tri_Edge *edge,P_HASH_CLASS hash_class, IVP_DOUBLE rating){
    IVP_ASSERT(edge->tmp.gen.hash_class == P_HASH_CLASS_NONE);
    IVP_Tri_Edge *opp = edge->opposite;
    if (opp<edge) edge = opp;
    IVP_ASSERT(edge->tmp.gen.hash_class == P_HASH_CLASS_NONE);
    min_hash[hash_class]->add((void *)edge,rating);
    edge->tmp.gen.hash_class = hash_class;
}

void IVP_Object_Polygon_Tetra::remove_edge_from_min_list(IVP_Tri_Edge *edge){
    if (edge->tmp.gen.hash_class){
	this->min_hash[edge->tmp.gen.hash_class]->remove((void *)edge);	
	edge->tmp.gen.hash_class = P_HASH_CLASS_NONE;
    }
    edge = edge->opposite;
    if (edge->tmp.gen.hash_class){
	this->min_hash[edge->tmp.gen.hash_class]->remove((void *)edge);	
	edge->tmp.gen.hash_class = P_HASH_CLASS_NONE;
    }
}


void IVP_Object_Polygon_Tetra::hide_triangle(IVP_Triangle *tri)
{
    // prepos: links are perfect
    if (1 || !tri->flags.is_hidden){
	IVP_Tri_Edge *edge = &tri->three_edges[0];
	for (int i=2;i>=0;i--,edge = edge->next){
	    this->remove_edge_from_min_list(edge);
	    this->tetra_intrude->checkout_edge(edge);
	    this->tetra_intrude->checkout_edge(edge->opposite);
	}
    }
    tri->flags.is_hidden = 1;
}

IVP_BOOL IVP_Object_Polygon_Tetra::p_link_edge(IVP_Tri_Edge *edge, IVP_Tri_Edge *neighb)
{
    // prepos: - perfect linked obj
    //         - other_side of edge

    IVP_Tri_Edge *orig_edge_oppo = edge->opposite;
    IVP_Tri_Edge *edge_oth = edge->other_side();
    
    if(!neighb){
	// triangle is open at this side, only linked with
	// triangle->other_side
	edge->opposite = edge_oth;
	edge_oth->opposite = edge;
	edge->behind = edge;
	edge_oth->behind = edge_oth;
	edge->concavity = 12345.6f; // most important: convex
	edge_oth->concavity = edge->concavity;
	return IVP_FALSE;
    }
    
    IVP_Tri_Edge *neighb_opp = neighb->opposite;
    IVP_Tri_Edge *neighb_oth = neighb->other_side();
    IVP_ASSERT(neighb_opp != edge); // questionable; maybe just do a return?
    {
	IVP_ASSERT(neighb_opp != edge); // questionable; maybe just do a return?
	IVP_Tri_Edge *test_edge;
	for (test_edge = edge->next; test_edge != edge;test_edge = test_edge->next){
	    IVP_ASSERT(test_edge != neighb_opp);
	}
    }

    IVP_Tri_Edge *edge_opp = edge->opposite;

    neighb->opposite = edge; 
    neighb_opp->opposite = edge_opp;

    edge->opposite = neighb;
    edge_oth->behind = neighb;
    edge_opp->other_side()->behind = neighb_opp;
    edge_opp->opposite = neighb_opp;

    neighb->opposite = edge;
    neighb_oth->behind = edge;
    neighb_opp->other_side()->behind = edge_opp;
    neighb_opp->opposite = edge_opp;


    // set/recalc concavity infos
    IVP_BOOL retval = IVP_FALSE;
    int flag = this->check_concavity_and_manage(edge, P_CONVEXIFY_STATE_LINK);
    //if (flag <0) retval = IVP_TRUE;
    flag = this->check_concavity_and_manage(orig_edge_oppo, P_CONVEXIFY_STATE_LINK);
    //if (flag <0) retval = IVP_TRUE;
    return retval;
}

int IVP_Object_Polygon_Tetra::link_triangle_couple(
    IVP_Triangle *triangle,
    IVP_Tri_Edge *neighbor_0,
    IVP_Tri_Edge *neighbor_1,
    IVP_Tri_Edge *neighbor_2)
{
    // prepos: triangle internal links,
    // 	        triangle->other_side == another new triangle to be inserted
    //	 	  object has perfect links.
    
    // inserts triangle triangle and its other_side triangle
    // into object. perfect links with neighbours.

    // neighbor_0 will be opposited with triangle->edge etc.
    ivp_u_bool flag;
    flag = p_link_edge(&triangle->three_edges[0], neighbor_0);
    IVP_ASSERT(!flag);
    flag = p_link_edge(triangle->three_edges[0].next, neighbor_1);
    IVP_ASSERT(!flag);
    flag = p_link_edge(triangle->three_edges[0].prev, neighbor_2);
    IVP_ASSERT(!flag);
    this->triangles.insert(triangle);
    this->triangles.insert(triangle->other_side);
    this->make_double_triangle_permanent(triangle);
    return IVP_OK;
}

void p_link_triangle_self(IVP_Triangle *tri){
/* prepos: set other_side */
    int i;
    for (i=0;i<3;i++){
	tri->three_edges[i].triangle = tri;
	tri->three_edges[i].behind = &tri->three_edges[i];
	tri->three_edges[i].opposite = tri->three_edges[i].other_side();
    }
}

IVP_Tri_Edge *IVP_Object_Polygon_Tetra::get_an_edge_with_points(IVP_Poly_Point *p1, IVP_Poly_Point *p2)
{
    IVP_Poly_Point *p[2];
    int i=0;
    if(p1 > p2) i = 1;
    p[i] = p1;
    p[1-i] = p2;

    IVP_Tri_Edge *found_edge = (IVP_Tri_Edge *)this->points_to_edge_hash->find((char *)&p[0]);
    return found_edge;
}


void IVP_Object_Polygon_Tetra::add_edge_into_point_to_edge_hash(IVP_Tri_Edge *edge)
{
    IVP_Poly_Point *p[2], *hp;

    p[0] = edge->start_point;
    p[1] = edge->next->start_point;

    if(p[0]>p[1]){ // swap
	hp = p[0]; p[0] = p[1]; p[1] = hp;
    }
    if (this->points_to_edge_hash->find((char *)&p[0]) != 0) {
	return;			// already in hash
    }
    this->points_to_edge_hash->add((char *)&p[0], (void *)edge);
}


IVP_Triangle::IVP_Triangle()
{
    next = prev = other_side = NULL;
    pierced_triangle = NULL;
    flags.is_terminal = 0;
    flags.is_hidden = 0;
    ivp_surface = NULL;
    memset((char*)(&three_edges[0]), 0, 3 * sizeof(IVP_Tri_Edge));
}

IVP_Triangle::~IVP_Triangle()
{
}

IVP_Triangle *IVP_Object_Polygon_Tetra::generate_double_triangle(IVP_Poly_Point *p1, IVP_Poly_Point *p2, IVP_Poly_Point *p3)
{
    IVP_Triangle *tri_a = new IVP_Triangle();
    IVP_Triangle *i_tri_a = new IVP_Triangle();

    // provide new edges
    IVP_Tri_Edge *e0 = &tri_a->three_edges[0];
    IVP_Tri_Edge *e1 = &tri_a->three_edges[1];
    IVP_Tri_Edge *e2 = &tri_a->three_edges[2];

    IVP_Tri_Edge *i_e0 = &i_tri_a->three_edges[0];
    IVP_Tri_Edge *i_e1 = &i_tri_a->three_edges[1];
    IVP_Tri_Edge *i_e2 = &i_tri_a->three_edges[2];

    // provide some links in new elements
    e0->next = e1; e0->prev = e2; e0->start_point = p1; e0->triangle = tri_a;
    e1->next = e2; e1->prev = e0; e1->start_point = p2; e1->triangle = tri_a;
    e2->next = e0; e2->prev = e1; e2->start_point = p3; e2->triangle = tri_a;

    i_e0->next = i_e2; i_e0->prev = i_e1; i_e0->start_point = p2; i_e0->triangle = i_tri_a;
    i_e1->next = i_e0; i_e1->prev = i_e2; i_e1->start_point = p3; i_e1->triangle = i_tri_a;
    i_e2->next = i_e1; i_e2->prev = i_e0; i_e2->start_point = p1; i_e2->triangle = i_tri_a;

    tri_a->other_side = i_tri_a; i_tri_a->other_side = tri_a;
    
    p_link_triangle_self(tri_a); p_link_triangle_self(i_tri_a);        

    tri_a->calc_hesse();
    i_tri_a->calc_hesse();
    int i;
    IVP_Tri_Edge *e;

    for (i=0,e=e0;i<3;e=e->next,i++){
	e->tmp.gen.tetra_point = e->start_point->tmp.tetra_point;
    }    
    for (i=0,e=i_e0;i<3;e=e->next,i++){
	e->tmp.gen.tetra_point = e->start_point->tmp.tetra_point;
    }
    return tri_a;
}

void IVP_Object_Polygon_Tetra::make_double_triangle_permanent(IVP_Triangle *triangle)
{
    int i;
    IVP_Tri_Edge *e;
    for (i=0,e= &triangle->three_edges[0];i<3;e=e->next,i++){
	this->add_edge_into_point_to_edge_hash(e);
    }
    
}

IVP_DOUBLE p_calc_min_intrude_dist(IVP_Triangle *tri, IVP_Intrusion_Status *stat,
			       IVP_DOUBLE start_min)
{
    IVP_Intrusion_Included_Points *i_point;
    IVP_DOUBLE min = start_min;
    for(i_point=stat->intruded_points; i_point; i_point=i_point->next){
	IVP_DOUBLE dist = tri->tmp.gen.hesse.get_dist(i_point->tetra_point->opoint);
	if(dist<min) min=dist;
    }
    IVP_Intrusion_Intersection *intersect;
    for(intersect=stat->intersections; intersect; intersect=intersect->next){
	if (intersect->type != IVP_INTRUSION_CHECK_OVERLAP){
	    IVP_DOUBLE dist = tri->tmp.gen.hesse.get_dist(&intersect->intersect_point);
	    if(dist<min) min=dist;
	}
    }
    return min;
}

void IVP_Object_Polygon_Tetra::calc_extrusion_point(const IVP_Tri_Edge *edge, IVP_U_Point *point_out){
    // assert IVP_Inline_Math::fabsd(concavity) > 8 * P_Pop_Eps

    IVP_U_Point mid_of_line;
    IVP_DOUBLE eps = P_Pop_Eps * 1.1f;
    IVP_Tri_Edge *oppo = edge->opposite; // mid of line
    mid_of_line.set_multiple(edge->start_point,0.5f);
    mid_of_line.add_multiple(oppo->start_point,0.5f);

    IVP_U_Point mpo;		// mid point of tri_a, P_Pop_Eps dist to tria
    mpo.add_multiple(&mid_of_line, &edge->triangle->tmp.gen.hesse, eps);

    IVP_U_Plain *oppo_ebene;
    {
	IVP_U_Point p0,p1;
	p0.add_multiple(oppo->start_point, &oppo->triangle->tmp.gen.hesse,eps);
	p1.add_multiple(oppo->next->start_point, &oppo->triangle->tmp.gen.hesse,eps);
	oppo_ebene = new IVP_U_Plain(&p0,&p1,oppo->next->next->start_point);
    }
    IVP_U_Straight straight;
    IVP_U_Point svec;
    svec.subtract(&mpo, edge->prev->start_point);
	
    straight.set(edge->prev->start_point, &svec);
    ((IVP_U_Hesse *)oppo_ebene)->calc_intersect_with(&straight,point_out);
    delete oppo_ebene;
}




void IVP_Object_Polygon_Tetra::pop_problematic_edge(IVP_Tri_Edge *edge){
    IVP_Tri_Edge *oppo = edge->opposite;
    int is_epsilon_problem = 0;
    int i_got_a_real_problem =0;
    IVP_DOUBLE eps = P_Pop_Eps * 1.1f;
    
    /* **** First check dist of opposing points *********/
    {
	IVP_DOUBLE dist0 = edge->triangle->tmp.gen.hesse.get_dist(oppo->prev->start_point);
	IVP_DOUBLE dist1 = oppo->triangle->tmp.gen.hesse.get_dist(edge->prev->start_point);
	if (edge->tmp.gen.concav_flag == 0) { // convex
	    dist0 = -dist0;
	    dist1 = -dist1;
	}
	
	if (dist1< dist0) dist0 = dist1;
	if (dist0 < P_Pop_Eps * 6.0f){
	    is_epsilon_problem = 1;
	}
#ifdef CONVEX_DEBUG
	printf("***** Problematic %s,pop: edge %i %i, other_p %i %i, dist %f\n",
	       (edge->tmp.gen.concav_flag == 0) ? "convex": "CONCAV",
	       edge->start_point->point_num(),
	       oppo->start_point->point_num(),
	       edge->prev->start_point->point_num(),
	       oppo->prev->start_point->point_num(),
	       dist0);
#endif	

    }
    
    IVP_U_Point mid_of_line;
    mid_of_line.set_multiple(edge->start_point,0.5f);
    mid_of_line.add_multiple(oppo->start_point,0.5f);

    IVP_U_Point new_point;
    
    if (is_epsilon_problem){
	if (edge->tmp.gen.concav_flag == 0) { // convex
	    new_point.add_multiple(&mid_of_line,&edge->triangle->tmp.gen.hesse,eps);
	}else{
	    i_got_a_real_problem = 1;
	}
    }else{
	this->calc_extrusion_point(edge,&new_point);
    }

    if (!i_got_a_real_problem && edge->tmp.gen.concav_flag != 0){ // check flatness of concav edges
	// provide edge variables of already existing triangles
	IVP_Tri_Edge *a0 = edge;     		// pop triangle a
	IVP_Tri_Edge *a1 = a0->next;
	IVP_Tri_Edge *a2 = a1->next;
   
	IVP_Tri_Edge *b0 = a0->opposite; 	// pop triangle b
	IVP_Tri_Edge *b1 = b0->next;
	IVP_Tri_Edge *b2 = b1->next;

	// provide four new triangles
	IVP_Triangle *tri_a = generate_double_triangle(b2->start_point, a2->start_point, a0->start_point);
	IVP_Triangle *tri_b = generate_double_triangle(a2->start_point, b2->start_point, a1->start_point);

    // check wether pop becomes too flat 
	int pop_too_flat_flag = 0;
	pop_too_flat_flag = p_check_for_flat( &tri_a->three_edges[0], &tri_b->three_edges[0], P_Pop_Too_Flat_Eps);
	if(pop_too_flat_flag == 1){
#ifdef CONVEX_DEBUG	
	    printf("problematic pop would be too flat.\n");
#endif
	    p_del_double_triangles(&tri_a, &tri_b);
	    i_got_a_real_problem = 1;
	}
    }
    

    if (!i_got_a_real_problem){

	remove_edge_from_min_list(edge); // in case of convex edges
	// insert new point in extra linked list
	IVP_Extra_Point *extra_point = new IVP_Extra_Point;
	IVP_Tetra_Point *extra_tetra_point = new IVP_Tetra_Point;
	P_MEM_CLEAR(extra_point);
	P_MEM_CLEAR(extra_tetra_point);

	extra_point->tmp.tetra_point = extra_tetra_point;
	extra_tetra_point->opoint = extra_point;
	extra_point->set(&new_point); // koords
	
	extra_point->l_tetras = this;
	extra_tetra_point->init(tetra_intrude);
	
	IVP_Tri_Edge *a0 = edge;     		// pop triangle a
	IVP_Tri_Edge *a1 = a0->next;
	IVP_Tri_Edge *a2 = a1->next;    
	// perform pyramid pop
	IVP_Triangle *tri_pa = generate_double_triangle(a0->next->start_point, a0->start_point, extra_point);
	IVP_Triangle *tri_pb = generate_double_triangle(a1->next->start_point, a1->start_point, extra_point);
	IVP_Triangle *tri_pc = generate_double_triangle(a2->next->start_point, a2->start_point, extra_point);

	// check wether an epsilon problem occurred
	IVP_INTRUSION_CHECK_RESULTS i_flag = this->tetra_intrude->check_intrusion(a0,
										 tri_pa->three_edges[0].prev,
										 tri_pb->three_edges[0].next,
										 tri_pc->three_edges[0].next, extra_points, 3);
	if(i_flag){
#ifdef CONVEX_DEBUG
	    printf("********* Epsilon problem with extra point. P_Pop_Eps: %f\n", P_Pop_Eps);
#endif
	    P_DELETE(extra_point);
	    P_DELETE(extra_tetra_point);
	    p_del_double_triangles(&tri_pa, &tri_pb, &tri_pc);
	    i_got_a_real_problem = 1;
	}else{
	    extra_point->next = this->extra_points;
	    this->extra_points = extra_point;
	    this->n_extra_points++;
	    
	    this->link_triangle_couple(tri_pa, a0, NULL, NULL); 
	    this->link_triangle_couple(tri_pb, a1, tri_pa->three_edges[0].prev, NULL);
	    this->link_triangle_couple(tri_pc, a2, tri_pb->three_edges[0].prev, tri_pa->three_edges[0].next);

	    this->hide_triangle(tri_pa);
	    this->hide_triangle(tri_pb);
	    this->hide_triangle(tri_pc);
	    this->hide_triangle(a0->triangle);
	}
    }
    
    if (i_got_a_real_problem){
#ifdef CONVEX_DEBUG
	printf("********* Epsilon problem moved to epsilon hash: epsilon = %f\n", P_Pop_Eps);
#endif
	this->move_edge_to_epsilon_hash(edge);
    }
}




void IVP_Object_Polygon_Tetra::record_intruding_convex_edges(IVP_Intrusion_Status *status)
{
    IVP_Intrusion_Intersection *is = status->intersections;
    for( ; is; is = is->next){
	IVP_INTRUSION_CHECK_RESULTS typ = is->type;
	if(typ != IVP_INTRUSION_CHECK_LINE) continue;
	IVP_Poly_Point *p1 = is->line_endpoints[0];
	IVP_Poly_Point *p2 = is->line_endpoints[1];
	IVP_Tri_Edge *e=this->get_an_edge_with_points(p1, p2);
	IVP_ASSERT(e);

	// search the matching edge with this points
	IVP_Tri_Edge *e2 = e;
	do {
	    if((e2->tmp.gen.concav_flag==0) && !e2->triangle->flags.is_hidden){
		this->move_edge_to_convex_intrude_hash(e2);
		break; // searching for convex unhidden edges only
	    }
	    e2 = e2->behind;
	}while( e2 != e);
    }
}

void IVP_Object_Polygon_Tetra::link_existing_pop_edge(IVP_Tri_Edge *pop_edge)
{
	IVP_Tri_Edge *e = pop_edge->opposite->next;
	IVP_Tri_Edge *end_edge = pop_edge;
	//IVP_Poly_Point *p_sp = pop_edge->start_point;
	IVP_Poly_Point *p_ep = pop_edge->next->start_point;
	while(1){
	    if(e==end_edge) break; // all neighbours visited
	    //IVP_Poly_Point *n_sp = e->start_point;
	    IVP_Poly_Point *n_ep = e->next->start_point;
	    //IVP_ASSERT (p_sp==n_sp);
	    if(p_ep==n_ep){
		// encountered a pop edge -> link it
		this->p_link_edge(e, pop_edge->opposite);
#ifdef CONVEX_DEBUG	
		printf("\n**********************************************pop edge already existed in neighbour region -> linked it.\n");
		e->print("e");
		pop_edge->print("e0");
		printf("\n");
#endif
		break;
	    }
	    e = e->opposite->next;
	}
}

void IVP_Object_Polygon_Tetra::pop_concav_edge(IVP_Tri_Edge *edge)
{
    // assert edge is real concave 
#ifdef CONVEX_DEBUG
    printf("Try pop: edge %d %d with %d & %d. concavity %g)",
	   edge->start_point->point_num(),
	   edge->next->start_point->point_num(),
	   edge->prev->start_point->point_num(),
	   edge->opposite->prev->start_point->point_num(),
	   edge->concavity );
#endif

    // provide edge variables of already existing triangles
    IVP_Tri_Edge *a0 = edge;     		// pop triangle a
    IVP_Tri_Edge *a1 = a0->next;
    IVP_Tri_Edge *a2 = a1->next;
   
    IVP_Tri_Edge *b0 = a0->opposite; 	// pop triangle b
    IVP_Tri_Edge *b1 = b0->next;
    IVP_Tri_Edge *b2 = b1->next;
       
    // provide four new triangles
    IVP_Triangle *tri_a = generate_double_triangle(b2->start_point, a2->start_point, a0->start_point);
    IVP_Triangle *tri_b = generate_double_triangle(a2->start_point, b2->start_point, a1->start_point);

    IVP_Tri_Edge *e0 = &tri_a->three_edges[0];
    IVP_Tri_Edge *f0 = &tri_b->three_edges[0];
    
    /* Check wether pop is allowed */
    // calculate a required pop flag (for a single neighbour triangle)
    ivp_u_bool tetra_close_flag = IVP_FALSE;
    IVP_Triangle *tri_first = tri_a;
    IVP_Triangle *tri_second = tri_b;
    int n_new_triangles = 2;
    if(a0->next->opposite->triangle == b0->prev->opposite->triangle){
	n_new_triangles--;
	tetra_close_flag = IVP_TRUE;
    }
    if(a0->prev->opposite->triangle == b0->next->opposite->triangle){
	tetra_close_flag = IVP_TRUE;
	n_new_triangles--;
	// swap: calc_intrusion_status wants existing tris first
	tri_first = tri_b;
	tri_second = tri_a;
    }

    this->remove_edge_from_min_list(edge);

    // check wether pop becomes too flat 
    int pop_too_flat_flag = 0;
    if(!tetra_close_flag){
	pop_too_flat_flag = p_check_for_flat(e0, f0, P_Pop_Too_Flat_Eps);
	if(pop_too_flat_flag == 1){
	    this->move_edge_to_epsilon_hash(edge);
#ifdef CONVEX_DEBUG	
	    printf("\n            too flat -> edge moved to epsilon problem hash!\n");
#endif
	    p_del_double_triangles(&tri_a, &tri_b);
	    return;
	}
    }
    
    IVP_INTRUSION_CHECK_RESULTS intrusion_val;
    {
	IVP_Tri_Edge *otherside0 = tri_first->three_edges[0].other_side();
	IVP_Tri_Edge *otherside1 = tri_second->three_edges[0].other_side();
	intrusion_val = this->tetra_intrude->check_intrusion(a0, b0,otherside0, otherside1,
				 this->extra_points, n_new_triangles );
    }
    
    if((intrusion_val!=IVP_INTRUSION_CHECK_NONE)){
	// alas, must abort pop
	if(intrusion_val!=IVP_INTRUSION_CHECK_EPSILON){
	    this->move_edge_to_problem_hash(edge);
	    IVP_Intrusion_Status *status =
		this->tetra_intrude->calc_intrusion_status(a0, b0,
							   tri_first->three_edges[0].other_side(),
							   tri_second->three_edges[0].other_side(),
							   this->extra_points, n_new_triangles );
	    this->record_intruding_convex_edges(status);
#ifdef CONVEX_DEBUG	
	    printf("\n            intrusion: edge added to problem_min_hash.\n");
	    status->print("test status");
#endif
	    delete status;
	}else{
#ifdef CONVEX_DEBUG	
	    printf("\n            epsilon violation. smaller epsilon will work. shifted to eps hash.\n");
#endif
	    this->move_edge_to_epsilon_hash(edge);
	}
	p_del_double_triangles(&tri_a, &tri_b);
	return;
    }

    /*** insert new triangles ***/
    IVP_Tri_Edge *p0 = b1->opposite;
    IVP_Tri_Edge *q0 = b2->opposite;
    IVP_Tri_Edge *n0 = a2->opposite;
    IVP_Tri_Edge *o0 = a1->opposite;
    
    IVP_Tri_Edge *link_to_tri_a_edge = NULL;
    IVP_Tri_Edge *link_to_tri_b_edge = NULL;
    
    if(p0->triangle == n0->triangle){
	link_to_tri_a_edge = n0->next->opposite;
	P_DELETE(tri_a);
    }else{
	make_double_triangle_permanent(tri_a);
    }
    if(q0->triangle == o0->triangle){
	link_to_tri_b_edge = o0->prev->opposite;
	P_DELETE(tri_b);
    }else{
	make_double_triangle_permanent(tri_b);
    }
    
    if (tri_a){
	if (tri_b){
	    this->link_triangle_couple(tri_a, NULL, n0, p0);
	    this->link_triangle_couple(tri_b, e0, q0, o0);
	}else{
	    this->link_triangle_couple(tri_a, link_to_tri_b_edge, n0, p0);
	}
    }else{
	if (tri_b){
	    this->link_triangle_couple(tri_b, link_to_tri_a_edge, q0, o0);
	}else{
	    printf("* NULL pop");
	    // anything else?
	}
    }
	    
    // set hiding flags. algorithm wants to know what he may access
    this->hide_triangle(a0->triangle); // popped triangles are hidden now
    this->hide_triangle(b0->triangle);

    this->hide_triangle(a1->opposite->triangle);
    this->hide_triangle(b1->opposite->triangle);

    // if pop edge already exists in neighborhood -> link it
    if(!tetra_close_flag){
	this->link_existing_pop_edge(f0);
	//this->link_existing_pop_edge(e0);
    } 

#ifdef CONVEX_DEBUG	
    printf(" *\n");
#endif

}
    
void IVP_Object_Polygon_Tetra::convexify()
{
    // ATT! not idempotent
    
#ifdef CONVEX_DEBUG
    printf("\n\nCONVEXIFY\n==============\n");
#endif

    
    // init global epsilons
    P_Pop_Eps = P_POP_EPS_INIT;
    P_Pop_Scal_Eps = P_POP_SCAL_EPS_INIT;
    P_Pop_Too_Flat_Eps = P_POP_TOO_FLAT_EPS_FACTOR * P_Pop_Eps;
    
    // set random seed for random breakup
    // p_srand(123);

    // set tetra points in polygon
    int point_anz = this->n_points;
    int n_malloced_points = point_anz;
    IVP_Tetra_Point *t4_points = (IVP_Tetra_Point *)p_calloc(sizeof(IVP_Tetra_Point),n_malloced_points);
    int i;
    for(i=point_anz-1; i>=0; i--){
	t4_points[i].opoint = &this->points[i];
	this->points[i].tmp.tetra_point = &t4_points[i];
    }
   
    this->tetra_intrude = new IVP_Tetra_Intrude(t4_points, this->n_points);
    tetra_intrude->n_tetra_points_malloced = n_tetra_points_malloced;

    IVP_Triangle *tri;
    for (tri = triangles.first; tri; tri = tri->next){
        for(i=0; i<=2; i++){
	    IVP_Tri_Edge *e = &tri->three_edges[i];
	    e->tmp.gen.tetra_point = e->start_point->tmp.tetra_point;
	}
	tri->calc_hesse();
    }
    {
	for (i= P_HASH_CLASS_NORMAL; i< P_HASH_CLASS_MAX;i++){
	    this->min_hash[i] = new IVP_U_Min_Hash(512);
	}
    }

    this->calc_concavities();
#ifdef CONVEX_DEBUG
    int num_of_loops = 0;
#endif
    
    /*** pop concav edges till everything is pretty convex ***/
    while(1){
#ifdef CONVEX_DEBUG	
	this->check_konsistency_of_triangles();
	    
	printf("%i:",num_of_loops);
	for (i=P_HASH_CLASS_NORMAL; i<P_HASH_CLASS_MAX;i++){
	    printf(" %i",min_hash[i]->counter);
	}
	printf(":   ");
	num_of_loops++;
	if(num_of_loops == n_physical_pops -1){
	    printf("Nearly Last Loop!\n");
	}

	if(num_of_loops == n_physical_pops ){
	    printf("P_CONVEX_LOOPS reached!\n");
	    break;
	}
#endif
	
	// ******** take an existing normal concav edge and pop it
	IVP_Tri_Edge *edge = (IVP_Tri_Edge *)min_hash[P_HASH_CLASS_NORMAL]->find_min_elem();
	if (edge){
	    this->pop_concav_edge(edge);
	    continue;
	}
	
	// ******** take an existing epsilon problem edge
	if(min_hash[P_HASH_CLASS_EPSILON]->counter && (P_Pop_Eps >= P_DOUBLE_EPS)){
	    P_Pop_Eps *= P_POP_EPS_REDUCE_FACTOR;
	    P_Pop_Too_Flat_Eps = P_Pop_Eps * P_POP_TOO_FLAT_EPS_FACTOR;
	    if(P_Pop_Eps < P_DOUBLE_EPS){
		printf("*** P_Pop_Eps went smaller than P_DOUBLE_EPS!!\n");
		continue;
	    }
#ifdef CONVEX_DEBUG
	    printf("#############################    Reduced P_Pop_Eps to %f. Hashes are recalculated\n", P_Pop_Eps);
#endif
	    //calc_concavities();
	    // swap hashes
	    {
		IVP_ASSERT(min_hash[P_HASH_CLASS_NORMAL]->counter == 0);
		IVP_U_Min_Hash *h = min_hash[P_HASH_CLASS_NORMAL];
		min_hash[P_HASH_CLASS_NORMAL] = min_hash[P_HASH_CLASS_EPSILON];
		min_hash[P_HASH_CLASS_EPSILON] = h;
		IVP_Tri_Edge *e;
		IVP_U_Min_Hash_Enumerator elems(min_hash[P_HASH_CLASS_NORMAL]);
		while ( (e=(IVP_Tri_Edge *)elems.get_next_element()) != NULL){
		    e->tmp.gen.hash_class = P_HASH_CLASS_NORMAL;
		}
	    }
	    continue;
	}

	
        // ********** take an existing problematic edge
	{
	                edge  = (IVP_Tri_Edge *)min_hash[P_HASH_CLASS_PROBLEM]->find_min_elem();
	    IVP_Tri_Edge *edge2 = (IVP_Tri_Edge *)min_hash[P_HASH_CLASS_CONVEX_INTRUDE]->find_min_elem();
	    if (edge){
		if (edge2){
		    if (min_hash[P_HASH_CLASS_PROBLEM]->find_min_value()  > min_hash[P_HASH_CLASS_CONVEX_INTRUDE]->find_min_value()){
			edge = edge2;
		    }
		}
	    }else{
		edge = edge2;
	    }
	}
	if(edge){
	    // handle problematic situation by adding an additional point
	    this->pop_problematic_edge(edge);
	    continue;
	}
//	printf("No edges left in all three min hashes.\n");
	break;
    }
    
    this->final_convexify_check();
    for (i= P_HASH_CLASS_NORMAL; i< P_HASH_CLASS_MAX; i++){
	P_DELETE(min_hash[i]);
    }
    P_DELETE(this->tetra_intrude);    

    P_FREE(t4_points);
    
#ifdef CONVEX_DEBUG   
    printf("Convexify finished.\n");
#endif
}



char *p_mergesort(void **array,IVP_INT32 start, IVP_INT32 end, IVP_INT32 (*compare)(void *,void *,char *cd), char *client_data)
{
	IVP_INT32 size;
	IVP_INT32 mid;
	IVP_INT32 i, j;
	IVP_INT32	dest;
	void **buffer;
	void *ibuf[256];
	char *error;

	size = end - start;
	if (size <= 1) {
		return 0;
	}
	mid = (start+end) / 2;
	error = p_mergesort(array,start,mid,compare,client_data);
	error = p_mergesort(array,mid,end,compare,client_data);
	if (size <256) {
		buffer = ibuf;
	}else{
		buffer = (void **)p_malloc((size_t)(size * sizeof(void *)));
	}

	for ( dest = 0, i = start, j = mid ;
		i< mid && j < end;){
		if (compare(array[i],array[j],client_data) < 0) {
			buffer[dest++] = array[i++];
		}else{
			buffer[dest++] = array[j++];
		}
	}
	while(i<mid)	buffer[dest++] = array[i++];
	while(j<end)	buffer[dest++] = array[j++];
	memcpy( (char *)(array+start),(char *)buffer,(int)size * sizeof(void *));
	if (size>=256) P_FREE(buffer);
	return error;
}





    



///////////////////////////////////////////
///////////////////////////////////////////
#define IVP_MAX_SCAL_VAL -1E-6f

void IVP_Object_Polygon_Tetra::insert_pierce_info()
{
    // Each triangle gets info about another triangle which
    // is a valid start for traversing to an opposite triangle
    // with smaller termination_len in case of backside termination.
    // (piercing=durchstossen)

    // clear pierce info (to be sure)
    {
	IVP_Triangle *tri;
	for(tri=triangles.first; tri; tri=tri->next){
	    tri->pierced_triangle = 0;
	}
    }

    // insert all pierce infos
    {
	IVP_Triangle *tri;
	for(tri=triangles.first; tri; tri=tri->next){
	    if(tri->flags.is_hidden) continue; // works at convex surface only
	    if(tri->pierced_triangle) continue; // already treated by corresp. triangle
	    
	    // search matching triangle
	    IVP_Triangle *tri2;
	    IVP_Triangle *found_tri = 0;
	    IVP_DOUBLE min_scal_val = IVP_MAX_SCAL_VAL;
	    for(tri2=triangles.first; tri2; tri2=tri2->next){
		if(tri2->flags.is_hidden) continue;
		// minimize scalarproduct
		// @@@ we could stop earlier to avoid full quadratic runtime
		IVP_DOUBLE scal_val = tri->tmp.gen.hesse.dot_product(&tri2->tmp.gen.hesse);
		if(scal_val < min_scal_val){
		    // @@@ we could add some more intelligence like dist from tri...
		    min_scal_val = scal_val;
		    found_tri = tri2;
		}
	    }

	    // insert pierce info
	    IVP_ASSERT(found_tri);
	    tri->pierced_triangle = found_tri;
	    found_tri->pierced_triangle = tri; // problem is pretty good symmetric (depending on bewertungsfunktion)
	}
    }
}



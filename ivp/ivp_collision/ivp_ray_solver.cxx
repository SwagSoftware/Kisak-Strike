
// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#include <ivp_physics.hxx>

#include <ivu_min_hash.hxx>

#include <ivp_compact_ledge.hxx>
#include <ivp_compact_ledge_solver.hxx>

#include <ivp_clustering_longrange.hxx>
#include <ivp_compact_surface.hxx>

#ifndef WIN32
#	pragma implementation "ivp_ray_solver.hxx"
#endif

#include <ivp_ray_solver.hxx>

    // ATT: must be the same than in class definition!

#define IVP_LEFT 1
#define IVP_RIGHT 2
#define IVP_ABOVE 4
#define IVP_BELOW 8
#define IVP_FRONT 16
#define IVP_BEHIND 32

IVP_Ray_Solver::IVP_Ray_Solver(const IVP_Ray_Solver_Template *templ) 
{
    // ray dir must be normized.
    IVP_ASSERT( (templ->ray_normized_direction.real_length() > (1.0f - 1e-4f) ) &&
	      (templ->ray_normized_direction.real_length() < (1.0f + 1e-4f) ) );
    
    this->ray_direction.set(&templ->ray_normized_direction);
    this->ray_start_point.set(&templ->ray_start_point);
    
    this->ray_length = templ->ray_length;
    this->ray_flags  = templ->ray_flags;
    
    // calc end point of ray
    this->ray_center_point.set(&this->ray_start_point);
    this->ray_center_point.add_multiple( &this->ray_direction, this->ray_length * 0.5f);
    
    this->ray_end_point.add_multiple(&ray_start_point, &ray_direction, ray_length);
}


void IVP_Ray_Solver_Min_Hash::add_hit_object(IVP_Real_Object *object, const IVP_Compact_Ledge *compact_ledge, const IVP_Compact_Triangle *compact_triangle, IVP_DOUBLE hit_dist,
				    IVP_U_Point *hit_sur_vec_os)
{
    // Insert object into class member 'output_min_hash'
    // when ray flags allow for it.
    
    if(this->output_min_hash.counter >= IVP_MAX_NUM_RAY_HITS){
	IVP_IF(1){
	    printf("IVP_Ray_Solver::add_hit_object - max ray hits exceeded -> hit list truncated.\n");
	}
	// todo: if hit_dist < current hash minimum: replace any min hash entry with this hit
	// ...
	return;
    }

    IVP_Ray_Hit *ray_hit = &this->hit_info[this->output_min_hash.counter];
    ray_hit->hit_real_object = object;
    ray_hit->hit_compact_ledge = compact_ledge;
    ray_hit->hit_compact_triangle = compact_triangle;
    IVP_IF(0){
	printf("object inserted: %s, dist %g\n", object->get_name(), hit_dist);
    }
    ray_hit->hit_surface_direction_os.set(hit_sur_vec_os);
    ray_hit->hit_distance = hit_dist;

    this->output_min_hash.add((void *)ray_hit, hit_dist);
}


IVP_BOOL IVP_Ray_Solver::check_ray_against_sphere(const IVP_U_Float_Point *sphere_center_ws,	  IVP_FLOAT sphere_radius){
    // returns IVP_FALSE if sphere is not hit by ray
    IVP_U_Float_Point delta_vec;
    IVP_DOUBLE rad_sum = ray_length * 0.5f + sphere_radius;
    delta_vec.subtract(sphere_center_ws, &ray_center_point);

    if (delta_vec.quad_length() >= rad_sum * rad_sum){
	return IVP_FALSE;
    }
    
    IVP_U_Float_Point h;
    h.inline_calc_cross_product(&ray_direction, &delta_vec);
    
    IVP_FLOAT quad_dist = (IVP_FLOAT)h.quad_length();
    
    IVP_FLOAT coll_dist = sphere_radius;
    
    if(quad_dist < coll_dist*coll_dist) return IVP_TRUE;
    
    return IVP_FALSE;
}

IVP_BOOL IVP_Ray_Solver_Group::check_ray_group_against_sphere(const IVP_U_Float_Point *sphere_center_ws,	  IVP_FLOAT sphere_radius){
    // returns IVP_FALSE if sphere is not hit by ray
    IVP_U_Float_Point delta_vec;
    IVP_DOUBLE rad_sum = radius + sphere_radius;
    delta_vec.subtract(sphere_center_ws, &center_ws);

    if (delta_vec.quad_length() >= rad_sum * rad_sum){
	return IVP_FALSE;
    }
    return IVP_TRUE;
}

IVP_BOOL IVP_Ray_Solver_Os::check_ray_against_sphere_os(const IVP_U_Float_Point *sphere_center_os,  IVP_FLOAT sphere_radius){
    // returns IVP_FALSE if sphere is not hit by ray
    IVP_U_Float_Point delta_vec; // quick check for ends of ray
    IVP_DOUBLE qrad_sum = ray_length * 0.5f + sphere_radius;
    IVP_DOUBLE qsphere_rad = sphere_radius * sphere_radius;
    qrad_sum *= qrad_sum;
    delta_vec.subtract(sphere_center_os, &ray_center_point);
    IVP_DOUBLE quad_center_dist = delta_vec.quad_length();
    if ( quad_center_dist >= qrad_sum ){
	return IVP_FALSE;
    }
    if ( quad_center_dist < qsphere_rad ){
	return IVP_TRUE;
    }
    
    IVP_U_Float_Point h;
    h.inline_calc_cross_product(&ray_direction, &delta_vec);
    
    IVP_FLOAT quad_dist = (IVP_FLOAT)h.quad_length();
    
    IVP_FLOAT coll_dist = sphere_radius;
    
    if((coll_dist*coll_dist - quad_dist) >= -0.01f){ //@@CB
	return IVP_TRUE;
    }
    
    return IVP_FALSE;
}


IVP_BOOL IVP_Ray_Solver::check_ray_against_square(IVP_FLOAT pos_dist,
						  IVP_FLOAT pos_axis_len,
						  const IVP_U_Float_Point *min_coords,
						  const IVP_U_Float_Point *max_coords,
						  int coord_0, int coord_1)
{

    // returns IVP_FALSE if square is not hit by ray
    
    // better than 'if' in the loop!? -OG
    short coord_index[2];
    coord_index[0] = coord_0;
    coord_index[1] = coord_1;
    
    // each axis
    int i;
    for(i=1; i>=0; --i){
	// check against one limiting plane
	int coord=coord_index[i];
	IVP_FLOAT comp_val = pos_dist * ray_length * ray_direction.k[coord];
	if( (comp_val < (min_coords->k[coord] - ray_start_point.k[coord]) * pos_axis_len ) ||
	    (comp_val > (max_coords->k[coord] - ray_start_point.k[coord]) * pos_axis_len ) ){
	    // miss
	    return IVP_FALSE;
	}
    }
    
    // ray hits for both coords
    return IVP_TRUE;
}

IVP_BOOL IVP_Ray_Solver::check_ray_against_cube(const IVP_U_Float_Point *luf_point,
						const IVP_U_Float_Point *rlb_point)
{
    // Checks wether the specified cube is hit by the specified ray.
    // Returns IVP_TRUE in case of a hit.
    // Returns IVP_FALSE if cube is not hit by the ray.
    
    // calc clip flags for start point.
    
    // todo: use ray radius, but beware of the corner roundings!
    
    short start_point_flags = 0;
    short end_point_flags   = 0;
    
    if(ray_start_point.k[0]<luf_point->k[0]) start_point_flags = IVP_LEFT;
    else if(ray_start_point.k[0]>rlb_point->k[0]) start_point_flags = IVP_RIGHT;
    
    if(ray_start_point.k[1]<luf_point->k[1]) start_point_flags |= IVP_ABOVE;
    else if(ray_start_point.k[1]>rlb_point->k[1]) start_point_flags |= IVP_BELOW;
    
    if(ray_start_point.k[2]<luf_point->k[2]) start_point_flags |= IVP_FRONT;
    else if(ray_start_point.k[2]>rlb_point->k[2]) start_point_flags |= IVP_BEHIND;
    
    // if startpoint is inside the cube, ray hits.
    if(start_point_flags==0) return IVP_TRUE;
    
    // calc clip flags for end point.
    
    if(ray_end_point.k[0]<luf_point->k[0]) end_point_flags = IVP_LEFT;
    else if(ray_end_point.k[0]>rlb_point->k[0]) end_point_flags = IVP_RIGHT;
    
    if(ray_end_point.k[1]<luf_point->k[1]) end_point_flags |= IVP_ABOVE;
    else if(ray_end_point.k[1]>rlb_point->k[1]) end_point_flags |= IVP_BELOW;
    
    if(ray_end_point.k[2]<luf_point->k[2]) end_point_flags |= IVP_FRONT;
    else if(ray_end_point.k[2]>rlb_point->k[2]) end_point_flags |= IVP_BEHIND;
    
    // if endpoint is inside the cube, ray hits.
    if(end_point_flags==0) return IVP_TRUE;
    
    // if points are both outside for any coordinate, ray doesn't hit.
    if(start_point_flags & end_point_flags){
	return IVP_FALSE;
    }
    
    // now we need a more thorough check...
    
    // facet 0 (front)
    if((start_point_flags&IVP_FRONT) && !(end_point_flags&IVP_FRONT)){
	IVP_FLOAT pos_dist = luf_point->k[2] - ray_start_point.k[2];
	IVP_FLOAT pos_axis_len = ray_end_point.k[2] - ray_start_point.k[2];
	IVP_ASSERT(pos_dist >= 0.0f);
	IVP_ASSERT(pos_axis_len >= 0.0f);
	IVP_U_Float_Point max_coords(rlb_point->k[0], rlb_point->k[1], luf_point->k[2]);
	IVP_BOOL res = check_ray_against_square(pos_dist, pos_axis_len, luf_point, &max_coords, 0, 1);
	if(res) return IVP_TRUE;
    }
    // facet 1 (right)
    if((start_point_flags&IVP_RIGHT) && !(end_point_flags&IVP_RIGHT)){
	IVP_FLOAT pos_dist = ray_start_point.k[0] - rlb_point->k[0];
	IVP_FLOAT pos_axis_len = ray_start_point.k[0] - ray_end_point.k[0];
	IVP_ASSERT(pos_dist >= 0.0f);
	IVP_ASSERT(pos_axis_len >= 0.0f);
	IVP_U_Float_Point min_coords(rlb_point->k[0], luf_point->k[1], luf_point->k[2]);
	IVP_BOOL res = check_ray_against_square(pos_dist, pos_axis_len, &min_coords, rlb_point, 1, 2);
	if(res) return IVP_TRUE;
    }
    // facet 2 (back)
    if((start_point_flags&IVP_BEHIND) && !(end_point_flags&IVP_BEHIND)){
	IVP_FLOAT pos_dist = ray_start_point.k[2] - rlb_point->k[2];
	IVP_FLOAT pos_axis_len =  ray_start_point.k[2] - ray_end_point.k[2];
	IVP_ASSERT(pos_dist >= 0.0f);
	IVP_ASSERT(pos_axis_len >= 0.0f);
	IVP_U_Float_Point min_coords(luf_point->k[0], luf_point->k[1], rlb_point->k[2]);
	IVP_BOOL res = check_ray_against_square(pos_dist, pos_axis_len, &min_coords, rlb_point, 0, 1);
	if(res) return IVP_TRUE;
    }
    // facet 3 (left)
    if((start_point_flags&IVP_LEFT) && !(end_point_flags&IVP_LEFT)){
	IVP_FLOAT pos_dist =  luf_point->k[0] - ray_start_point.k[0];
	IVP_FLOAT pos_axis_len = ray_end_point.k[0] - ray_start_point.k[0];
	IVP_ASSERT(pos_dist >= 0.0f);
	IVP_ASSERT(pos_axis_len >= 0.0f);
	IVP_U_Float_Point max_coords(luf_point->k[0], rlb_point->k[1], rlb_point->k[2]);
	IVP_BOOL res = check_ray_against_square(pos_dist, pos_axis_len, luf_point, &max_coords, 1, 2);
	if(res) return IVP_TRUE;
    }
    // facet 4 (top)
    if((start_point_flags&IVP_ABOVE) && !(end_point_flags&IVP_ABOVE)){
	IVP_FLOAT pos_dist = luf_point->k[1] - ray_start_point.k[1];
	IVP_FLOAT pos_axis_len =  ray_end_point.k[1] - ray_start_point.k[1];
	IVP_ASSERT(pos_dist >= 0.0f);
	IVP_ASSERT(pos_axis_len >= 0.0f);
	IVP_U_Float_Point max_coords(rlb_point->k[0], luf_point->k[1], rlb_point->k[2]);
	IVP_BOOL res = check_ray_against_square(pos_dist, pos_axis_len, luf_point, &max_coords, 0, 2);
	if(res) return IVP_TRUE;
    }
    // facet 5 (down)
    if((start_point_flags&IVP_BELOW) && !(end_point_flags&IVP_BELOW)){
	IVP_FLOAT pos_dist = ray_start_point.k[1] - rlb_point->k[1];
	IVP_FLOAT pos_axis_len =  ray_start_point.k[1] - ray_end_point.k[1];
	IVP_ASSERT(pos_dist >= 0.0f);
	IVP_ASSERT(pos_axis_len >= 0.0f);
	IVP_U_Float_Point min_coords(luf_point->k[0], rlb_point->k[1], luf_point->k[2]);
	IVP_BOOL res = check_ray_against_square(pos_dist, pos_axis_len, &min_coords, rlb_point, 0, 2);
	if(res) return IVP_TRUE;
    }
    
    return IVP_FALSE;
}

IVP_BOOL IVP_Ray_Solver_Group::check_ray_group_against_cube(const IVP_U_Float_Point *cube_center_ws, IVP_FLOAT cube_size){
    IVP_U_Float_Point dist;
    dist.subtract( cube_center_ws, &center_ws);
    dist.set( IVP_Inline_Math::fabsd(dist.k[0]),
	IVP_Inline_Math::fabsd(dist.k[1]),
	IVP_Inline_Math::fabsd(dist.k[2]));
    cube_size = 0.5f * cube_size + this->radius;
    if ( dist.k[0] > cube_size ) return IVP_FALSE;
    if ( dist.k[1] > cube_size ) return IVP_FALSE;
    if ( dist.k[2] > cube_size ) return IVP_FALSE;
    if ( dist.quad_length() >= cube_size * cube_size * 3 ) return IVP_FALSE;
    
    return IVP_TRUE;
}

IVP_BOOL IVP_Ray_Solver_Os::check_ray_against_compact_ledge_os(const IVP_Compact_Ledge *ledge_to_compare){
    // Checks wether the specified ledge is hit by the specified ray.
    // Adds the corresponding object to the min hash in case of a hit.
    // Get hold of a surface that is pointing into the opposite of the ray direction.
    int n_triangles = ledge_to_compare->get_n_triangles();

    if (n_triangles == 2){ // super fast version for triangle ledges
	const IVP_Compact_Triangle *tri = ledge_to_compare->get_first_triangle();
	const IVP_Compact_Edge *edge = tri->get_first_edge();
	
	IVP_U_Point hesse_vec_os; // prefer normized ?
	IVP_CLS.calc_hesse_vec_object_not_normized(edge, ledge_to_compare, &hesse_vec_os);

	const IVP_U_Float_Point *p0 = IVP_CLS.give_object_coords(edge, ledge_to_compare);

	IVP_DOUBLE z = hesse_vec_os.dot_product(p0);
	IVP_DOUBLE a = ray_start_point.dot_product( &hesse_vec_os ) - z;
	IVP_DOUBLE b = ray_end_point.dot_product( &hesse_vec_os ) - z;

	if (a*b >=0){
	    return IVP_FALSE; // does not hit
	}

	IVP_DOUBLE hit_dist = a/(a-b);
	IVP_U_Point intersect_point; intersect_point.set_interpolate( &ray_start_point, &ray_end_point, hit_dist);

	IVP_Unscaled_QR_Result qr;
	IVP_CLS.calc_unscaled_qr_vals_F_space(ledge_to_compare, edge, &intersect_point,&qr);

	if (qr.is_outside()){
	    return IVP_FALSE;
	}
	
	if (a<0) hesse_vec_os.mult(-1);
	hesse_vec_os.IVP_U_Point::normize();
	hit_listener->add_hit_object(object, ledge_to_compare, tri, hit_dist * ray_length, &hesse_vec_os);
	return IVP_TRUE;
    }
    
    IVP_U_Float_Point ray_start_os;
    IVP_U_Float_Point ray_dir_os;
    
    ray_start_os.set(&ray_start_point);
    ray_dir_os.set(&ray_direction);
    
    const IVP_Compact_Triangle *tri = ledge_to_compare->get_first_triangle();
    const IVP_Compact_Edge *take_edge = 0;
    int i;
    for(i=0; i<n_triangles; i++, tri=tri->get_next_tri()){
	// check triangle
	const IVP_Compact_Edge *edge = tri->get_first_edge();
	IVP_U_Point hesse_vec_os; // prefer normized ?
	IVP_CLS.calc_hesse_vec_object_not_normized(edge, ledge_to_compare, &hesse_vec_os);
	
	// check direction
	IVP_DOUBLE dot_prod = hesse_vec_os.dot_product(&ray_dir_os);	

	if(dot_prod > -P_DOUBLE_RES){ // @@@ select correct side
	    continue;
	}

	// now check first point whether it sees the area
        const IVP_U_Float_Point *p0 = IVP_CLS.give_object_coords(edge, ledge_to_compare);
	IVP_DOUBLE z = hesse_vec_os.dot_product(p0);
	IVP_DOUBLE a = ray_start_point.dot_product( &hesse_vec_os ) - z;
	if ( a > 0 ){
	    // quick check second object 
	    IVP_DOUBLE b = ray_end_point.dot_product( &hesse_vec_os ) - z;
	    if (b > 0) return IVP_FALSE;	// no way to get a hit
	    take_edge = edge;
	    break;
	}

    } // for triangles
    
    if(!take_edge){
	IVP_IF(0){
	    printf("IVP_Ray_Solver::check_ray_against_compact_ledge - no correct face found.\n");
	}
	return IVP_FALSE;
    }
    
    {
	// navigate to potential hit position.
	const IVP_Compact_Edge *edge = take_edge;
	int tri_cnt = ledge_to_compare->get_n_triangles();
	for(; --tri_cnt>=0; ){
	    // if plane looks in opposite direction: no hit!
	    IVP_U_Hesse hesse_vec_os;
	    IVP_CLS.calc_hesse_vec_object_not_normized(edge, ledge_to_compare, &hesse_vec_os);

	    IVP_DOUBLE dot_p = hesse_vec_os.dot_product(&ray_dir_os);
	    if(dot_p > -P_DOUBLE_RES){ // @@@ select correct EPS
		// walked beyond the 'terminator'
		return IVP_FALSE;
	    }
	    const IVP_U_Float_Point *p0 = IVP_CLS.give_object_coords(edge, ledge_to_compare);

	    IVP_DOUBLE z = hesse_vec_os.dot_product(p0);
	    IVP_DOUBLE a = ray_start_point.dot_product( &hesse_vec_os ) - z;
	    IVP_DOUBLE b = ray_end_point.dot_product( &hesse_vec_os ) - z;

	    if (a*b >= 0 ){
		return IVP_FALSE; // does not hit because 
		// a>0 b>0   : both points are outside the object
		// a<0 b<0     both points are inside the object, no hits possible
	    }

    	    IVP_DOUBLE hit_dist = a/(a-b);
	    IVP_U_Point intersect_point; intersect_point.set_interpolate( &ray_start_point, &ray_end_point, hit_dist);
	    
	    // what neighboring triangle shall be visited? 
	    IVP_Unscaled_QR_Result qr;
	    IVP_CLS.calc_unscaled_qr_vals_F_space(ledge_to_compare, edge, &intersect_point,&qr);
	    
	    // search best edge and take corresponding neighbor triangle
	    {
    		const IVP_Compact_Edge *e;
		int j;
		for (e=edge,j=0; j<3; e=e->get_next(),j++){
		    if (qr.checks[j] > 0.0f ) continue;  // inside triangle
		    edge = e->get_opposite();
		    goto continue_with_next_triangle;
		    break;
		}
	    }
	    
	    // no movement possible anymore, take this triangle as final triangle
	    hit_dist *= ray_length;
	    hesse_vec_os.IVP_U_Point::normize();
    	    hit_listener->add_hit_object(object, ledge_to_compare, edge->get_triangle(), hit_dist, &hesse_vec_os);
	    return IVP_TRUE;
continue_with_next_triangle:;
	} // for (tricnt)
	
	// now we know that we are in an endless loop
	// ...
	IVP_IF(1){
	    printf("check_ray_against_compact_ledge: endl loop.\n");
	}
	return IVP_FALSE;
    }
}


void IVP_Ray_Solver_Os::check_ray_against_ledge_tree_node_os(const IVP_Compact_Ledgetree_Node *node){
    // Recursive function
    
    // Check whether ray hits the ledge tree node sphere
    IVP_U_Float_Point center; center.set( node->center.k);
    IVP_BOOL ray_hits_sphere = this->check_ray_against_sphere_os(&center, node->radius);
    if(!ray_hits_sphere) return; // no hit
    
    // Check ledge that is contained by this node.
    if(node->is_terminal() == IVP_TRUE){
	const IVP_Compact_Ledge *ledge = node->get_compact_ledge();
	this->check_ray_against_compact_ledge_os(ledge);
	return;
    }
    
    // check nodes children
    check_ray_against_ledge_tree_node_os(node->left_son());
    check_ray_against_ledge_tree_node_os(node->right_son());
}



void IVP_Ray_Solver_Os::check_ray_against_compact_surface_os(const IVP_Compact_Surface *compact_surface_to_compare){
    // Checks wether the specified compact surface is hit by the specified ray.
    // Adds the corresponding object to the min hash for each hit compact ledge.
    const IVP_Compact_Ledgetree_Node *lt_node_root;
    lt_node_root = compact_surface_to_compare->get_compact_ledge_tree_root();
    check_ray_against_ledge_tree_node_os(lt_node_root);
}



IVP_Ray_Solver_Os::IVP_Ray_Solver_Os(class IVP_Ray_Solver *so, IVP_Real_Object *obj){
    object = obj;
    hit_listener = so;
    
    IVP_Cache_Object *co = object->get_cache_object_no_lock();

    co->transform_position_to_object_coords(&so->ray_start_point, &this->ray_start_point);
    co->transform_vector_to_object_coords(&so->ray_direction, &this->ray_direction);
    this->ray_length = so->ray_length;
    this->ray_end_point.add_multiple(  &this->ray_start_point,  &this->ray_direction, this->ray_length);
    this->ray_center_point.set(&this->ray_start_point);
    this->ray_center_point.add_multiple( &this->ray_direction, this->ray_length * 0.5f);
    //co->remove_reference();
}


void IVP_Ray_Solver::check_ray_against_ball(IVP_Ball *ball)
{
    // what is the exact hit distance (if any)?
    IVP_Cache_Object *co = ball->get_cache_object_no_lock();
    IVP_U_Float_Point center_ws;
    center_ws.set( co->m_world_f_object.get_position());
    
    // does ray start inside the ball?
    IVP_DOUBLE quad_sphere_radius = ball->get_radius() * ball->get_radius();
    IVP_DOUBLE quad_delta_len = ray_start_point.quad_distance_to(&center_ws);
    if(quad_delta_len < quad_sphere_radius){
		//IVP_U_Point dummy_dir(0,1,0);
		//dummy_dir.set_to_zero();
		//this->add_hit_object(ball, NULL, NULL, 0.0f, &dummy_dir);
		return;
    }
    
    // calculate hit distance
    
    IVP_U_Point equ; // quadratic equation
    IVP_U_Point delta;
    delta.subtract(&ray_start_point, &center_ws);
    
    equ.k[0] = 1.0f;
    equ.k[1] = 2.0f * delta.dot_product(&ray_direction);
    equ.k[2] = quad_delta_len - quad_sphere_radius;
    
    IVP_U_Point solution;
    solution.solve_quadratic_equation_accurate(&equ); //TL: was fast
    if(solution.k[0] < 0.0f) return; // sphere missed!
		IVP_FLOAT hit_dist = (IVP_FLOAT)solution.k[1];
    if(hit_dist < 0.0f){
		hit_dist = (IVP_FLOAT)solution.k[2];
		if(hit_dist < 0.0f) return; // hit lies in negative ray direction
    }
    
	IVP_U_Point sur_hit_point;
	sur_hit_point.add_multiple(&ray_start_point, &ray_direction, hit_dist);
	IVP_U_Point hit_sur_normal;
	hit_sur_normal.subtract(&sur_hit_point, &center_ws);
	if(hit_sur_normal.normize()!=IVP_OK) {
		hit_sur_normal.set(1.0f, 0.0f, 0.0f);
	}

    add_hit_object(ball, NULL, NULL, hit_dist, &hit_sur_normal);
}

void IVP_Ray_Solver::check_ray_against_object(IVP_Real_Object *object){
    // Checks wether the specified object is hit by the specified ray.
    // Adds the corresponding object to the min hash for each hit.
    // check flags
    if (object){
	if(ray_flags & IVP_RAY_SOLVER_IGNORE_PHANTOMS){
	    if (object->get_controller_phantom())
		return;
	}
	if(ray_flags & IVP_RAY_SOLVER_IGNORE_MOVINGS){
	    if (!IVP_MTIS_IS_STATIC(object->get_movement_state()))
		return;
	}
	if(ray_flags & IVP_RAY_SOLVER_IGNORE_STATICS){
	    if (IVP_MTIS_IS_STATIC(object->get_movement_state()))
		return;
	}
    }
    
    IVP_OBJECT_TYPE type = object->get_type();
    switch(type){
    case IVP_BALL:
	{
	    IVP_Ball *ball = object->to_ball();
	    this->check_ray_against_ball(ball);
	    break;
	}
    case IVP_POLYGON:
	{
	    IVP_Polygon *poly = object->to_poly();
	    IVP_SurfaceManager *surman = poly->get_surface_manager();    
	    // Check all ledges within ray radius
	    surman->insert_all_ledges_hitting_ray(this, poly);
	    break;
	}
    default:
	break;
    }
}

void IVP_Ray_Solver_Group::check_ray_group_against_object(IVP_Real_Object *object){
    // Checks wether the specified object is hit by the specified ray.
    // Adds the corresponding object to the min hash for each hit.
    // check flags
    if (object){
	IVP_RAY_SOLVER_FLAGS ray_flags = ray_solvers[0]->ray_flags;

	if(ray_flags & IVP_RAY_SOLVER_IGNORE_PHANTOMS){
	    if (object->get_controller_phantom())
		return;
	}
	if(ray_flags & IVP_RAY_SOLVER_IGNORE_MOVINGS){
	    if (!IVP_MTIS_IS_STATIC(object->get_movement_state()))
		return;
	}
	if(ray_flags & IVP_RAY_SOLVER_IGNORE_STATICS){
	    if (IVP_MTIS_IS_STATIC(object->get_movement_state()))
		return;
	}
    }
    
    IVP_OBJECT_TYPE type = object->get_type();
    switch(type){
    case IVP_BALL:
	{
	    IVP_Ball *ball = object->to_ball();
	    for (int i = n_ray_solvers-1; i>=0;i--){
		ray_solvers[i]->check_ray_against_ball(ball);
	    }
	    break;
	}
    case IVP_POLYGON:
	{
	    IVP_Polygon *poly = object->to_poly();
	    IVP_SurfaceManager *surman = poly->get_surface_manager();    
	    for (int i = n_ray_solvers-1; i>=0;i--){
		surman->insert_all_ledges_hitting_ray(ray_solvers[i], poly);
	    }
	    // Check all ledges within ray radius
	    break;
	}
    default:
	break;
    }
}


void IVP_Ray_Solver::check_ray_against_node(IVP_OV_Node *node, IVP_OV_Tree_Manager *ov_tree_man) {
    // Recursive function
    
    // Check whether ray hits the long range cluster node.
    IVP_U_Float_Point luf_point, rlb_point; // left upper front / right lower back
    IVP_FLOAT cubesize;
    ov_tree_man->get_luf_coordinates_ws(node, &luf_point, &cubesize); // fills vars with left-upper-front corner's coordinates
    rlb_point.set(luf_point.k[0]+cubesize, luf_point.k[1]+cubesize, luf_point.k[2]+cubesize);
    IVP_BOOL ray_hits_cube = this->check_ray_against_cube(&luf_point, &rlb_point);
    if(!ray_hits_cube) return; // not hit
    
    // Check all objects that are contained by this node.
    {
	int i;
	for(i=node->elements.len()-1; i>=0; --i){
	    IVP_OV_Element *elem = node->elements.element_at(i);
	    
	    // Does ray hit object's surrounding sphere?
	    IVP_Core *core = elem->real_object->get_core();
	    IVP_U_Float_Point object_center_os(core->get_position_PSI());
	    
	    IVP_BOOL ray_hits_sphere = check_ray_against_sphere(&object_center_os, core->upper_limit_radius);
	    if(ray_hits_sphere){
		this->check_ray_against_object(elem->real_object);
	    }
	}
    }
    
    // Check node's children.
    {
	int i;
	for(i=node->children.len()-1; i>=0; --i){
	    IVP_OV_Node *child_node = node->children.element_at(i);
	    this->check_ray_against_node(child_node, ov_tree_man);
	}
    }
}

void IVP_Ray_Solver_Group::check_ray_group_against_node(IVP_OV_Node *node, IVP_OV_Tree_Manager *ov_tree_man) {
    // Recursive function
    
    // Check whether ray hits the long range cluster node.
    IVP_U_Float_Point luf_point, cube_center_point; // left upper front / right lower back
    IVP_FLOAT cubesize;
    ov_tree_man->get_luf_coordinates_ws(node, &luf_point, &cubesize); // fills vars with left-upper-front corner's coordinates
    IVP_FLOAT csh = cubesize * 0.5f;
    cube_center_point.set( luf_point.k[0] + csh, luf_point.k[1] + csh, luf_point.k[2] + csh );

    
    IVP_BOOL ray_hits_cube = this->check_ray_group_against_cube(&cube_center_point, cubesize);
    if(!ray_hits_cube) return; // not hit
    
    // Check all objects that are contained by this node.
    {
	int i;
	for(i=node->elements.len()-1; i>=0; --i){
	    IVP_OV_Element *elem = node->elements.element_at(i);
	    
	    // Does ray hit object's surrounding sphere?
	    IVP_Core *core = elem->real_object->get_core();
	    IVP_U_Float_Point object_center_os(core->get_position_PSI());
	    
	    IVP_BOOL ray_hits_sphere = check_ray_group_against_sphere(&object_center_os, core->upper_limit_radius);
	    if(ray_hits_sphere){
		this->check_ray_group_against_object(elem->real_object);
	    }
	}
    }
    
    // Check node's children.
    {
	int i;
	for(i=node->children.len()-1; i>=0; --i){
	    IVP_OV_Node *child_node = node->children.element_at(i);
	    this->check_ray_group_against_node(child_node, ov_tree_man);
	}
    }
}


void IVP_Ray_Solver::check_ray_against_all_objects_in_sim(const IVP_Environment *environment)
{
    // Checks wether any object in long range tree is hit by the specified ray.
    // Returns IVP_TRUE and adds the corresponding objects each time they are hit.
    // Returns IVP_FALSE if the object is not hit by the ray.
    
    // get root node from long range cluster manager
    IVP_OV_Tree_Manager *ov_tree_man = environment->get_ov_tree_manager();
    if(!ov_tree_man) return;
    
    IVP_OV_Node *node = ov_tree_man->root;
    
    check_ray_against_node(node, ov_tree_man);  
}

void IVP_Ray_Solver_Group::check_ray_group_against_all_objects_in_sim(const IVP_Environment *environment){
    // Checks wether any object in long range tree is hit by the specified ray.
    // Returns IVP_TRUE and adds the corresponding objects each time they are hit.
    // Returns IVP_FALSE if the object is not hit by the ray.
    
    // get root node from long range cluster manager
    IVP_OV_Tree_Manager *ov_tree_man = environment->get_ov_tree_manager();
    if(!ov_tree_man) return;
    
    IVP_OV_Node *node = ov_tree_man->root;
    
    check_ray_group_against_node(node, ov_tree_man);  
}


IVP_Ray_Solver_Group::IVP_Ray_Solver_Group( int n_ray_solvers_, IVP_Ray_Solver **ray_solvers_ ){
    ray_solvers = ray_solvers_;
    n_ray_solvers = n_ray_solvers_;
    
    center_ws.set_to_zero();

    for (int i =0; i < n_ray_solvers; i++){
	center_ws.add( &ray_solvers[i]->ray_center_point );
    }
    center_ws.mult(1.0f / n_ray_solvers);
    IVP_DOUBLE qradius = 0.0f;
    for (int j = 0; j < n_ray_solvers; j++){
	IVP_DOUBLE qdist = ray_solvers[j]->ray_start_point.quad_distance_to( &center_ws );
	if (qdist > qradius) qradius = qdist;

	qdist = ray_solvers[j]->ray_end_point.quad_distance_to( &center_ws );
	if (qdist > qradius) qradius = qdist;
    }
    radius = IVP_Inline_Math::sqrtd(qradius);
}

void IVP_Ray_Solver_Min::add_hit_object(IVP_Real_Object *object, const IVP_Compact_Ledge *compact_ledge, const IVP_Compact_Triangle *compact_triangle, IVP_DOUBLE hit_dist, IVP_U_Point *hit_sur_vec_os){
    if (hit_dist >= this->min_dist) return;
    min_dist = hit_dist;
    ray_hit.hit_real_object = object;
    ray_hit.hit_compact_ledge = compact_ledge;
    ray_hit.hit_compact_triangle = compact_triangle;
    ray_hit.hit_distance = hit_dist;
    ray_hit.hit_surface_direction_os.set( hit_sur_vec_os);
}

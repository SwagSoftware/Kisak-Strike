// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#include <ivp_physics.hxx>

#include <ivp_compact_ledge.hxx>
#include <ivp_compact_surface.hxx>

#ifndef WIN32
#	pragma implementation "ivp_surman_polygon.hxx"
#endif

#include <ivp_surman_polygon.hxx>
#include <ivp_compact_ledge_solver.hxx>

#include <ivu_min_hash.hxx>
#include <ivp_ray_solver.hxx>


class IVP_SurfaceManager_Polygon_Solver {
public:
    IVP_DOUBLE visitor_bb_min_x;	// #+# kill + convert to IVP_U_Float_Point
    IVP_DOUBLE visitor_bb_min_y;
    IVP_DOUBLE visitor_bb_min_z;
    IVP_DOUBLE visitor_bb_max_x;
    IVP_DOUBLE visitor_bb_max_y;
    IVP_DOUBLE visitor_bb_max_z;
    int traversion_depth;
    int max_traversion_depth;

    void traverse_cluster(const IVP_Compact_Ledgetree_Node *node, const IVP_U_Point *visitor_position_object_space, IVP_DOUBLE radius,
				IVP_U_BigVector<IVP_Compact_Ledge> *resulting_ledges);

};
						  
void IVP_SurfaceManager_Polygon_Solver::traverse_cluster(const IVP_Compact_Ledgetree_Node *node,
							 const IVP_U_Point *visitor_position_object_space,
							 IVP_DOUBLE radius,
							 IVP_U_BigVector<IVP_Compact_Ledge> *resulting_ledges)
{
    const IVP_DOUBLE work = IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;

    //this->traversion_depth++;
    //if ( this->traversion_depth > this->max_traversion_depth ) this->max_traversion_depth = this->traversion_depth;

#if 1
    // first check sphere against sphere.
    IVP_U_Point dist_vec;
    IVP_U_Float_Point center; center.set( node->center.k);

    dist_vec.subtract(&center, visitor_position_object_space);
    //IVP_DOUBLE qdistance = dist_vec.quad_length();
    IVP_DOUBLE check_dist = radius + node->radius;

    if ( dist_vec.quad_length() > check_dist * check_dist ) {
	IVP_IF(0) {
	    printf("Sphere\n");
	}
	return;
    }
#endif
    
#if 1
    // sphere collision detected! now check bounding box against bounding box.
    register IVP_FLOAT work2 = work * node->radius;

    if ( IVP_Inline_Math::fabsd(dist_vec.k[0]) >= (node->box_sizes[0] * work2 + radius) ) {
	IVP_IF(0) {
	    printf("Box\n");
	}
	return;
    }
    if ( IVP_Inline_Math::fabsd(dist_vec.k[1]) >= (node->box_sizes[1] * work2 + radius) ) {
	IVP_IF(0) {
	    printf("Box\n");
	}
	return;
    }
    if ( IVP_Inline_Math::fabsd(dist_vec.k[2]) >= (node->box_sizes[2] * work2 + radius) ) {
	IVP_IF(0) {
	    printf("Box\n");
	}
	return;
    }

    //printf("Node    : %f - %f *** %f - %f *** %f - %f\n", min_x, max_x, min_y, max_y, min_z, max_z);
    //printf("Intruder: %f - %f *** %f - %f *** %f - %f\n", s_bb_min_x, s_bb_max_x, s_bb_min_y, s_bb_max_y, s_bb_min_z, s_bb_max_z);
#endif
    const IVP_Compact_Ledge *hull = node->get_compact_hull();
    if ( hull ){
	resulting_ledges->add(&(IVP_Compact_Ledge&)*hull);
	return;
    }
    
    if ( node->is_terminal() ) {
	const IVP_Compact_Ledge *ledge = node->get_compact_ledge();
	if (ledge->get_n_triangles() == 2){	// IVP_DOUBLE triangle allows deeper search
	    const IVP_Compact_Triangle *triangle = ledge->get_first_triangle();
	    IVP_DOUBLE qdist = IVP_CLS.calc_qlen_PF_F_space(ledge, triangle, visitor_position_object_space);
	    if ( qdist > radius * radius ) {
		return;
	    }
	}
	resulting_ledges->add((IVP_Compact_Ledge *)ledge);
	return;
    }
    traverse_cluster(node->left_son(),  visitor_position_object_space, radius, resulting_ledges );
    //this->traversion_depth--;
    traverse_cluster(node->right_son(), visitor_position_object_space, radius, resulting_ledges );
    //this->traversion_depth--;

    return;
}

void IVP_SurfaceManager_Polygon::insert_all_ledges_hitting_ray(IVP_Ray_Solver *ray_solver, IVP_Real_Object *object){
    IVP_Ray_Solver_Os ray_solver_os( ray_solver, object);
    const IVP_Compact_Surface *cs = this->get_compact_surface();
    ray_solver_os.check_ray_against_compact_surface_os(cs);
}

void IVP_SurfaceManager_Polygon::get_all_ledges_within_radius(const IVP_U_Point *visitor_position_object_space,  IVP_DOUBLE radius,
							      const IVP_Compact_Ledge *root_ledge, IVP_Real_Object * /*other_object*/, const IVP_Compact_Ledge * /*other_reference_ledge*/,
							      IVP_U_BigVector<IVP_Compact_Ledge> *resulting_ledges){
    IVP_SurfaceManager_Polygon_Solver sps;
    sps.visitor_bb_max_x = visitor_position_object_space->k[0] + radius;
    sps.visitor_bb_max_y = visitor_position_object_space->k[1] + radius;
    sps.visitor_bb_max_z = visitor_position_object_space->k[2] + radius;
    sps.visitor_bb_min_x = visitor_position_object_space->k[0] - radius;
    sps.visitor_bb_min_y = visitor_position_object_space->k[1] - radius;
    sps.visitor_bb_min_z = visitor_position_object_space->k[2] - radius;
    sps.traversion_depth = 0;
    sps.max_traversion_depth = 0;
    if (!root_ledge) {
	const IVP_Compact_Ledgetree_Node *root_node = compact_surface->get_compact_ledge_tree_root();
	sps.traverse_cluster(root_node, visitor_position_object_space, radius, resulting_ledges );
    }else{
	const IVP_Compact_Ledgetree_Node *root_node = root_ledge->get_ledgetree_node();
	sps.traverse_cluster(root_node->left_son(), visitor_position_object_space, radius, resulting_ledges );
	sps.traverse_cluster(root_node->right_son(), visitor_position_object_space, radius, resulting_ledges );
    }
    return;
}

void IVP_SurfaceManager_Polygon::get_all_terminal_ledges(IVP_U_BigVector<IVP_Compact_Ledge> *resulting_ledges){
    IVP_Compact_Ledge_Solver::get_all_ledges( compact_surface, resulting_ledges );
}

void IVP_SurfaceManager_Polygon::get_mass_center(IVP_U_Float_Point *mass_center_out) const{
    mass_center_out->set(compact_surface->mass_center.k);
}

void IVP_SurfaceManager_Polygon::get_rotation_inertia( IVP_U_Float_Point *rotation_inertia_out ) const {
    rotation_inertia_out->set( compact_surface->rotation_inertia.k);
}

void IVP_SurfaceManager_Polygon::get_radius_and_radius_dev_to_given_center(const IVP_U_Float_Point *center, IVP_FLOAT *radius, IVP_FLOAT *radius_deviation) const {
    IVP_U_Float_Point mass_center;    
    mass_center.set(compact_surface->mass_center.k);
    
    IVP_DOUBLE center_shift = mass_center.quad_distance_to( center );
    center_shift = IVP_Inline_Math::sqrtd(center_shift);
    *radius = compact_surface->upper_limit_radius + center_shift;
    *radius_deviation = compact_surface->max_factor_surface_deviation * IVP_COMPACT_SURFACE_DEVIATION_STEP_SIZE *
	compact_surface->upper_limit_radius + center_shift;
}


const IVP_Compact_Ledge *IVP_SurfaceManager_Polygon::get_single_convex() const{
    const IVP_Compact_Ledgetree_Node *root = compact_surface->get_compact_ledge_tree_root();
    if (root->is_terminal()) return root->get_compact_ledge();
    return root->get_compact_hull();
}

IVP_SURMAN_TYPE IVP_SurfaceManager_Polygon::get_type(){
    return IVP_SURMAN_POLYGON; 

}


void IVP_SurfaceManager_Polygon::add_reference_to_ledge(const IVP_Compact_Ledge *ledge)
{
    IVP_USE(ledge);
    return;
}

void IVP_SurfaceManager_Polygon::remove_reference_to_ledge(const IVP_Compact_Ledge *ledge)
{
    IVP_USE(ledge);
    return;
}



IVP_SurfaceManager_Polygon::IVP_SurfaceManager_Polygon()
{
}

IVP_SurfaceManager_Polygon::~IVP_SurfaceManager_Polygon()
{
  //    if (!compact_surface_is_shared){
  //	P_FREE_ALIGNED(compact_surface);
  // }
}



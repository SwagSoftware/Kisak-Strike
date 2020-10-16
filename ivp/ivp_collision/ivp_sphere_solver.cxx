#include <ivp_physics.hxx>
#include <ivp_clustering_longrange.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_surman_polygon.hxx>
#include <ivp_compact_surface.hxx>
#include <ivp_compact_ledge.hxx>

#include <ivp_sphere_solver.hxx>

/////////////////////////////////////////////////////////////////////
// start of file local helpers
/////////////////////////////////////////////////////////////////////

static inline bool AABB_overlaps_AABB_d(IVP_U_Float_Point* rlb1, 
											IVP_U_Float_Point* pos1,
											IVP_U_Float_Point* rlb2, 
											IVP_U_Float_Point* pos2)
{
	if(IVP_Inline_Math::fabsd(pos1->k[0] - pos2->k[0]) > (rlb1->k[0] + rlb2->k[0]))
		return false;

	if(IVP_Inline_Math::fabsd(pos1->k[1] - pos2->k[1]) > (rlb1->k[1] + rlb2->k[1]))
		return false;

	if(IVP_Inline_Math::fabsd(pos1->k[2] - pos2->k[2]) > (rlb1->k[2] + rlb2->k[2]))
		return false;

	return true;
}

static inline IVP_BOOL sphere_overlaps_sphere_d(IVP_U_Point* pos1, 
												IVP_DOUBLE rad1,
												const IVP_U_Point* pos2, 
												IVP_FLOAT rad2)
{
	IVP_U_Float_Point center_distance_vec;
	center_distance_vec.subtract(pos1, pos2);

	IVP_FLOAT center_distance = center_distance_vec.quad_length();

	if( center_distance > (rad1 + rad2) * (rad1 + rad2))
		return IVP_FALSE;
	else
		return IVP_TRUE;
}

static IVP_BOOL found_valid_sphere_node = IVP_FALSE;

static inline void sphere_against_ledgetree_node(const IVP_Compact_Ledgetree_Node* cln, IVP_U_Point* query_pos, IVP_DOUBLE query_rad, int current_depth, int max_depth)
{
	IVP_U_Point node_pos(cln->center.k[0], cln->center.k[1], cln->center.k[2]);
	IVP_FLOAT node_rad = cln->radius;

	if(sphere_overlaps_sphere_d(query_pos, query_rad, &node_pos, node_rad))
	{
		if(current_depth < max_depth && !cln->is_terminal())
		{
			sphere_against_ledgetree_node(cln->left_son(), query_pos, query_rad, current_depth + 1, max_depth);
			sphere_against_ledgetree_node(cln->right_son(), query_pos, query_rad, current_depth + 1, max_depth);
		}
		else
		{
			found_valid_sphere_node = IVP_TRUE;
		}
	}

	return;
}

/**
 * If obj has more than one convex part, sphere_against_object will traverse
 * the obj sphere tree down as far as max_depth, and use that set of spheres
 * for culling against the query sphere, rather than just use the bounding sphere
 * (i.e. sphere at the root of the sphere tree). This should give a better
 * approximation of the object being queried.
 */
static inline IVP_BOOL sphere_against_object(IVP_Real_Object* obj, IVP_U_Point* query_pos, IVP_DOUBLE query_rad, int max_depth)
{
	found_valid_sphere_node = IVP_FALSE;

	if(obj->get_type() == IVP_POLYGON)
	{
		const IVP_Compact_Surface* cs = static_cast<IVP_SurfaceManager_Polygon*>(obj->get_surface_manager())->get_compact_surface();
		const IVP_Compact_Ledgetree_Node* cln = cs->get_compact_ledge_tree_root();

		sphere_against_ledgetree_node(cln, query_pos, query_rad, 0, max_depth);

		return found_valid_sphere_node;
	}
	else if(obj->get_type() == IVP_BALL)
	{
		return sphere_overlaps_sphere_d(query_pos, query_rad, obj->get_core()->get_position_PSI(), obj->get_core()->upper_limit_radius);
	}

	return IVP_FALSE;
}

/////////////////////////////////////////////////////////////////////
// end of file local helpers
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
// Implementation of IVP_Sphere_Solver begins
/////////////////////////////////////////////////////////////////////

IVP_BOOL IVP_Sphere_Solver::check_sphere_against_object(const IVP_Sphere_Solver_Template* tmpl, IVP_Real_Object* obj)
{
	IVP_U_Point obj_center;

	m_center.set(&tmpl->center);
	m_radius = tmpl->radius;
	m_max_traversal_depth = tmpl->max_traversal_depth;

	IVP_Cache_Object* co = obj->get_cache_object();
	co->transform_position_to_object_coords(&m_center, &obj_center);
	co->remove_reference();

	return sphere_against_object(obj, &obj_center, m_radius, m_max_traversal_depth);
}

void IVP_Sphere_Solver::check_sphere_against_environment(const IVP_Sphere_Solver_Template* tmpl, IVP_Environment* env)
{
	m_center.set(&tmpl->center);
	m_radius = tmpl->radius;
	m_max_traversal_depth = tmpl->max_traversal_depth;

	m_tree_manager = env->get_ov_tree_manager();
	IVP_OV_Node* root = m_tree_manager->root;

	IVP_U_Float_Point sphere_bb_rlb, sphere_bb_pos;
	
	sphere_bb_rlb.set(m_radius, m_radius, m_radius);
	sphere_bb_pos.set(m_center.k[0], m_center.k[1], m_center.k[2]);

	recursively_collect_intruding_objects(root, &sphere_bb_rlb, &sphere_bb_pos);
	return;
}

void IVP_Sphere_Solver::recursively_collect_intruding_objects(IVP_OV_Node* node, IVP_U_Float_Point* query_rlb, IVP_U_Float_Point* query_pos)
{
	IVP_U_Float_Point node_luf, node_rlb, node_pos;
	IVP_FLOAT node_size;

	m_tree_manager->get_luf_coordinates_ws(node, &node_luf, &node_size);
	node_rlb.set(node_size / 2.0f, node_size / 2.0f, node_size / 2.0f);
	node_pos.set(node_luf.k[0] + (node_size / 2.0f), node_luf.k[1] + (node_size / 2.0f), node_luf.k[2] + (node_size / 2.0f));

	{
		if(AABB_overlaps_AABB_d(&node_rlb, &node_pos, query_rlb, query_pos))
		{
			for(int i = 0; i < node->elements.len(); i++)
			{
				IVP_Real_Object* query_object = node->elements.element_at(i)->real_object;

				const IVP_U_Point* pos = query_object->get_core()->get_position_PSI();
				IVP_FLOAT rad = query_object->get_core()->upper_limit_radius;

				// check object bounding sphere against sphere query
				if(sphere_overlaps_sphere_d(&m_center, m_radius, pos, rad))
				{
					IVP_U_Point query_center_object_space;

					// transform the query sphere center into object local space
					IVP_Cache_Object* co = query_object->get_cache_object();
					co->transform_position_to_object_coords(&m_center, &query_center_object_space);

					if(sphere_against_object(query_object, &query_center_object_space, m_radius, m_max_traversal_depth))
					{
						m_intruding_objects.add(query_object);
					}

					co->remove_reference();
				}
			}

			{
				for(int i = 0; i < node->children.len(); i++)
				{
					recursively_collect_intruding_objects(node->children.element_at(i), query_rlb, query_pos);
				}
			}
		}
	}

	return;
}

/////////////////////////////////////////////////////////////////////
// Implementation of IVP_Sphere_Solver ends
/////////////////////////////////////////////////////////////////////
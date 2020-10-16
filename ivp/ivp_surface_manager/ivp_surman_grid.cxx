// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#include <ivp_physics.hxx>

#include <ivp_compact_ledge.hxx>
#include <ivp_compact_grid.hxx>

#ifndef WIN32
#	pragma implementation "ivp_surman_grid.hxx"
#endif

#include <ivp_ray_solver.hxx>
#include <ivp_surman_grid.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_compact_ledge_solver.hxx>


void IVP_SurfaceManager_Grid::traverse_grid(	const IVP_U_Point &visitor_position_object_space,
						IVP_DOUBLE radius,
						IVP_U_BigVector<IVP_Compact_Ledge> *resulting_ledges) const
{
	IVP_U_Point visitor_position_grid_space;
	compact_grid->m_grid_f_object.vmult4( &visitor_position_object_space, &visitor_position_grid_space);
	
	IVP_DOUBLE	radius_gs = radius * compact_grid->inv_grid_size;

	IVP_DOUBLE fmin_row = (visitor_position_grid_space.k[0] - radius_gs);
	IVP_DOUBLE fmax_row = (visitor_position_grid_space.k[0] + radius_gs);

	IVP_DOUBLE fmin_col = (visitor_position_grid_space.k[1] - radius_gs);
	IVP_DOUBLE fmax_col = (visitor_position_grid_space.k[1] + radius_gs);

	if (fmin_row < 0) fmin_row = 0;
	if (fmin_col < 0) fmin_col = 0;

	if (fmax_row >= IVP_DOUBLE(compact_grid->n_rows) ) fmax_row = compact_grid->n_rows-0.5f;
	if (fmax_col >= IVP_DOUBLE(compact_grid->n_columns) ) fmax_col = compact_grid->n_columns-0.5f;

	int min_row = int(fmin_row);
	int min_col = int(fmin_col);

	int max_row = int(fmax_row);
	int max_col = int(fmax_col);

	char flag_buffer[8192];			// flags are needed to flag compact ledges already in vector
	char *flags;
	if ( compact_grid->n_compact_ledges >= 8192 ){
		flags = p_calloc( sizeof(char), compact_grid->n_compact_ledges);
	}else{
		flags = &flag_buffer[0];
		memset(flags,0,compact_grid->n_compact_ledges);
	}

	const IVP_Compact_Grid_Element *ge = compact_grid->get_grid_elements();
	for (int row = min_row; row <= max_row; row++){
		for (int col = min_col; col <= max_col; col++){
			int idx = row * compact_grid->n_columns + col;
			for (int i =0;i<2; i++){
				int ledge_id = ge[idx].compact_ledge_index[i];
				if (ledge_id < 0) continue;						// flagged as not existent
				if ( flags[ledge_id] ) continue;				// ledge already in our vector
				flags[ledge_id] = 1;
				const IVP_Compact_Ledge *ledge = compact_grid->get_compact_ledge_at(ledge_id);
#ifdef DEBUG				
				IVP_Compact_Ledge_Solver::check_ledge(ledge);
#endif				
				if (ledge->get_n_triangles() == 2){	// IVP_DOUBLE triangle allows deeper search
					const IVP_Compact_Triangle *triangle = ledge->get_first_triangle();
					IVP_DOUBLE qdist = IVP_CLS.calc_qlen_PF_F_space(ledge, triangle, &visitor_position_object_space);
					if ( qdist > radius * radius ) continue;
				}

				resulting_ledges->add((IVP_Compact_Ledge *)ledge);
			}
		}
	}
	if (flags != &flag_buffer[0]) P_FREE(flags);		// get rid of malloced array
}



void IVP_SurfaceManager_Grid::get_all_ledges_within_radius(const IVP_U_Point *visitor_position_object_space,     IVP_DOUBLE radius,
							   const IVP_Compact_Ledge * /*root_ledge*/, IVP_Real_Object * /*other_object*/, const IVP_Compact_Ledge * /*other_reference_ledge*/,
							   IVP_U_BigVector<IVP_Compact_Ledge> *resulting_ledges){
    traverse_grid(*visitor_position_object_space, radius, resulting_ledges);
}

void IVP_SurfaceManager_Grid::insert_all_ledges_hitting_ray(IVP_Ray_Solver *ray_solver, IVP_Real_Object *object){
	IVP_Vector_of_Ledges_256 ledges;
    IVP_Ray_Solver_Os ray_solver_os( ray_solver, object);

	IVP_U_Point center(&ray_solver_os.ray_center_point);
	get_all_ledges_within_radius( &center, ray_solver_os.ray_length * 0.5f, NULL,NULL, NULL, &ledges);

	for (int i=ledges.len()-1;i>=0;i--){
		const IVP_Compact_Ledge *l = ledges.element_at(i);
		ray_solver_os.check_ray_against_compact_ledge_os(l);
	}
}

void IVP_SurfaceManager_Grid::get_all_terminal_ledges(IVP_U_BigVector<IVP_Compact_Ledge> *resulting_ledges){
    IVP_U_Point visitor_position_object_space; visitor_position_object_space.set_to_zero();
    traverse_grid(visitor_position_object_space, P_DOUBLE_MAX, resulting_ledges);
}

void IVP_SurfaceManager_Grid::get_mass_center(IVP_U_Float_Point *mass_center_out) const{
    mass_center_out->set(compact_grid->center.k);
}

void IVP_SurfaceManager_Grid::get_rotation_inertia( IVP_U_Float_Point *rotation_inertia_out ) const {
	IVP_DOUBLE r = compact_grid->radius * compact_grid->radius;
    rotation_inertia_out->set( r, r, r);
}

void IVP_SurfaceManager_Grid::get_radius_and_radius_dev_to_given_center(const IVP_U_Float_Point * /*center*/ , IVP_FLOAT *radius, IVP_FLOAT *radius_deviation) const {
    *radius = compact_grid->radius;
    *radius_deviation = compact_grid->radius;
}


const IVP_Compact_Ledge *IVP_SurfaceManager_Grid::get_single_convex() const{
    return NULL;
}

IVP_SURMAN_TYPE IVP_SurfaceManager_Grid::get_type(){
    return IVP_SURMAN_POLYGON; 
}

void IVP_SurfaceManager_Grid::add_reference_to_ledge(const IVP_Compact_Ledge * /*ledge*/ ){

}
 
void IVP_SurfaceManager_Grid::remove_reference_to_ledge(const IVP_Compact_Ledge * /*ledge*/ ){


}

IVP_SurfaceManager_Grid::~IVP_SurfaceManager_Grid(){

}

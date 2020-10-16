// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

/********************************************************************************
 *	File:	       	ivp_rot_inertia_solver.cxx	
 *	Description:	function collection
 ********************************************************************************/


#include <ivp_physics.hxx>
#include <ivu_float.hxx>
#include <ivu_hash.hxx>

#include <ivp_compact_ledge.hxx>
#include <ivp_rot_inertia_solver.hxx>
#include <ivp_compact_ledge_solver.hxx>

/********************************************************************************
 *	Names:	       	p_find_center_given_xyz
 *	Description:	integrate the volume of a compact ledge: dz*dy/dx
 *	In:
 *		x,y,z:	permutation of the original axis: x is index of x-axis ...
 *		transform:	transformation matrix of the original points
 *	Out:
 *		center:		the center of mass x-axis
 *		mass:		the volume of the object
 *		inertia:	the inertia given that the object is centered
 ********************************************************************************/

IVP_Compact_Ledge_Mass_Center_Solver::IVP_Compact_Ledge_Mass_Center_Solver(const IVP_U_Matrix *transform_in){
    transform = transform_in;
    msum = 0.0f;
    mzsum = 0.0f;
    mzzsum = 0.0f;
}

IVP_Compact_Ledge_Find_Mass_Center::IVP_Compact_Ledge_Find_Mass_Center(){
    sum_det = 0.0f;
    qsum_surface = 0.0f;
    sum_mass.set_to_zero();
}

void IVP_Compact_Ledge_Mass_Center_Solver::integrate_triangle( const IVP_Compact_Ledge *ledge, const IVP_Compact_Triangle *triangle, int x,int y,int z){

    IVP_U_Point hesse2;
    const IVP_Compact_Edge *edge = triangle->get_first_edge();
    IVP_CLS.calc_hesse_vec_object_not_normized( edge, ledge, &hesse2);
    hesse2.normize();
    IVP_U_Point *hesse = &hesse2;
    
    IVP_DOUBLE fzdy; // Steigung der Flaeche in z nach dy
    IVP_DOUBLE fydz;
    IVP_BOOL three_edge_area;	// flag: integrate triangle or square
	    
    if ( IVP_DOUBLE(IVP_Inline_Math::fabsd(hesse->k[z])) < IVP_DOUBLE(IVP_Inline_Math::fabsd(hesse->k[y])) ){ 
	three_edge_area = IVP_FALSE;	// intagrate triangle plus square
	fydz = 0.5f *  hesse->k[z] / hesse->k[y];
	fzdy = 0.0f;
    }else{
	three_edge_area = IVP_TRUE;	// Integrate triangle only
	if ( IVP_Inline_Math::fabsd(hesse->k[z]) > P_DOUBLE_RES ) {
	    fzdy = -0.5f * hesse->k[y] / hesse->k[z];  // hesse form is vertical to area !!
	} else {
	    fzdy =  0.0f;
	}
	fydz = 0.0f;
    }
    int l;
    IVP_DOUBLE area_msum = 0.0f;
    IVP_DOUBLE area_mzsum = 0.0f;
    IVP_DOUBLE area_mzzsum = 0.0f;
    for (l=0;l<3;l++){
	IVP_U_Float_Point p0,p1;
	{	// get the point coords
	    const IVP_Compact_Edge *e = triangle->get_edge(l);
	    const IVP_Compact_Edge *en = e->get_next();
	    const IVP_U_Float_Point *p0_object = IVP_CLS.give_object_coords(e,ledge);
	    const IVP_U_Float_Point *p1_object = IVP_CLS.give_object_coords(en,ledge);
	    transform->vimult4(p0_object, &p0);		// start point
	    transform->vimult4(p1_object,&p1);		// start point
	}

	IVP_U_Point vec; vec.subtract(&p1,&p0);
	if (IVP_Inline_Math::fabsd(vec.k[x]) < P_DOUBLE_RES * vec.real_length() ){
	    continue;
	}
	IVP_DOUBLE lydx = vec.k[y] / vec.k[x]; 		// Steigung der Linie in y nach dx
	IVP_DOUBLE ly0 = p0.k[y]  - lydx * p0.k[x]; 	// intersection point of line with x-axis
		
	IVP_DOUBLE ka,kb,kc;
	if (three_edge_area){
	    // loese Integrale im Bereich [x0 x1]: (y0 + x lydx)(y0 + x lydx) fzdy [x][x] dx
	    ka = fzdy * ly0 * ly0;
	    kb = fzdy * 2.0f * ly0 * lydx;
	    kc = fzdy * lydx * lydx;
	}else{
	    // loese Integral (x*y)dx = (y0 + x lydx)(lz0 + x lzdx)dx
	    //					+ fydz(z0 + x lzdx)(lz0 + x lzdx) 
	    IVP_DOUBLE lzdx = vec.k[z] / vec.k[x];
	    IVP_DOUBLE lz0 = p0.k[z]  - lzdx * p0.k[x];
	    ka = (ly0 + fydz * lz0) * lz0;
	    kb = ly0 * lzdx + lydx * lz0 + 2.0f * fydz * lz0 * lzdx;
	    kc = (lydx + fydz * lzdx)*  lzdx;
	}		
	IVP_DOUBLE x0 = p0.k[x];
	IVP_DOUBLE x1 = p1.k[x];
	IVP_DOUBLE	dx = x1 - x0; 
	IVP_DOUBLE	dx2 = (x1*x1 - x0 * x0)				*.5f;
	IVP_DOUBLE	dx3 = (x1*x1*x1 - x0*x0*x0)			* (1.0f/ 3.0f);
	IVP_DOUBLE	dx4 = (x1*x1*x1*x1 - x0*x0*x0*x0) 		* .25f;
	IVP_DOUBLE	dx5 = (x1*x1*x1*x1*x1 - x0*x0*x0*x0*x0) 	*.2f;
	IVP_DOUBLE	f =  dx * ka + dx2 * kb + dx3 * kc;	// Flaecheninhalt
	IVP_DOUBLE	k = dx2 * ka + dx3 * kb + dx4 * kc;	// Kippmoment
	IVP_DOUBLE	m = dx3 * ka + dx4 * kb + dx5 * kc;	// Drehmomentum
	area_msum += f;
	area_mzsum += k;
	area_mzzsum += m;
    }
    msum += area_msum;
    mzsum += area_mzsum;
    mzzsum += area_mzzsum;
}


void IVP_Compact_Ledge_Find_Mass_Center::integrate_triangle( const IVP_Compact_Ledge *ledge, const IVP_Compact_Triangle *triangle){
    const IVP_Compact_Edge *edge = triangle->get_first_edge();
    const IVP_U_Float_Point *p0 = IVP_CLS.give_object_coords(edge, ledge);
    const IVP_U_Float_Point *p1 = IVP_CLS.give_object_coords(edge->get_next(), ledge);
    const IVP_U_Float_Point *p2 = IVP_CLS.give_object_coords(edge->get_prev(), ledge);

    IVP_U_Float_Point surface_normal; surface_normal.inline_set_vert_to_area_defined_by_three_points(p0, p2, p1);
    IVP_DOUBLE det = surface_normal.dot_product( p0 );

    qsum_surface += surface_normal.quad_length();
    sum_det += det;
    sum_mass.add_multiple( p0, det * 0.25f);
    sum_mass.add_multiple( p1, det * 0.25f);
    sum_mass.add_multiple( p2, det * 0.25f);
}
    

void IVP_Compact_Ledge_Find_Mass_Center::integrate_ledge( const IVP_Compact_Ledge *ledge){
    const IVP_Compact_Triangle *tri;
    int t;
    for (t = 0, tri = ledge->get_first_triangle();
	 t< ledge->get_n_triangles();
	 t++, tri= tri->get_next_tri() ){
	integrate_triangle( ledge,tri);
    }
}

void IVP_Compact_Ledge_Find_Mass_Center::integrate_ledges( IVP_U_BigVector<IVP_Compact_Ledge> *v_ledges){
   for (int l = v_ledges->len()-1; l>=0;l--){
	const IVP_Compact_Ledge *ledge = v_ledges->element_at(l);
	integrate_ledge(ledge);
   }
}

void IVP_Rot_Inertia_Solver::find_center_given_xyz(IVP_U_BigVector<IVP_Compact_Ledge> *v_ledges,
						   int x,int y,int z, const IVP_U_Matrix *transform,
						   IVP_DOUBLE *center, IVP_DOUBLE *mass, IVP_DOUBLE *inertia)
{
    /* Search Center value, relativ x-Axis */
    IVP_Compact_Ledge_Mass_Center_Solver solver(transform);
    for (int l = v_ledges->len()-1; l>=0;l--){
	const IVP_Compact_Ledge *ledge = v_ledges->element_at(l);
	const IVP_Compact_Triangle *tri;
	int t;
	for (t = 0, tri = ledge->get_first_triangle();
	     t< ledge->get_n_triangles();
	     t++, tri= tri->get_next_tri() ){

	    solver.integrate_triangle( ledge,tri,x,y,z);
	}
    }
    if(solver.msum < P_DOUBLE_EPS){
	*mass = 0.0f;
	*center = 0.0f;
	*inertia = 1.0f;
    }else{
	*center = solver.mzsum/solver.msum;
	*mass = solver.msum;
	*inertia = solver.mzzsum/solver.msum;
    }
}

static IVP_RETURN_TYPE localCalcMassCenterAndRotInertia(
	IVP_U_BigVector<IVP_Compact_Ledge>* all_ledges,
	IVP_U_Point* mass_center_out,
	IVP_U_Point* rotation_inertia_out)
{
    IVP_U_Matrix transform;
    transform.init();

    IVP_Compact_Ledge_Find_Mass_Center fmc;
    fmc.integrate_ledges( all_ledges );

    // do not check the surface if height of object is very smally
    IVP_DOUBLE surface = IVP_Inline_Math::sqrtd( fmc.qsum_surface );
    if( fmc.sum_det > surface * surface * surface * 1e-9f){
	transform.init();
	transform.vv = fmc.get_mass_center();
	mass_center_out->set(transform.get_position());

	IVP_DOUBLE volume;
	IVP_U_Point rot_inertia_2d;
	IVP_U_Point check_center;
	IVP_Rot_Inertia_Solver::find_center_given_xyz(all_ledges,0,1,2,&transform,&check_center.k[0],&volume,&rot_inertia_2d.k[0]);
	IVP_Rot_Inertia_Solver::find_center_given_xyz(all_ledges,1,2,0,&transform,&check_center.k[1],&volume,&rot_inertia_2d.k[1]);
	IVP_Rot_Inertia_Solver::find_center_given_xyz(all_ledges,2,0,1,&transform,&check_center.k[2],&volume,&rot_inertia_2d.k[2]);

	
	{		// calc 3d rotation inertia based on 2d values
	    IVP_DOUBLE a = rot_inertia_2d.k[0];
	    IVP_DOUBLE b = rot_inertia_2d.k[1];
	    IVP_DOUBLE c = rot_inertia_2d.k[2];
	    a *=a; b *= b; c *= c;	    
	    IVP_DOUBLE sa = IVP_Fast_Math::sqrt(b+c);
	    IVP_DOUBLE sb = IVP_Fast_Math::sqrt(a+c);
	    IVP_DOUBLE sc = IVP_Fast_Math::sqrt(a+b);
	    rotation_inertia_out->set(sa,sb,sc);
	}
	return IVP_OK;
    }else{	// no valid masses found, take geom center
	IVP_DOUBLE radius;
        IVP_U_Point min_extents, max_extents;

	IVP_CLS.calc_bounding_box(all_ledges->element_at(0), &min_extents, &max_extents);
	for (int l = all_ledges->len()-1; l>0;l--){
            IVP_U_Point mi, ma;
	    const IVP_Compact_Ledge *ledge = all_ledges->element_at(l);
	    IVP_CLS.calc_bounding_box(ledge, &mi, &ma);
	    min_extents.line_min( &mi );
	    max_extents.line_max( &ma );
	}
	mass_center_out->set_interpolate( &min_extents, &max_extents, 0.5f);
	radius = 0.5f * IVP_Inline_Math::sqrtd( min_extents.quad_distance_to( &max_extents));
	IVP_DOUBLE estimate_inertia = radius * radius * 0.5f;
	rotation_inertia_out->set(estimate_inertia, estimate_inertia, estimate_inertia);
	return IVP_FAULT;
    }
}

IVP_RETURN_TYPE IVP_Rot_Inertia_Solver::calc_mass_center_and_rotation_inertia(
    const IVP_Compact_Surface *c_surface_in,
    IVP_U_Point *mass_center_out,
    IVP_U_Point *rotation_inertia_out)
{
    IVP_U_BigVector<IVP_Compact_Ledge> all_ledges(128);
    IVP_Compact_Ledge_Solver::get_all_ledges(c_surface_in, &all_ledges );

	return localCalcMassCenterAndRotInertia(&all_ledges, mass_center_out, rotation_inertia_out);
}

#ifdef HAVOK_MOPP
IVP_RETURN_TYPE IVP_Rot_Inertia_Solver::calc_mass_center_and_rotation_inertia(
	const IVP_Compact_Mopp* c_mopp_in,
	IVP_U_Point* mass_center_out,
	IVP_U_Point* rotation_inertia_out)
{
	IVP_U_BigVector<IVP_Compact_Ledge> all_ledges(128);
	IVP_Compact_Ledge_Solver::get_all_ledges(c_mopp_in, &all_ledges);

	return localCalcMassCenterAndRotInertia(&all_ledges, mass_center_out, rotation_inertia_out);
}
#endif // HAVOK_MOPP
// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

//#include <ivp_surman_polygon.hxx>
#include <ivp_compact_ledge.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_compact_ledge_solver.hxx>
#include <ivp_halfspacesoup.hxx>


/*------------------------------------------------------------------------
 * insert_plane_into_list
 * ======================
 *
 * insert a new plane into planelist; either skip plane if there is already
 * a almost(!) parallel (but closer to object) plane in the list or remove
 * old (almost parallel) plane from list if new plane is closer to the
 * IVP_U_Hesses are deleted when necessary
 ------------------------------------------------------------------------*/

void IVP_Halfspacesoup::add_halfspace(const IVP_U_Hesse *plane)
{
    IVP_U_Vector<IVP_U_Hesse> *planes = this;
    int i;
    IVP_U_Vector<IVP_U_Hesse> planes_to_delete;

    for (i=0; i< planes->len(); i++) {
	IVP_U_Hesse *plane_old = planes->element_at(i);
	
	IVP_DOUBLE dp = plane->dot_product(plane_old);
	if ( dp > 1-HALFSPACESOUP_TOLERANCE ) { // plane is almost identical (position, orientation and direction) to a previous one in list
	    if ( plane_old->hesse_val < plane->hesse_val ) { // drop plane that is further away
		return;
	    }
	    planes_to_delete.add(plane_old);
	}

    }

    planes->add(new IVP_U_Hesse(*plane));

    for (i=planes_to_delete.len()-1; i>=0; i--) {
	IVP_U_Hesse *old_plane = planes_to_delete.element_at(i);
	planes->remove(old_plane);
	P_DELETE(old_plane);
    }
    return;
}

IVP_Halfspacesoup::~IVP_Halfspacesoup(){
    for (int i=this->len()-1; i>=0; i--) {
	IVP_U_Hesse *old_plane = this->element_at(i);
	P_DELETE(old_plane);
    }
    this->clear();
}

IVP_Halfspacesoup::IVP_Halfspacesoup(){

}

IVP_Halfspacesoup::IVP_Halfspacesoup( const IVP_Compact_Ledge *ledge ){
    const IVP_Compact_Triangle *tri;
    tri = ledge->get_first_triangle();
    for (int i = 0; i< ledge->get_n_triangles(); tri = tri->get_next_tri(), i++){
	IVP_U_Hesse hesse;
	const IVP_Compact_Edge *edge = tri->get_first_edge();
	IVP_CLS.calc_hesse_vec_object_not_normized( edge , ledge, &hesse );
	hesse.hesse_val = 0.0f;

	hesse.normize();
	hesse.mult(-1);
	IVP_U_Point a_point;
	a_point.set( edge->get_start_point(ledge) );
	hesse.calc_hesse_val( &a_point );
	
        add_halfspace(&hesse);
    }
}

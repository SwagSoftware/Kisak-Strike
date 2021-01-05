// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

/**** INCLUDES *****/
#include <ivp_physics.hxx>

#include <ivu_float.hxx>
#include <ivu_geometry.hxx>


void IVP_U_Straight::calc_orthogonal_vec_from_point(const IVP_U_Point *point,
						IVP_U_Point *vec_out) const
{
    // vorr: normiert!
    vec_out->subtract(point, &this->start_point);

    IVP_DOUBLE sp = this->vec.dot_product(vec_out);
    vec_out->add_multiple(&this->vec, -sp);
    vec_out->mult(-1.0f);
}


IVP_DOUBLE IVP_U_Straight::get_quad_dist_to_point(IVP_U_Point *point) const
{
    IVP_U_Point w;
    w.subtract(point, &this->start_point);

    // already normized!!
    IVP_U_Point cross;
    cross.calc_cross_product(&this->vec, &w);
    return cross.quad_length();
}


IVP_U_Plain::IVP_U_Plain(const IVP_U_Hesse *i_hesse)
{
    // copy hesse values
    this->set(i_hesse);
    this->hesse_val = i_hesse->hesse_val;

    // calc a point on the ebene
    IVP_U_Point hp;
    hp.set_to_zero();	// any point
    this->proj_on_plane(&hp, &this->start_point);

    // calc spanning vectors
    vec1.calc_an_orthogonal(this);

    vec2.calc_cross_product(&vec1, this);
}

IVP_U_Plain::IVP_U_Plain(const IVP_U_Point *p0,const IVP_U_Point *p1,const IVP_U_Point *p2)
{
    // calc hesse
    this->calc_hesse(p0, p1, p2);

    start_point = *p0;
    vec1.subtract(p1, p0);
    vec2.subtract(p2, p0);
}

IVP_U_Straight::IVP_U_Straight(const IVP_U_Point *i_start_point, const IVP_U_Point *i_vec)
{
    start_point = *i_start_point;
    vec = *i_vec;
    vec.normize();
}

void IVP_U_Straight::set(const IVP_U_Point *i_start_point, const IVP_U_Point *i_vec)
{
    start_point = *i_start_point;
    vec = *i_vec;
    vec.normize();
}

void IVP_U_Straight::set(const IVP_U_Float_Point *i_start_point, const IVP_U_Float_Point *i_vec)
{
    start_point = *i_start_point;
    vec = *i_vec;
    vec.normize();
}

IVP_RETURN_TYPE IVP_U_Plain::calc_intersect_with(const IVP_U_Hesse *plane2,
				                     IVP_U_Straight *straight_out) const
{
    // if exists: intersection line between two planes
    // ATT: check return flag!
    
    if(this->is_parallel(plane2, P_EPS_PARALLEL)){
	return IVP_FAULT;
    }
    
    // calc direction of straight
    straight_out->vec.calc_cross_product(this, plane2);
    straight_out->vec.normize();
    
    /** calc startpoint of straight **/
    IVP_U_Point hp;
    plane2->proj_on_plane(&start_point, &hp);

    IVP_U_Point hp2;
    this->proj_on_plane(&hp, &hp2);

    // now we've got all for a intersecting straight
    IVP_U_Straight straight;
    straight.start_point = this->start_point;
    straight.vec.subtract(&hp2, &start_point);
    straight.vec.normize();

    // calc intersection of straight with plane2
    if(plane2->calc_intersect_with(&straight, &straight_out->start_point) == IVP_FAULT){
	printf("Merkwuerden.\n");
	CORE;
    }
    return IVP_OK;
}

IVP_RETURN_TYPE IVP_U_Hesse::calc_intersect_with(const IVP_U_Straight *straight,
				                     IVP_U_Point *point_out) const
{
    // intersection point of line/plane (if it exists)
    // ATT: check return flag!
    
    IVP_DOUBLE dist1 = this->get_dist(&straight->start_point);
    
    IVP_U_Point hp = straight->start_point;
    hp.add(&straight->vec);
    IVP_DOUBLE dist2 = this->get_dist(&hp);
    
    IVP_DOUBLE delta_dist = dist2 - dist1;
    if(IVP_Inline_Math::fabsd(delta_dist) < P_DOUBLE_EPS){
	if(dist1 >= P_DOUBLE_EPS){
	    return IVP_FAULT;	// true parallel
	}else{
	    *point_out = straight->start_point;
	    point_out->set(&straight->start_point);
	    return IVP_OK;
	}
    }
    
    IVP_DOUBLE factor = dist1 / delta_dist;

    point_out->add_multiple(&straight->start_point, &straight->vec, -factor);
    return IVP_OK;
}

IVP_U_INTERSECT_TYPE IVP_U_Straight::calc_intersect_with(const IVP_U_Straight *straight2,
				     IVP_U_Point *p_out, IVP_DOUBLE *dist_out)
{
    // calcs common point of two straights
    // and returns minimal quad dist, even if no common point exists
    
    // ATT: check return flag!
    // 0: real intersection in 3D
    // 1: no intersection, but not parallel
    // 2: identic
    // 3: parallel
    
    IVP_U_Point norm, hp;
    norm.calc_cross_product(&this->vec, &straight2->vec);

    if(norm.quad_length() < P_DOUBLE_EPS*P_DOUBLE_EPS){
	this->calc_orthogonal_vec_from_point(&start_point, &hp);
        *dist_out = hp.quad_length();
	if(*dist_out < P_DOUBLE_EPS*P_DOUBLE_EPS){
	    return IVP_U_INTERSECT_IDENTIC;	// identic
	}else{
	    return IVP_U_INTERSECT_PARALLEL; // parallel
	}
    }

    // create plane
    IVP_U_Point e_p0 = straight2->start_point;
    IVP_U_Point e_p1 = e_p0;
    IVP_U_Point e_p2 = e_p0;
    e_p1.add(&straight2->vec);
    e_p2.add(&norm);
    
    IVP_U_Hesse hesse;
    hesse.calc_hesse(&e_p0, &e_p1, &e_p2);
#ifdef DEBUG
    IVP_RETURN_TYPE flag = 
#endif
	hesse.calc_intersect_with(this, p_out);
    IVP_ASSERT(flag == IVP_OK);

    straight2->calc_orthogonal_vec_from_point(p_out, &hp);
    *dist_out = hp.quad_length();
    if(*dist_out < P_DOUBLE_EPS*P_DOUBLE_EPS){
	return IVP_U_INTERSECT_OK; // heureka! real common point
    }else{
	return IVP_U_INTERSECT_NO_INTERSECTION; // have distance
    }
    CORE;

    //lwss add - Need to return something even if it coredumps (gcc -Werror=return-type)
    return IVP_U_INTERSECT_NO_INTERSECTION;
}








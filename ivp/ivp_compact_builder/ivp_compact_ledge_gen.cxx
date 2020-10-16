// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

/********************************************************************************
 *	File:	       	ivp_compact_ledge_gen.cxx	
 *	Description:	generation of compacted ledge representation
 *                      from intermediate (non-compact) representation.
 ********************************************************************************/


#include <ivp_physics.hxx>
#include <string.h>
#include <ivu_hash.hxx>
#include <ivp_object_polygon_tetra.hxx>
#include <ivp_i_point_vhash.hxx>

#include <ivp_cache_object.hxx>
#include <ivp_compact_ledge.hxx>
#include <ivp_compact_ledge_solver.hxx>

#include <ivp_compact_ledge_gen.hxx>

///////////////////////////////////////////////////////////////////////////////////////

IVP_Compact_Ledge_Generator::IVP_Compact_Ledge_Generator()
{
    P_MEM_CLEAR(this);
}

int IVP_Compact_Ledge_Generator::prepare_compact_ledge(IVP_U_Vector<IVP_Triangle>*triangles)
{
    this->orig_triangles = triangles; // for validate    
    n_triangles = triangles->len();

    IVP_ASSERT( n_triangles > 0 ) //lwss add
    // some intermediate data structs
    int hash_size=16;
    while(hash_size < (n_triangles * 3) / 2){
	hash_size = hash_size<<1; // 2^x
    }
    point_hash = new IVP_point_hash(hash_size);
    
    // generate compact triangles
    // generate compact triangles
    // generate compact triangles
    
    // index triangles, ATT: destroys hesse info
    {
	int i;
	for(i=0; i<n_triangles; i++){
	    IVP_Triangle *tri = triangles->element_at(i);
	    tri->index = i;

	    // build up point hash and compact point hash
	    int j;
	    IVP_Tri_Edge *e = &tri->three_edges[0];
	    for(j=0; j<3; e = e->next, j++){
		if(!point_hash->find_point(e->start_point)){
		    e->start_point->tmp.compact_index = point_cnt;
		    point_cnt++;
		    point_hash->add_point(e->start_point);

		    // make new compact point and add to vector
		    point_vec.add(e->start_point);
		}
	    }
	}
    }
    edge_cnt=0;
    edge_hash = new IVP_Hash(hash_size*4, sizeof(void *), (void *)-1); //lwss - this is correct
    {
	int i;
    // fill in triangles
    for(i=0; i<n_triangles; i++){
	IVP_Triangle *tri = triangles->element_at(i);

	IVP_Compact_Triangle *c_tri = new IVP_Compact_Triangle();
	// index
	c_tri->set_tri_index(tri->index);

	// pierce info
	if(tri->pierced_triangle){
	    c_tri->set_pierce_index(tri->pierced_triangle->index);
	}else{
	    printf("no valid pierce index!? Probably a backside triangle...\n");
	}

	// material info
	c_tri->set_material_index(0); // @@@ DUMMY

	// fill in edge topologie
	// fill in edge topologie
	{
	    int j;
	    IVP_Tri_Edge *edge = &tri->three_edges[0];
	    for( j=0; j<3; edge = edge->next , j++){
		IVP_Compact_Edge *c_edge = &c_tri->c_three_edges[j];
		// point index already set
		c_edge->set_start_point_index(edge->start_point->tmp.compact_index);

		IVP_ASSERT(edge_hash->find((char *)&edge) == (void *)-1);
		edge_hash->add((char *)&edge, (void *)(i * 4 + j + 1));	// for opposites
		edge_cnt++;
	    }
	}
	// add triangle to vec
	triangle_vec.add(c_tri);
    }
    }
    {
	int i;
	// Triangle info second pass: opposites
	IVP_ASSERT(n_triangles == triangle_vec.len());
	for(i=0; i<n_triangles; i++){
	    int j;
	    IVP_Triangle *tri = triangles->element_at(i);
	    IVP_Compact_Triangle *c_tri = triangle_vec.element_at(i);

	    IVP_Tri_Edge *e = &tri->three_edges[0];
	    for(j=0; j<3; e = e->next, j++){
		IVP_Compact_Edge *c_edge = &c_tri->c_three_edges[j];
		IVP_Tri_Edge *opp = e->opposite;
		//lwss - x64 fixes
		//int opp_index = (int)edge_hash->find((char *)&opp);
		intptr_t opp_index = (intptr_t)edge_hash->find((char *)&opp);
		IVP_ASSERT(opp_index>=0);
		int rel_index = opp_index - (i*4 + j + 1);
		//volatile int rel_index = opp_index - ( i*sizeof(IVP_Compact_Edge) + j + 1);
        //lwss end
        c_edge->set_opposite_index(rel_index);
	    }
	}
    }
    
    // return calculated size of all
    int size=0;
    size += sizeof(IVP_Compact_Ledge);
    size += sizeof(IVP_Compact_Triangle) * n_triangles;
    size += sizeof(IVP_Compact_Poly_Point) * point_cnt;

    return size;
}
	
void IVP_Compact_Ledge_Generator::generate_compact_ledge(uchar *mem)
{
    // mem should be 16 aligned
    // mem: - IVP_Compact_Ledge
    //      - Array of IVP_Compact_Triangle
    //      - Array of IVP_Compact_Poly_Point

    // some header info
    IVP_Compact_Ledge *c_ledge = (IVP_Compact_Ledge *)mem;
    this->compact_ledge =  c_ledge;
    c_ledge->c_ledge_init();
    c_ledge->n_triangles = n_triangles;
    IVP_ASSERT( n_triangles > 0 ); //lwss add
    mem += sizeof(IVP_Compact_Ledge);
	
    int i;

    // copy triangle info (including edges) 
    IVP_ASSERT(n_triangles == triangle_vec.len());
    for(i=0; i<n_triangles; i++){
	memcpy(mem, triangle_vec.element_at(i), sizeof(IVP_Compact_Triangle));
	mem += sizeof(IVP_Compact_Triangle);
    }

    // copy point info
    IVP_ASSERT(point_cnt == point_vec.len());
    c_ledge->set_offset_ledge_points( (int)(mem - (uchar *)c_ledge) );
    for(i=0; i<point_cnt; i++){
	IVP_Compact_Poly_Point *cpp = (IVP_Compact_Poly_Point *)mem;
	cpp->hesse_val = 0.0f;
	cpp->set( point_vec.element_at(i));
	mem += sizeof(IVP_Compact_Poly_Point);
    }

    // copy edge common info
    //c_ledge->c_common_offset = 0;
    // set sizeinfo
    c_ledge->set_is_compact( IVP_TRUE);
    c_ledge->set_size((int)(mem - (uchar *)c_ledge));
}

#ifdef DEBUG
IVP_RETURN_TYPE IVP_Compact_Ledge_Generator::validate()
{
    // checks wether the compact ledge has any similarity with the
    // original version ...

    IVP_ASSERT(compact_ledge->n_triangles == triangle_vec.len());


    // all triangles
    const IVP_Compact_Triangle *c_triangles = compact_ledge->get_first_triangle();
    int i;
    for(i=0; i<n_triangles; i++){
	IVP_Triangle *tri = this->orig_triangles->element_at(i);
	const IVP_Compact_Triangle *c_tri = &c_triangles[i];

	// check tri info
	IVP_ASSERT(c_tri->get_tri_index() == i);
	if(tri->pierced_triangle){
	    IVP_ASSERT(c_tri->get_pierce_index() == tri->pierced_triangle->index);
	}
	IVP_ASSERT(c_tri->get_material_index() == 0);
	
	// all tri edges
	
	IVP_Tri_Edge *edge = &tri->three_edges[0];

	int j;
	for(j=0; j<3; edge=edge->next,j++){
	    const IVP_Compact_Edge *c_edge = &c_tri->c_three_edges[j];
	    
	    intptr_t opp_index = (intptr_t)edge_hash->find((char *)&edge->opposite); //lwss x64 fix
	    IVP_ASSERT(opp_index>=0);
	    int rel_index = opp_index - (i*sizeof(IVP_Compact_Edge) + j + 1);
        IVP_ASSERT(rel_index == c_edge->get_opposite_index());

	    const IVP_Compact_Poly_Point *c_po = IVP_CLS.give_object_coords(c_edge->get_opposite(),compact_ledge);

	    //IVP_ASSERT(edge->opposite->start_point->quad_distance_to(c_po) < 1e-3f);

	    IVP_ASSERT(edge->opposite->opposite == edge);
	    IVP_ASSERT(c_edge->get_opposite()->get_opposite() == c_edge);

	    IVP_ASSERT(c_edge->get_compact_ledge() == compact_ledge);
	    const IVP_Compact_Poly_Point *c_point = IVP_CLS.give_object_coords(c_edge,compact_ledge);
	    IVP_ASSERT(edge->start_point->quad_distance_to(c_point) < 1e-3f);

	    c_po = IVP_CLS.give_object_coords(c_edge->get_next(),compact_ledge);

	    IVP_ASSERT(edge->next->start_point->quad_distance_to(c_po) < 1e-3f);
	    

	}
    }    
    return IVP_OK;
}
#endif


IVP_Compact_Ledge_Generator::~IVP_Compact_Ledge_Generator()
{
    int i;
    
    // restore hesse info
    for(i=0; i<n_triangles; i++){
	IVP_Triangle *tri = orig_triangles->element_at(i);
	tri->calc_hesse();
    }
    
    // delete point intermediates
    point_vec.clear();
    P_DELETE(point_hash);

    // edge intermediates
    P_DELETE(edge_hash);

    // delete triangle intermediates
    for(i=0; i<n_triangles; i++){
	delete triangle_vec.element_at(i);
    }
    triangle_vec.clear();
}

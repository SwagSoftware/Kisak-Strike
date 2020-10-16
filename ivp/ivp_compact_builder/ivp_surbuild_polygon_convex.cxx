// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <ivp_object_polygon_tetra.hxx>
#include <ivu_hash.hxx>

#ifndef WIN32
#	pragma implementation "ivp_surbuild_polygon_convex.hxx"
#endif
//#include <ivp_surman_polygon.hxx>
#include <ivp_surbuild_polygon_convex.hxx>

#include <ivp_cache_object.hxx>
#include <ivp_compact_ledge.hxx>
#include <ivp_compact_ledge_solver.hxx>
#include <ivp_compact_ledge_gen.hxx>
#include <ivp_surbuild_ledge_soup.hxx>
#include <ivp_templates_intern.hxx>
#include <ivp_i_point_vhash.hxx>


IVP_SurfaceBuilder_Polygon_Convex::IVP_SurfaceBuilder_Polygon_Convex(IVP_Template_Polygon *templ)
{
    this->poly_point_hash = new IVP_point_hash(8);
    this->tetras = new IVP_Object_Polygon_Tetra(templ);
    this->tetras->template_polygon = templ;
    this->c_ledge = NULL;
    this->init_surface_manager_polygon();
}


IVP_SurfaceBuilder_Polygon_Convex::IVP_SurfaceBuilder_Polygon_Convex(IVP_Template_Ledge_Polygon_Soup *templ_ledge) // more of a hack :)
{
    this->tetras = NULL;
    this->c_ledge = NULL;
    this->poly_point_hash = new IVP_point_hash(8);
    
    // creates compact ledge from template
    
    if(!templ_ledge) return;
    IVP_ASSERT(templ_ledge->ledge_is_open==IVP_FALSE);
    IVP_ASSERT(templ_ledge->n_templ_triangles==2);
    
    // fill it with triangle and material info
    IVP_Template_Triangle *templ_tri = &templ_ledge->templ_triangles_array[0];
    IVP_U_Vector<IVP_Triangle> tri_vec; // to be filled now
    IVP_U_Vector<IVP_Poly_Surface> sur_vec; // to be filled now
    for(int i=0; i<1 /*templ_ledge->n_templ_triangles*/; i++, templ_tri++){
	
	// treat triangle

	// generate the 3 IVP_Poly_Points
	IVP_Poly_Point *new_point[3];
	for(int j=0; j<3; j++){
	    
	    // set up poly point for comparison
	    IVP_Poly_Point new_point_static;
	    new_point_static.set(&templ_tri->tri_points[j]);
	    
	    // is it a new point ?
	    if(!(new_point[j] = poly_point_hash->find_point(&new_point_static))){
		// malloc point
		new_point[j] = new IVP_Poly_Point();
		new_point[j]->set(&new_point_static);
		poly_point_hash->add_point(new_point[j]);
	    }	    
	}

	// make IVP_DOUBLE triangle
	IVP_Triangle *double_tri = IVP_Object_Polygon_Tetra::generate_double_triangle(new_point[0], new_point[1], new_point[2]);
	
	// some further triangle info
	double_tri->pierced_triangle = double_tri->other_side;
	double_tri->other_side->pierced_triangle = double_tri;
	IVP_Poly_Surface *ivp_sur = new IVP_Poly_Surface();
	ivp_sur->tetras = 0; // @@@ attention

	double_tri->ivp_surface = ivp_sur;
	double_tri->other_side->ivp_surface = ivp_sur;


	// add tri to vec
	tri_vec.add(double_tri);
	tri_vec.add(double_tri->other_side); // @@@OG debug: zwei auf einen streich
	sur_vec.add(ivp_sur);
    }
	
    // build compact ledge from tri's
    uchar *mem;
    {
	// could be subroutine, but: must be compact at last!
	IVP_Compact_Ledge_Generator ledge_gen;
    
	int size = ledge_gen.prepare_compact_ledge(&tri_vec);
    
	//printf("Compact ledge size: '%d'\n", size);
	mem = (uchar *)ivp_malloc_aligned(size,16); // 16 should be enough, but ...
	memset(mem,0,size);
	ledge_gen.generate_compact_ledge(mem);
#ifdef DEBUG
	if(ledge_gen.validate()!=IVP_OK){
	    printf("Compact ledge generation fizzled :-(\n");
	}
#endif	
    }
    {
	int j;
	for (j = tri_vec.len()-1;j>=0;j--){
	    IVP_Triangle *t = tri_vec.element_at(j);
	    P_DELETE(t);
	}
	for (j = sur_vec.len()-1;j>=0;j--){
	    IVP_Poly_Surface *s = sur_vec.element_at(j);
	    P_DELETE(s);
	}

	sur_vec.clear();
	tri_vec.clear();
    }

    this->c_ledge = (IVP_Compact_Ledge *)mem;

    return;
}

IVP_SurfaceBuilder_Polygon_Convex::~IVP_SurfaceBuilder_Polygon_Convex()
{
    for (int i = poly_point_hash->len()-1; i >=0;i--){
	IVP_U_Point *point = poly_point_hash->element_at(i);
	if (point) delete point;
    }
    P_DELETE(poly_point_hash);
    P_DELETE(tetras);
}


void IVP_SurfaceBuilder_Polygon_Convex::init_surface_manager_polygon()
{
    
    tetras->points_to_edge_hash = new IVP_Hash(500, 2*sizeof(void *), (void *)0);
    IVP_ERROR_STRING error = this->tetras->make_triangles();
    P_DELETE(this->tetras->points_to_edge_hash);

//lwss add
#ifdef DEBUG
    fprintf(stderr, "num triangles: %d\n", tetras->triangles.len);
    IVP_ASSERT( this->tetras->triangles.len > 0 )
#endif
//lwss end
    if (error){
	this->tetras = NULL;
	this->c_ledge = NULL;
	return;
    }
    
    IVP_Triangle *tri;
    for (tri = this->tetras->triangles.first; tri; tri = tri->next){
	tri->calc_hesse();
    }
    this->tetras->insert_pierce_info();	// only for convex objects


    // build compact ledge
    uchar *mem;
    {
	IVP_U_Vector<IVP_Triangle> tri_vec;
	this->fill_list_with_all_triangles(&tri_vec);
	{
	    IVP_Compact_Ledge_Generator ledge_gen;
    
	    int size = ledge_gen.prepare_compact_ledge(&tri_vec);

	    //printf("Compact ledge size: '%d'\n", size);
	    mem = (uchar *)ivp_malloc_aligned(size,16); // @@@ 16 should be enough, but ...
	    ledge_gen.generate_compact_ledge(mem);
#ifdef DEBUG
	    if(ledge_gen.validate()!=IVP_OK){
		printf("Compact ledge generation fizzled :-(\n");
	    }
#endif	    
	}
    }
    
    this->c_ledge = (IVP_Compact_Ledge *)mem;
}


void IVP_SurfaceBuilder_Polygon_Convex::fill_list_with_all_triangles(IVP_U_Vector<IVP_Triangle>*tri_vec)
{
    IVP_Triangle *tri;
    for (tri = this->tetras->triangles.first; tri; tri=tri->next) {
	if(tri->flags.is_hidden) continue; // skip backside triangles
	tri_vec->add(tri);			
    }
}


IVP_Compact_Ledge *IVP_SurfaceBuilder_Polygon_Convex::convert_template_to_ledge(IVP_Template_Polygon *templat)
{
    if ( !templat ) return(NULL);
    
    IVP_SurfaceBuilder_Polygon_Convex surman(templat);
    IVP_Compact_Ledge *res = surman.get_and_remove_compact_ledge();

    return(res);
}


IVP_Compact_Ledge *IVP_SurfaceBuilder_Polygon_Convex::convert_templateledgepolygonsoup_to_ledge(IVP_Template_Ledge_Polygon_Soup *templat)
{
    if ( !templat ) return(NULL);
    
    IVP_SurfaceBuilder_Polygon_Convex surface_builder(templat);
    IVP_Compact_Ledge *compact_ledge = surface_builder.get_and_remove_compact_ledge();
    return(compact_ledge);
}








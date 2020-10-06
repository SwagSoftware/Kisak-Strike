// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <ivp_templates_intern.hxx>

IVP_Template_Polygon::IVP_Template_Polygon()
{
    this->n_points = this->n_surfaces = this->n_lines = 0;
    this->points = NULL;
    this->surfaces = NULL;
    this->lines = NULL;
}

IVP_Template_Polygon::IVP_Template_Polygon(int point_count,
					   int line_count,
					   int surface_count)
{
    this->n_points = point_count;
    this->n_lines = line_count;
    this->n_surfaces = surface_count;
    this->lines = new IVP_Template_Line[n_lines];
    this->points = new IVP_Template_Point[n_points];
    this->surfaces = new IVP_Template_Surface[n_surfaces];
    int i;
    for (i=0;i<this->n_surfaces;i++){
	this->surfaces[i].templ_poly = this;
    }
}

IVP_Template_Polygon::~IVP_Template_Polygon()
{
    P_DELETE_ARRAY(this->lines);
    P_DELETE_ARRAY(this->points);
    P_DELETE_ARRAY(this->surfaces);
}



IVP_Template_Surface::IVP_Template_Surface()
{
    P_MEM_CLEAR(this);
}

void IVP_Template_Surface::close_surface()
{
    P_FREE(this->lines);
    P_DELETE_ARRAY(this->revert_line);
}

void IVP_Template_Surface::calc_surface_normal_template(int a,int b,int c)
{
    IVP_U_Hesse my_hesse;
    my_hesse.calc_hesse( &templ_poly->points[a], &templ_poly->points[b], &templ_poly->points[c] );
    my_hesse.normize();
    this->normal.set( my_hesse.k[0], my_hesse.k[1], my_hesse.k[2] );    
}

void IVP_Template_Surface::init_surface(int line_count)
{
    this->n_lines = line_count;
    this->lines = (ushort *)p_calloc(sizeof(lines), line_count);
    this->revert_line = new char[line_count];
}

int IVP_Template_Surface::get_surface_index()
{
    return this - templ_poly->surfaces;
}


IVP_Template_Surface::~IVP_Template_Surface()
{
    this->close_surface();
}



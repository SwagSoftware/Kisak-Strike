// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#include <ivp_physics.hxx>
#include <ivu_hash.hxx>
#include <ivp_compact_surface.hxx>
#include <ivp_compact_ledge.hxx>
#include <ivp_compact_recursive.hxx>
#include <ivp_surbuild_pointsoup.hxx>

IVP_Compact_Recursive::IVP_Compact_Recursive() : ledges(128) {

}

IVP_Compact_Recursive::~IVP_Compact_Recursive(){

}


void IVP_Compact_Recursive::add_compact_ledge(const IVP_Compact_Ledge *ledge){
  ledges.add((IVP_Compact_Ledge *)ledge);
}

void IVP_Compact_Recursive::add_compact_ledge_treenode(const IVP_Compact_Ledgetree_Node *node){
  if (node->is_terminal()){
    add_compact_ledge(node->get_compact_ledge());
  }else{
    add_compact_ledge_treenode( node->left_son());
    add_compact_ledge_treenode( node->right_son());
  }
}

void IVP_Compact_Recursive::add_compact_surface( const IVP_Compact_Surface *surface){
  add_compact_ledge_treenode( surface->get_compact_ledge_tree_root());
}

void IVP_Compact_Recursive::build_convex_hull(){
  IVP_Hash point_hash( 1024, sizeof(IVP_U_Float_Point),0);
  IVP_U_Vector<IVP_U_Point> points;
  
  for (int k = ledges.len()-1; k>=0;k--){
    IVP_Compact_Ledge *ledge = ledges.element_at(k);
    IVP_U_Float_Point *point_array = ledge->get_point_array();
    IVP_Compact_Triangle *tri = ledge->get_first_triangle();
    for (int t = ledge->get_n_triangles()-1; t>=0; t--){
      for (int e = 0; e<3;e++){
	const IVP_Compact_Edge *edge = tri->get_edge(e);
	const IVP_U_Float_Point *p = &point_array[edge->get_start_point_index()];
	if ( point_hash.find( (char *)p )) continue;
	point_hash.add((char *)p,(void *)p);
	points.add( new IVP_U_Point(p));
      }
      tri = tri->get_next_tri();
    }
  }

  hull = IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_ledge(&points);

  for (int x = points.len()-1; x>=0;x--){
    delete points.element_at(x);
  }
}

void IVP_Compact_Recursive::set_rekursive_convex_hull(){
  IVP_Hash point_hash( 1024, sizeof(IVP_U_Float_Point),(void *)-1);
  int n_points_in_hash = 0;
  
  struct Triangle_Key {
    int point_index[3];
    void set_tri(int a, int b, int c){
      if (b<a){ int x = a;a=b;b=a;a=x; };
      if (c<a){ int x = a;a=c;c=a;a=x; };
      if (b<a){ int x = a;a=b;b=a;a=x; };
      // now a < b < c
      point_index[0] = a; point_index[1] = b; point_index[2] = c;
    }
  } triangle_key;

  struct Edge_Key {
    int point_index[2];
    void set_edge(int a, int b){
      if (b<a){ int x = a;a=b;b=a;a=x; };
      // now a < b < c
      point_index[0] = a; point_index[1] = b;
    }
  } edge_key;

  
  IVP_Hash triangle_hash(1024, sizeof(Triangle_Key),0);
  IVP_Hash edge_hash(    1024, sizeof(Edge_Key),0);
    
  // set all hashes
  for (int ledge_i = ledges.len()-1; ledge_i>=0; ledge_i--){
    IVP_Compact_Ledge *ledge = ledges.element_at(ledge_i);
  
    IVP_Compact_Triangle *tri = ledge->get_first_triangle();

    for (int t = ledge->get_n_triangles()-1; t>=0; tri = tri->get_next_tri(),t--){
      const IVP_U_Float_Point *p[3];
      p[0] = tri->get_edge(0)->get_start_point( ledge );
      p[1] = tri->get_edge(1)->get_start_point( ledge );
      p[2] = tri->get_edge(2)->get_start_point( ledge );
      int pi[3];
      // convert points to unique point nums
      for (int i = 0; i<3;i++){
    //lwss - x64 fixes
	//int ind = (int)point_hash.find( (char *)p[i]);
	intptr_t ind = (intptr_t)point_hash.find( (char *)p[i]);
	//lwss end
	if (ind<0){
	  point_hash.add( (char *)p[i], (void *)n_points_in_hash);
	  ind = n_points_in_hash++;
	}
	pi[i] = ind;
      }
      // add triangle to hash
      triangle_key.set_tri(pi[0], pi[1], pi[2]);
      if (!triangle_hash.find( (char *)&triangle_key )){
	triangle_hash.add( (char *)&triangle_key, (void *)tri );
      }

      // add edges to hash
      for (int j = 0; j<3;j++){
	edge_key.set_edge( pi[j], pi[ (j+1)%3]);
	if (!edge_hash.find( (char *)&edge_key)){
	  edge_hash.add( (char *)&edge_key, tri);
	}
      }
    }
  }
  int edges_found = 0;
  int edges_not_found = 0;
  // set all flags
  {
    IVP_Compact_Ledge *ledge = hull;
    IVP_Compact_Triangle *tri = ledge->get_first_triangle();

    for (int t = ledge->get_n_triangles()-1; t>=0; tri = tri->get_next_tri(),t--){
      const IVP_U_Float_Point *p[3];
      p[0] = tri->get_edge(0)->get_start_point( ledge );
      p[1] = tri->get_edge(1)->get_start_point( ledge );
      p[2] = tri->get_edge(2)->get_start_point( ledge );
      int pi[3];
      // convert points to unique point nums
      for (int i = 0; i<3;i++){
    //lwss -x64 fixes
	//int ind = (int)point_hash.find( (char *)p[i]);
	intptr_t ind = (intptr_t)point_hash.find( (char *)p[i]);
	//lwss end
	pi[i] = ind;
      }
      // find triangle in hash
      triangle_key.set_tri(pi[0], pi[1], pi[2]);
      if (!triangle_hash.find( (char *)&triangle_key )){
	tri->set_is_virtual(1);
      }

      // find edges in hash
      for (int j = 0; j<3;j++){
	edge_key.set_edge( pi[j], pi[ (j+1)%3]);
	if (!edge_hash.find( (char *)&edge_key)){
	  ((IVP_Compact_Edge *)tri->get_edge(j))->set_is_virtual(1);
	  edges_not_found ++;
	}else{
	  edges_found ++;
	}
      }
    }
  }
  IVP_IF(1){
      //printf("extra convex hull rekursive: edges not found %i found %i\n", edges_not_found, edges_found);
  }
}

IVP_Compact_Ledge *IVP_Compact_Recursive::compile(){
  build_convex_hull();
  if (hull) set_rekursive_convex_hull();
  return hull;
}

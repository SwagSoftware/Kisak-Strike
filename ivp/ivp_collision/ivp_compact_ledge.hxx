// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

/********************************************************************************
 *	File:	       	ivp_compact_ledge.hxx	
 *	Description:	compacted ledge representation
 *                      contains: Triangle and Tri_Edge infos
 ********************************************************************************/

#ifndef WIN32
#	pragma interface
#endif

#ifndef _IVP_U_TYPES_INCLUDED
#	include "ivu_types.hxx"
#endif

#ifndef _IVP_VECTOR_INCLUDED
#	include "ivu_vector.hxx"
#endif

#include <cstdint> //lwss - x64 fixes. Hard to do without uintptr_t

class IVP_Compact_Triangle;

class IVP_Compact_Surface;
class IVP_Compact_Ledge;
class IVP_Poly_Point;
class IVP_Compact_Ledgetree_Node;

// aligned point for storing data in the compact ledge,
// based on hesse to enforce alignment of data
class IVP_Compact_Poly_Point : public IVP_U_Float_Hesse {
public:
    IVP_Compact_Poly_Point(IVP_U_Point *ipoint){
	this->set(ipoint);
    }
    void set_client_data( void * cl ){
	((void **)&this->hesse_val)[0] = cl;
    }
    void *get_client_data ( ) const {
	return ((void **)&this->hesse_val)[0];
    }
};

#define IVP_MAX_TRIANGLES_PER_LEDGE 8192

class IVP_Compact_Edge	{
private:
  // for edge navigation
  static int next_table[];
  static int prev_table[];

  // contents
  unsigned int start_point_index:16;		// point index
  signed   int opposite_index:15;		// rel to this // maybe extra array, 3 bits more than tri_index/pierce_index
  unsigned int   is_virtual:1;
public:
    // safety and interface methods (mainly internal usage)
  inline void set_start_point_index(int val) {IVP_ASSERT(val>=0 && val< (1<<16)); start_point_index=val; }
  inline void set_opposite_index(int val) {IVP_ASSERT(val>=-(1<<14)+1 && val< (1<<14)-1); opposite_index=val; }
  inline void set_is_virtual(unsigned int val){ is_virtual = val; }; 

    // real public 
public:
  inline int get_start_point_index() const { return start_point_index; };
  inline int get_opposite_index() const { return opposite_index; };
  inline int get_is_virtual() const { return is_virtual; }

    // read content
    //inline const IVP_Compact_Poly_Point *get_start_point() const; // SLOW!	see IVP_Compact_Ledge_Solver
    inline const IVP_Compact_Poly_Point *get_start_point(const IVP_Compact_Ledge *c_ledge) const;
    inline const IVP_Compact_Triangle   *get_triangle() const;
    inline int get_edge_index() const;
    
    // navigation
    inline const IVP_Compact_Edge *get_opposite() const;
    inline       IVP_Compact_Edge *get_opposite();
    inline const IVP_Compact_Edge *get_next() const;
    inline       IVP_Compact_Edge *get_next();
    inline const IVP_Compact_Edge *get_prev() const;
    inline       IVP_Compact_Edge *get_prev();
    inline const IVP_Compact_Ledge *get_compact_ledge() const;
    
    // measures
    IVP_Compact_Edge();

	void byte_swap(); // just byte swap this data

};

class IVP_Compact_Triangle	//
{
private:
  // triangle specific info
  unsigned int tri_index:12; // used for upward navigation
  unsigned int pierce_index:12;
  unsigned int material_index:7;
  unsigned int is_virtual:1;
public:
    // three edges
    IVP_Compact_Edge c_three_edges[3];

  // safety functions for creation
  inline void set_tri_index(int val) {IVP_ASSERT(val>=0 && val< (1<<12)); tri_index=val; }
  inline void set_pierce_index(int val) {IVP_ASSERT(val>=0 && val< (1<<12)); pierce_index=val; }
  inline void set_material_index(int val) {IVP_ASSERT(val>=0 && val< (1<<7)); material_index=val; }
  inline void set_is_virtual(unsigned int val){ is_virtual = val; }; 

  inline int get_tri_index() const { return tri_index; }
  inline int get_pierce_index() const { return pierce_index; }
  inline int get_material_index() const { return material_index; }
  inline int get_is_virtual() const { return is_virtual; }

    inline const IVP_Compact_Edge *get_first_edge() const { return &c_three_edges[0]; };
    inline       IVP_Compact_Edge *get_first_edge()       { return &c_three_edges[0]; };

    inline const IVP_Compact_Edge *get_edge(int index) const { return &c_three_edges[index]; };
    inline const IVP_Compact_Ledge *get_compact_ledge() const;
    inline const IVP_Compact_Triangle *get_next_tri() const { return this+1; };
    inline       IVP_Compact_Triangle *get_next_tri()       { return this+1; };
    
    IVP_Compact_Triangle();

	void byte_swap(); // just byte swap this data ( and the owned edge data )
};

#define IVP_COMPACT_BOUNDINGBOX_STEP_SIZE (1.0f / 250.0f )

class IVP_Compact_Ledge {
    // ATTENTION: some functions depend on EXACTLY THIS SIZE AND SHAPE of the structure
    // e.g. end of class must be 16 aligned for triangle address trick.
    // and assert(sizeof(IVP_Compact_Ledge) == 16 == sizeof(IVP_Compact_Triangle))
    friend class IVP_Compact_Ledge_Generator;
    friend class IVP_SurfaceBuilder_Ledge_Soup;
    friend class IVP_SurfaceBuilder_Mopp;
    friend class IVP_GridBuilder_Array;
//private: //lwss - make this public to fix errors
public:
    int c_point_offset; // byte offset from 'this' to (ledge) point array
    union {
	int ledgetree_node_offset;
	int client_data;	// if indicates a non terminal ledge
    };
    unsigned int has_chilren_flag:2;
    IVP_BOOL is_compact_flag:2;  // if false than compact ledge uses points outside this piece of memory
    unsigned int dummy:4;
    unsigned int size_div_16:24; 
    short n_triangles;
    short for_future_use;

	inline void set_offset_ledge_points(int offset) { 
		IVP_ASSERT( (offset & 15) == 0 );
		c_point_offset=offset; 
	};

    inline void set_size(int size){ IVP_ASSERT( (size > 0) && (size & 0xf) == 0); size_div_16 = size>>4; };
    inline void set_is_compact(IVP_BOOL x){ is_compact_flag = x; };
public:

    void c_ledge_init();	// memclear(this)

    inline const IVP_Compact_Poly_Point *get_point_array() const { return (IVP_Compact_Poly_Point *)(      ((char *)this) + c_point_offset);   };
    inline       IVP_Compact_Poly_Point *get_point_array()       { return (IVP_Compact_Poly_Point *)(      ((char *)this) + c_point_offset);   };

    // triangles are always placed behind the class instance
    inline const IVP_Compact_Triangle *get_first_triangle() const { return (IVP_Compact_Triangle *)(this+1); };
    inline       IVP_Compact_Triangle *get_first_triangle()       { return (IVP_Compact_Triangle *)(this+1); };
    inline IVP_BOOL is_terminal() const { return (IVP_BOOL)(has_chilren_flag == 0); };

    // get corresponding ledge tree node for recursive compace ledges only ( no grids )
    inline const IVP_Compact_Ledgetree_Node *get_ledgetree_node() const { IVP_ASSERT( !is_terminal() ); return ( IVP_Compact_Ledgetree_Node *)( ((char *)this) + ledgetree_node_offset); };
    
    inline int get_n_triangles() const { return n_triangles; };
    
#if defined(LINUX) || defined(SUN) || (defined(__MWERKS__) && defined(__POWERPC__)) || defined(GEKKO)
    inline int get_n_points() const { return  size_div_16 - n_triangles - 1; };
#endif    

    inline int get_size() const { return size_div_16 * 16; };
    inline IVP_BOOL is_compact(){ return (IVP_BOOL)is_compact_flag; }; // returns true if vertex info is included in compact ledge
    inline int get_client_data() const { IVP_ASSERT( is_terminal()); return client_data; };   // see IVP_Surface_Manager_Polygon for user acces
    inline void set_client_data(unsigned int x){ IVP_ASSERT(is_terminal()); if (is_terminal()) client_data = x; };

	void byte_swap(); // just byte swap this data BUT WILL NOT do the point array, as may be external data or shared and so may be byte swapped > 1 times
	void byte_swap_all(IVP_U_BigVector<IVP_Compact_Poly_Point>* pre_swapped_points); // byte_swap, and recurse too all related child data including the point and triangle data
};


/********************************************************************************
 *	Names:	       	IVP_Compact_Ledgetree_Node
 *	Description:	node on a hierarchy tree
 *			if this->offset < 0 -> this is terminal
 *			else:		left branch of subtree: successor node
 *					right branch of node: this + offset
 ********************************************************************************/
#define IVP_CLT_N_DIRECTIONS 3 // valid values: 3 = AABB

class IVP_Compact_Ledgetree_Node {
public:
    
    // ---------
    // Variables
    // ---------
    int offset_right_node; // (if != 0 than children
    int offset_compact_ledge; //(if != 0, pointer to hull that contains all subelements
    IVP_U_Float_Point3 center;	// in object_coords
    IVP_FLOAT radius; // size of sphere
    uchar box_sizes[IVP_CLT_N_DIRECTIONS];
    uchar free_0;
    // -------
    // Methods
    // -------
    const IVP_Compact_Ledge *get_compact_ledge() const {
	IVP_ASSERT(offset_right_node == 0);
	char *base = (char *)this;
	base += this->offset_compact_ledge;
	return (const IVP_Compact_Ledge *)base;
    }

    const IVP_Compact_Ledgetree_Node* left_son() const {
	IVP_ASSERT(offset_right_node);
	return this+1;
    }
    
    const IVP_Compact_Ledgetree_Node* right_son() const {
	IVP_ASSERT(offset_right_node);
	return (IVP_Compact_Ledgetree_Node *)(((char *)this) + this->offset_right_node);
    }

    IVP_BOOL is_terminal() const {
	if (!offset_right_node) return IVP_TRUE;
	else return IVP_FALSE;
    };
    
    const IVP_Compact_Ledge *get_compact_hull() const {
	if (this->offset_compact_ledge) {
	    return (IVP_Compact_Ledge *)(((char *)this) + this->offset_compact_ledge);
	}else{
	    return NULL;
	}
    };

	void byte_swap(); // just byte swap this data
	void byte_swap_all(IVP_U_BigVector<IVP_Compact_Poly_Point>* pre_swapped_points); // byte_swap, and recurse too all related child data

};




////////////////////////////////////////////////////////////////////////////////////
const IVP_Compact_Triangle *IVP_Compact_Edge::get_triangle() const
{
    // mask 4 lowest adress bits to receive triangle
    //lwss - x64 fixes ( original is x86 )
#if defined(__i386__)
    return (IVP_Compact_Triangle *)(((unsigned int)this) & 0xfffffff0);
#elif defined(__x86_64__) || defined(__e2k__)
    return (IVP_Compact_Triangle *)(((unsigned long int)this) & 0xFFFFFFFFFFFFFFF0);
#else
#error fix this for your platform
#endif
    //lwss end
}


const IVP_Compact_Ledge *IVP_Compact_Edge::get_compact_ledge() const
{
    const IVP_Compact_Triangle *c_tri = this->get_triangle();
    c_tri -= c_tri->get_tri_index(); // first triangle
    return (IVP_Compact_Ledge *)(((char *)c_tri) - sizeof(IVP_Compact_Ledge));
}


const IVP_Compact_Poly_Point *IVP_Compact_Edge::get_start_point(const IVP_Compact_Ledge *c_ledge) const
{
    IVP_ASSERT(c_ledge == this->get_compact_ledge());
    return &c_ledge->get_point_array()[this->get_start_point_index()];
}


// lwss - x64 fixes
const IVP_Compact_Edge *IVP_Compact_Edge::get_next() const
{
    int idx = (int)(((uintptr_t)this) & 0x0c);
    return (IVP_Compact_Edge *)(((char *)this) + ((int *)(((char *)this->next_table)+idx))[0]);
}

int IVP_Compact_Edge::get_edge_index()const{
  int idx = (int((((uintptr_t)this) & 0x0c))>>2) - 1;
    return idx;
}

IVP_Compact_Edge *IVP_Compact_Edge::get_next()
{
    int idx = (int)(((uintptr_t)this) & 0x0c);
    return (IVP_Compact_Edge *)(((char *)this) + ((int *)(((char *)this->next_table)+idx))[0]);
}

const IVP_Compact_Edge *IVP_Compact_Edge::get_prev() const
{
    int idx = (int)(((uintptr_t)this) & 0x0c);
    return (IVP_Compact_Edge *)(((char *)this) + ((int *)(((char *)this->prev_table)+idx))[0]);
}

IVP_Compact_Edge *IVP_Compact_Edge::get_prev() 
{
    int idx = (int)(((uintptr_t)this) & 0x0c);
    return (IVP_Compact_Edge *)(((char *)this) + ((int *)(((char *)this->prev_table)+idx))[0]);
}
//lwss end
const IVP_Compact_Edge *IVP_Compact_Edge::get_opposite() const
{
    int idx = opposite_index; // index is relative!
    return (IVP_Compact_Edge *)(this + idx);
}

IVP_Compact_Edge *IVP_Compact_Edge::get_opposite()
{
    int idx = get_opposite_index(); // index is relative!
    return (IVP_Compact_Edge *)(this + idx);
    //return (IVP_Compact_Edge *)((uintptr_t)this + (sizeof(IVP_Compact_Edge) * idx)); //lwss - 100% equal to this statement
}


const IVP_Compact_Ledge *IVP_Compact_Triangle::get_compact_ledge() const
{
  const IVP_Compact_Triangle *c_tri = this;
    c_tri -= c_tri->get_tri_index(); // first triangle
    return (IVP_Compact_Ledge *)(((char *)c_tri) - sizeof(IVP_Compact_Ledge));
}






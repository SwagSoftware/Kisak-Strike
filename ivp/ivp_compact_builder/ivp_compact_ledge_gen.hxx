// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

/********************************************************************************
 *	Name:	       	IVP_Compact_Ledge_Generator	
 *	Description:	used for generation of compacted ledge representation
 *                      out of ledge triangles
 ********************************************************************************/
class IVP_Triangle;
class IVP_Compact_Triangle;
class IVP_Compact_Poly_Point;

class IVP_Compact_Ledge_Generator
{
    // all for intermediate usage
    IVP_point_hash *point_hash;

    IVP_U_Vector<IVP_U_Point> point_vec;
    IVP_U_Vector<IVP_Compact_Triangle> triangle_vec;
    int point_cnt;
    int edge_cnt;
    int n_triangles;
    IVP_Hash *edge_hash;
    IVP_Hash *common_hash;

    IVP_Compact_Ledge *compact_ledge; // for validate
    IVP_U_Vector<IVP_Triangle> *orig_triangles; // for validate
public:
    int prepare_compact_ledge(IVP_U_Vector<IVP_Triangle>*tri_vec); // builds up intermediates and returns calculated size
    void generate_compact_ledge(uchar *mem_to_fill); // fills intermeds into mem
#ifdef DEBUG
    IVP_RETURN_TYPE validate(); // for debug purposes
#endif
    IVP_Compact_Ledge_Generator();
    ~IVP_Compact_Ledge_Generator();
};



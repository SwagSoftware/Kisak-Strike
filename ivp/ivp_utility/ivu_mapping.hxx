#ifndef _IVP_U_MAPPING_INCLUDED
#define _IVP_U_MAPPING_INCLUDED

enum IVP_MATRIX_CATEGORY {
    IVP_MC_UNDEFINED = 0,
    IVP_MC_RIGHT_HANDED = 1,
    IVP_MC_LEFT_HANDED = 2
};

class IVP_U_Mapping { // map_as_f_bs
public:
    char k[3];
    IVP_U_Mapping() {k[0] = IVP_INDEX_X; k[1] = IVP_INDEX_Y; k[2] = IVP_INDEX_Z;}
    IVP_COORDINATE_INDEX operator[](IVP_COORDINATE_INDEX which) {return IVP_COORDINATE_INDEX(k[which]);}
    void vapply(IVP_U_Point *point_as_f_cs, IVP_U_Point *point_bs_f_cs);
    void viapply(IVP_U_Point *point_bs_f_cs, IVP_U_Point *point_as_f_cs);
    //void vi2apply(IVP_U_Point *point_cs_f_as, IVP_U_Point *point_bs_f_cs);
    void mapply(IVP_U_Matrix3 *m_as_f_cs, IVP_U_Matrix3 *m_bs_f_cs);
    void miapply(IVP_U_Matrix3 *m_bs_f_cs, IVP_U_Matrix3 *m_as_f_cs);
    void mi2apply(IVP_U_Matrix3 *m_cs_f_as, IVP_U_Matrix3 *m_bs_f_cs);
    IVP_MATRIX_CATEGORY mapping_type();
};

#endif

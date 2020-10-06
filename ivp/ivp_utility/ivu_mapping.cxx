#include <ivp_physics.hxx>
#include <ivu_mapping.hxx>

void IVP_U_Mapping::vapply(IVP_U_Point *point_as_f_cs, IVP_U_Point *point_bs_f_cs) {
    point_bs_f_cs->k[IVP_INDEX_X] = point_as_f_cs->k[k[IVP_INDEX_X]];
    point_bs_f_cs->k[IVP_INDEX_Y] = point_as_f_cs->k[k[IVP_INDEX_Y]];
    point_bs_f_cs->k[IVP_INDEX_Z] = point_as_f_cs->k[k[IVP_INDEX_Z]];
}

void IVP_U_Mapping::viapply(IVP_U_Point *point_bs_f_cs, IVP_U_Point *point_as_f_cs) {
    point_as_f_cs->k[k[IVP_INDEX_X]] = point_bs_f_cs->k[IVP_INDEX_X];
    point_as_f_cs->k[k[IVP_INDEX_Y]] = point_bs_f_cs->k[IVP_INDEX_Y];
    point_as_f_cs->k[k[IVP_INDEX_Z]] = point_bs_f_cs->k[IVP_INDEX_Z];
}

/*
void IVP_U_Mapping::vi2apply(IVP_U_Point *point_cs_f_as, IVP_U_Point *point_bs_f_cs) {
    
}
*/

void IVP_U_Mapping::mapply(IVP_U_Matrix3 *m_as_f_cs, IVP_U_Matrix3 *m_bs_f_cs) {
    IVP_U_Point p;
    m_as_f_cs->get_row(IVP_COORDINATE_INDEX(k[IVP_INDEX_X]), &p); m_bs_f_cs->set_row(IVP_INDEX_X,&p);
    m_as_f_cs->get_row(IVP_COORDINATE_INDEX(k[IVP_INDEX_Y]), &p); m_bs_f_cs->set_row(IVP_INDEX_Y,&p);
    m_as_f_cs->get_row(IVP_COORDINATE_INDEX(k[IVP_INDEX_Z]), &p); m_bs_f_cs->set_row(IVP_INDEX_Z,&p);
}

void IVP_U_Mapping::miapply(IVP_U_Matrix3 *m_bs_f_cs, IVP_U_Matrix3 *m_as_f_cs) {
    IVP_U_Point p;
    m_bs_f_cs->get_row(IVP_INDEX_X, &p); m_as_f_cs->set_row(IVP_COORDINATE_INDEX(k[IVP_INDEX_X]),&p);
    m_bs_f_cs->get_row(IVP_INDEX_Y, &p); m_as_f_cs->set_row(IVP_COORDINATE_INDEX(k[IVP_INDEX_Y]),&p);
    m_bs_f_cs->get_row(IVP_INDEX_Z, &p); m_as_f_cs->set_row(IVP_COORDINATE_INDEX(k[IVP_INDEX_Z]),&p);
}

void IVP_U_Mapping::mi2apply(IVP_U_Matrix3 *m_cs_f_as, IVP_U_Matrix3 *m_bs_f_cs) {
    IVP_U_Point p;
    m_cs_f_as->get_col(IVP_COORDINATE_INDEX(k[IVP_INDEX_X]), &p); m_bs_f_cs->set_col(IVP_INDEX_X,&p);
    m_cs_f_as->get_col(IVP_COORDINATE_INDEX(k[IVP_INDEX_Y]), &p); m_bs_f_cs->set_col(IVP_INDEX_Y,&p);
    m_cs_f_as->get_col(IVP_COORDINATE_INDEX(k[IVP_INDEX_Z]), &p); m_bs_f_cs->set_col(IVP_INDEX_Z,&p);
}

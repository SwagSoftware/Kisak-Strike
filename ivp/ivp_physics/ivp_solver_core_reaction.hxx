// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PROTECTED

#ifndef IVP_SOLVER_CORE_REACTION_INCLUDED
#define IVP_SOLVER_CORE_REACTION_INCLUDED

#ifndef WIN32
//#	pragma interface
#endif



class IVP_U_Point_4: public IVP_U_Float_Hesse {
public:
  IVP_DOUBLE dot_product4( const IVP_U_Point_4 &o){
    return k[0] * o.k[0] + k[1] * o.k[1] + k[2] * o.k[2] + hesse_val * o.hesse_val;
  }
  

  void set_line_wise_mult4( const IVP_U_Point_4 *a, const IVP_U_Point_4 *b){
    IVP_DOUBLE x = a->k[0] * b->k[0];
    IVP_DOUBLE y = a->k[1] * b->k[1];
    IVP_DOUBLE z = a->k[2] * b->k[2];
    IVP_DOUBLE w = a->hesse_val * b->hesse_val;
    this->k[0] = x;
    this->k[1] = y;
    this->k[2] = z;
    this->hesse_val = w;
  }
  
};

class IVP_Solver_Core_Reaction {
private:
  void init_trans_ws(const IVP_U_Point *pos_ws, IVP_Core *core, IVP_U_Point_4 cross_direction_position_cs[3],IVP_U_Point_4 cr_mult_inv[3], IVP_FLOAT sign);
  void init_rot_ws(IVP_Core *core, IVP_U_Point_4 cross_direction_position_cs[3],IVP_U_Point_4 cr_mult_inv[3], IVP_FLOAT sign);
  
public:
  IVP_U_Float_Point *direction_ws[3];
  IVP_U_Point_4 cross_direction_position_cs0[3];
  IVP_U_Point_4 cross_direction_position_cs1[3];
  const IVP_U_Matrix *m_world_f_core_last_psi[2];
  IVP_U_Point_4 cr_mult_inv0[3];
  IVP_U_Point_4 cr_mult_inv1[3];
  
    /********************************************************************************
     *	Name:	       	m_velocity_ds_f_impulse_ds
     *	Description:	The symmetric matrix which describes the reaction of
     *			the core based on given impulses
     *	Note:		only the upper diagonal is filled
     ********************************************************************************/
  IVP_U_Matrix3 m_velocity_ds_f_impulse_ds;  // 
  IVP_U_Float_Point delta_velocity_ds;   // based on current rot_speeds only speeds

  /* init with normized axis, if direction_X_ws == 0 than not used, if core == 0 than static */
  void init_reaction_solver_translation_ws(IVP_Core *core_0, IVP_Core *core_1, IVP_U_Point &pos_ws,
				IVP_U_Float_Point *direction_0_ws, // the directions define the d_space (_ds)
				IVP_U_Float_Point *direction_1_ws,
				IVP_U_Float_Point *direction_2_ws); 

  void init_reaction_solver_rotation_ws(IVP_Core *core_0, IVP_Core *core_1,
				IVP_U_Float_Point *direction_0_ws, // the directions define the r_space (_ds)
				IVP_U_Float_Point *direction_1_ws,
				IVP_U_Float_Point *direction_2_ws); // if <3, m_velocity_ds_f_impulse_ds is not filled completely
	// invertion
	IVP_RETURN_TYPE invert_3x3_matrix(); // real inverts the m_velocity_ds_f_impulse_ds matrix (result is full 3x3

    // get the results
    IVP_U_Matrix3 *get_m_velocity_ds_f_impulse_ds(){	return &m_velocity_ds_f_impulse_ds;    };

  
  void exert_impulse_dim1(IVP_Core *core_0, IVP_Core *core_1, IVP_U_Float_Point &impulse_ds); // 1 dimension only
  void exert_impulse_dim2(IVP_Core *core_0, IVP_Core *core_1, IVP_U_Float_Point &impulse_ds); // 2 dimensions
  void exert_impulse_dim3(IVP_Core *core_0, IVP_Core *core_1, IVP_U_Float_Point &impulse_ds); // 3 dimensions

  void exert_angular_impulse_dim1(IVP_Core *core_0, IVP_Core *core_1, IVP_U_Float_Point &impulse_rs); // 1 dimensions
  void exert_angular_impulse_dim2(IVP_Core *core_0, IVP_Core *core_1, IVP_U_Float_Point &impulse_rs); // 2 dimensions
  void exert_angular_impulse_dim3(IVP_Core *core_0, IVP_Core *core_1, IVP_U_Float_Point &impulse_rs); // 3 dimensions

  //IVP_Solver_Core_Reaction(){;}; // real constructor is init
};

#endif

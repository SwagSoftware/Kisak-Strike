// IVP_EXPORT_PROTECTED

class IVP_Compact_Ledge_Mass_Center_Solver {
    const IVP_U_Matrix *transform;
public:
    IVP_DOUBLE msum;
    IVP_DOUBLE mzsum;
    IVP_DOUBLE mzzsum;
    IVP_Compact_Ledge_Mass_Center_Solver(const IVP_U_Matrix *transform_in);

    void integrate_triangle( const IVP_Compact_Ledge *ledge, const IVP_Compact_Triangle *triangle,
			     int x,int y,int z);
};

class IVP_Compact_Ledge_Find_Mass_Center {
public:
    IVP_U_Float_Point sum_mass;
    IVP_DOUBLE qsum_surface;
    IVP_DOUBLE sum_det;
    void integrate_triangle( const IVP_Compact_Ledge *ledge, const IVP_Compact_Triangle *triangle);
    void integrate_ledge( const IVP_Compact_Ledge *ledge);
    void integrate_ledges( IVP_U_BigVector<IVP_Compact_Ledge> *v_ledges);

    IVP_U_Point get_mass_center(){
	IVP_U_Point res; res.set_multiple(&sum_mass, 1.0f / sum_det);
	return res;
    }
    IVP_Compact_Ledge_Find_Mass_Center();
};

class IVP_Rot_Inertia_Solver {
    int dummy;		// there are no zero length classes in some C++ compilers
public:
    
    /********************************************************************************
     *	Name:	       	Bounding box and radius calculation 
     ********************************************************************************/
public:
    
    static void find_center_given_xyz(IVP_U_BigVector<IVP_Compact_Ledge> *v_ledges, int x,int y,int z,
				      const IVP_U_Matrix *transform,
				      IVP_DOUBLE *center_out, IVP_DOUBLE *volume_out, IVP_DOUBLE *inertia_out);
    
    static void calc_bounding_box( const IVP_Compact_Ledge *c_ledge_in,
				       IVP_U_Point *min_extents_out, IVP_U_Point *max_extents_out);

    static  IVP_RETURN_TYPE calc_mass_center_and_rotation_inertia(    const class IVP_Compact_Surface *c_surface_in,
							      IVP_U_Point *mass_center_out,     IVP_U_Point *rotation_inertia_out);

#ifdef HAVOK_MOPP
    static  IVP_RETURN_TYPE calc_mass_center_and_rotation_inertia(    const class IVP_Compact_Mopp* c_mopp_in,
							      IVP_U_Point *mass_center_out,     IVP_U_Point *rotation_inertia_out);
#endif // HAVOK_MOPP
};


// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef WIN32
#	pragma interface
#endif

class IVP_Liquid_Surface_Descriptor {
public:
    //lwss add virtual destructor ( was leaking 40 bytes )
    virtual ~IVP_Liquid_Surface_Descriptor() {}
    //lwss end

    virtual void calc_liquid_surface( IVP_Environment *environment,
				      IVP_Core *core,
				      IVP_U_Float_Hesse *surface_normal_out,
				      IVP_U_Float_Point *abs_speed_of_current_out) = 0;

};


class IVP_Liquid_Surface_Descriptor_Simple : public IVP_Liquid_Surface_Descriptor {

public:
    IVP_U_Float_Hesse surface;
    IVP_U_Float_Point abs_speed_of_current;
    
public:
    void calc_liquid_surface( IVP_Environment *environment,
			      IVP_Core *core,
			      IVP_U_Float_Hesse *surface_normal_out,
			      IVP_U_Float_Point *abs_speed_of_current_out);

    IVP_Liquid_Surface_Descriptor_Simple(const IVP_U_Float_Hesse *surface_in, const IVP_U_Float_Point *abs_speed_of_current_in);
};


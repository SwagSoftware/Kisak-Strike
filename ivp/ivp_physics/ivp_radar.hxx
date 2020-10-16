// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


/********************************************************************************
 *	Name:	       	IVP_Radar_Hit	
 *	Description:	Information about a hit
 *	Attention:	positions are only an estimate
 ********************************************************************************/
class IVP_Radar_Hit {
public:
    IVP_Object *this_object;
    IVP_Object *other_object;
    IVP_DOUBLE dist;
};

/********************************************************************************
 *	Name:	      	IVP_Radar	
 *	Description:	base class, needed for IVP_Environment::do_radar_checking(...)
 *	Note:		a callback class, which has to be subclassed by the user
 *			max_range and max_relative_error have to be set by the user
 *			before calling IVP_Environment::do_radar_checking
 *	Attention:	Only objects returned, which will also cause a collision
 *			with the start object
 ********************************************************************************/
class IVP_Radar {
public:
    //////////// minimal settings for radar checking
    IVP_DOUBLE max_range;		// max distance to check
    IVP_DOUBLE max_relative_error;	// maximum relative distance error for checking
    
    virtual void radar_hit( IVP_Radar_Hit *)=0;
};


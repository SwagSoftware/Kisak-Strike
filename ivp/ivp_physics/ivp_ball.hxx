// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_ball.hxx
 *  Description:    This file provides you with the class to implement a perfect
 *		    (yet simple) ball/sphere.
 *  Classes:	    IVP_Ball
 ********************************************************************************/

#ifndef _IVP_BALL_INCLUDED
#define _IVP_BALL_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

class IVP_Template_Ball;


 /********************************************************************************
 *  Class:	    IVP_Ball
 *  Description:    Base class for a non-polygonal (perfectly shaped) ball.
 *  Note:	    All balls share a global dummy surface manager and ledge.
 *  Note:	    To create a ball see "IVP_Environment::create_ball()".
 *  Version Info:   Not fully implemented (yet).
 *******************************************************************************/

class IVP_Ball : public IVP_Real_Object {
    friend class IVP_Environment;
    friend class IVP_Contact_Point;

protected:
    // internal methods
    IVP_Ball(IVP_Cluster *father, const IVP_Template_Ball *tball, const IVP_Template_Real_Object *real, const IVP_U_Quat *rotation, const IVP_U_Point *position);
    virtual ~IVP_Ball();

public:

    /******************************************************************************
     *  Method:		get_radius
     *  Description:    Use this method to retrieve the ball's radius.
     *	Output:		Radius of ball/sphere.
     *****************************************************************************/
    IVP_FLOAT get_radius() const {
	return( get_extra_radius() );
    };
};

#endif

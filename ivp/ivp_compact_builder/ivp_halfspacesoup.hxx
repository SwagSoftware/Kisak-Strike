// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

/********************************************************************************
 *  Filename:	    ivp_halfspacesoup.hxx
 *  Description:    This file provides you with a support class for easy handling
 *		    of halfspace planes.
 *  Classes:	    IVP_Halfspacesoup
 ********************************************************************************/

#ifndef _IVP_HALFSPACE_SOUP_INCLUDED
#define _IVP_HALFSPACE_SOUP_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

#define HALFSPACESOUP_TOLERANCE 0.01f * P_MIN_EDGE_LEN


/********************************************************************************
 *  Class:	    IVP_Halfspacesoup
 *  Description:    Support class. This class will maintain any set of halfspace
 *		    planes.
 *		    A halfspace plane is an infinite (virtual) plane that divides
 *		    space into two discreet parts.
 *  Note:	    The Ipion engine uses halfspace planes to describe discreet
 *		    volumes in space. For this it will expect the normal vectors
 *		    of the supplied planes to point AT the enclosed volume (and
 *		    not away from it)!
 *  Note:	    You should add new planes by using the supplied
 *		    "add_halfspace" method!
 *******************************************************************************/

class IVP_Halfspacesoup : public IVP_U_Vector<IVP_U_Hesse> {
public:

    /******************************************************************************
     *  Method:		add_halfspace
     *  Description:    This method allows to add a new plane to the already
     *			existing set of halfspace planes.
     *	Note:		This method will search the existing set for an almost
     *			parallel plane. If one exists, it will drop the one that
     *			is further away from the enclosed volume.
     *	Input:		<plane> a new halfspace plane
     *****************************************************************************/
    void add_halfspace(const IVP_U_Hesse *plane);

    /******************************************************************************
     *  Method:		constructor
     *  Description:    This constructor will immediately convert the supplied
     *			compact ledge into a set of halfspace planes.
     *	Input:		<ledge> compact ledge you want to have converted into
     *			        halfspace planes
     *****************************************************************************/
    IVP_Halfspacesoup(const IVP_Compact_Ledge *ledge);

    IVP_Halfspacesoup();
    ~IVP_Halfspacesoup();

};

#endif

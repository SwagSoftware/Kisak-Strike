/* initcb.f -- translated by f2c (version 19990311).
*/
#include <ivp_physics.hxx>


#include <geompack.hxx>


void IVP_Geompack::initcb_(double tolin) {

    /* Local variables */
    double epsp1, eps;


/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Initialize global variables in common blocks */
/*        GERROR, GCONST, and GPRINT. The latter is used for */
/*        printing debugging information. */

/*     Input parameters: */
/* 	 TOLIN - relative tolerance used to determine TOL */

/*     Output parameters in common blocks: */
/*        IERR - error code, initialized to 0 */
/*        PI - ACOS(-1.0D0) */
/*        TOL - relative tolerance MAX(TOLIN,100.0D0*EPS) where */
/*              EPS is approximation to machine epsilon */
/*        IPRT - standard output unit 6 */
/*        MSGLVL - message level, initialized to 0 */



    this->ierr = 0;
    eps = 1.0;
L10:
    eps /= 2.0;
    epsp1 = eps + 1.0;
    if (epsp1 > 1.0) {
	goto L10;
    }
    // Computing MAX
    this->tolerance = max(tolin, eps * 100.0);

    return;
}

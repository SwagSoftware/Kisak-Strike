class IVP_U_Matrix3;




#if 0
#ifndef __gluquat__
#define __gluquat__


#if defined (WIN32)
#	ifndef WIN32_LEAN_AND_MEAN
#		define	WIN32_LEAN_AND_MEAN
#	endif
#	include <windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>


// Quaternion Structure
// quaternion is represented as (w,[x,y,z])
// where: w       - scalar part
//        x, y, z - vector part
typedef struct tag_GL_QUAT { 
  GLIVP_FLOAT w, x, y, z;
} GL_QUAT;



// define APIENTRY and CALLBACK to null string if we aren't on Win32
#if !defined(WIN32)
#define APIENTRY
#define CALLBACK
#endif

#ifdef __cplusplus
extern "C" {
#endif


extern void APIENTRY gluQuatToMat_EXT(GL_QUAT *, GLIVP_FLOAT m[4][4]);		->set_matrix
inline extern void APIENTRY gluQuatToMat_EXT(GL_QUAT *, IVP_U_Matrix3 *matrix);
extern void APIENTRY gluQuatSlerp_EXT(GL_QUAT * , GL_QUAT * , GLfloat, GL_QUAT *);
extern void APIENTRY gluMatToQuat_EXT(GLIVP_FLOAT m[4][4], GL_QUAT *);
extern void APIENTRY gluMatToQuat_EXT(IVP_U_Matrix3 *mat3, GL_QUAT *); 		-> set_quat(P_Ma
inline extern void APIENTRY gluQuatMul_EXT(GL_QUAT*, GL_QUAT*, GL_QUAT*);
extern void APIENTRY gluQuatDiv_EXT(GL_QUAT*, GL_QUAT*, GL_QUAT*);

extern void APIENTRY gluQuatNormalize_EXT(GL_QUAT *);
extern void APIENTRY gluQuatInverse_EXT(GL_QUAT *);



extern void APIENTRY gluEulerToQuat_EXT(GLfloat, GLfloat, GLfloat, GL_QUAT * );
extern void APIENTRY gluQuatLerp_EXT(GL_QUAT *, GL_QUAT *, GLfloat, GL_QUAT *);
extern void APIENTRY gluQuatGetValue_EXT(GL_QUAT*, GLIVP_FLOAT*, GLIVP_FLOAT*, GLIVP_FLOAT*, GLIVP_FLOAT*);
extern void APIENTRY gluQuatSetValue_EXT(GL_QUAT *, GLfloat, GLfloat, GLfloat, GLIVP_FLOAT);
extern void APIENTRY gluQuatScaleAngle_EXT(GL_QUAT *, GLIVP_FLOAT);
extern void APIENTRY gluQuatSetFromAx_EXT(GLfloat, GLfloat, GLfloat, GLfloat, GLfloat, 
										  GLfloat, GL_QUAT *);
extern void APIENTRY gluQuatAdd_EXT(GL_QUAT*, GL_QUAT*, GL_QUAT*);
extern void APIENTRY gluQuatSub_EXT(GL_QUAT*, GL_QUAT*, GL_QUAT*);
extern void APIENTRY gluQuatCopy_EXT(GL_QUAT*, GL_QUAT*);
extern void APIENTRY gluQuatSquare_EXT(GL_QUAT*, GL_QUAT*);
extern void APIENTRY gluQuatSqrt_EXT(GL_QUAT*, GL_QUAT*);
extern GLIVP_FLOAT APIENTRY gluQuatDot_EXT(GL_QUAT*, GL_QUAT*);
extern GLIVP_FLOAT APIENTRY gluQuatLength_EXT(GL_QUAT*);
extern void APIENTRY gluQuatNegate_EXT(GL_QUAT*, GL_QUAT*);
extern void APIENTRY gluQuatExp_EXT(GL_QUAT*, GL_QUAT*);
extern void APIENTRY gluQuatLog_EXT(GL_QUAT*, GL_QUAT*);
extern void APIENTRY gluQuatLnDif_EXT(GL_QUAT*, GL_QUAT*, GL_QUAT*);







#ifdef __cplusplus
}

#endif


#endif  // __gluquat__

#endif









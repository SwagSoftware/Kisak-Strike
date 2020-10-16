// TKBMS v1.0 -----------------------------------------------------
//
// PLATFORM		: ALL
// PRODUCT		: FULL EXTENDED_SDK HARDCORE_SDK
// VISIBILITY	: INTERNAL
//
// ------------------------------------------------------TKBMS v1.0

#ifndef HK_MATH_VECMATH_H
#define HK_MATH_VECMATH_H

/* forwared declarations */

class hk_Vector3;
class hk_Vector4;
class hk_Quaternion;
class hk_Matrix3;
class hk_Rotation;
class hk_Transform;
class hk_QTransform;
class hk_Diagonal_Matrix;

/* class declarations */

#include <hk_base/base.h>

#include <hk_math/types.h>
#include <hk_math/math.h>

#include <hk_math/vector3/vector3.h>
#include <hk_math/vector4.h>
#include <hk_math/plane.h>
#include <hk_math/diagonal_matrix.h>
#include <hk_math/quaternion/quaternion.h>
#include <hk_math/matrix3.h>
#include <hk_math/rotation.h>
#include <hk_math/transform.h>
#include <hk_math/qtransform.h>
#include <hk_math/ray.h>
#include <hk_math/interval.h>

/* class inlines */

#include <hk_math/vector3/vector3.inl>
#include <hk_math/vector4.inl>
#include <hk_math/plane.inl>
#include <hk_math/quaternion/quaternion.inl>
#include <hk_math/matrix3.inl>
#include <hk_math/rotation.inl>
#include <hk_math/transform.inl>
#include <hk_math/qtransform.inl>

#endif /* HK_MATH_VECMATH_H */

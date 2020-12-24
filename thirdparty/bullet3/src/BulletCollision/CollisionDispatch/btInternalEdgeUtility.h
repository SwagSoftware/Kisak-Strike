
#ifndef BT_INTERNAL_EDGE_UTILITY_H
#define BT_INTERNAL_EDGE_UTILITY_H

#include "LinearMath/btHashMap.h"
#include "LinearMath/btVector3.h"

#include "BulletCollision/CollisionShapes/btTriangleInfoMap.h"

///The btInternalEdgeUtility helps to avoid or reduce artifacts due to wrong collision normals caused by internal edges.
///See also http://code.google.com/p/bullet/issues/detail?id=27

class btBvhTriangleMeshShape;
class btCollisionObject;
struct btCollisionObjectWrapper;
class btManifoldPoint;
class btIDebugDraw;
class btHeightfieldTerrainShape;

enum btInternalEdgeAdjustFlags
{
	BT_TRIANGLE_CONVEX_BACKFACE_MODE = 1,
	BT_TRIANGLE_CONCAVE_DOUBLE_SIDED = 2,  //double sided options are experimental, single sided is recommended
	BT_TRIANGLE_CONVEX_DOUBLE_SIDED = 4
};

///Call btGenerateInternalEdgeInfo to create triangle info, store in the shape 'userInfo'
void btGenerateInternalEdgeInfo(btBvhTriangleMeshShape* trimeshShape, btTriangleInfoMap* triangleInfoMap);

void btGenerateInternalEdgeInfo(btHeightfieldTerrainShape* trimeshShape, btTriangleInfoMap* triangleInfoMap);

///Call the btFixMeshNormal to adjust the collision normal, using the triangle info map (generated using btGenerateInternalEdgeInfo)
///If this info map is missing, or the triangle is not store in this map, nothing will be done
void btAdjustInternalEdgeContacts(btManifoldPoint& cp, const btCollisionObjectWrapper* trimeshColObj0Wrap, const btCollisionObjectWrapper* otherColObj1Wrap, int partId0, int index0, int normalAdjustFlags = 0, btIDebugDraw* drawer = NULL);

#endif  //BT_INTERNAL_EDGE_UTILITY_H
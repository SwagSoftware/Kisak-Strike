#include "StdAfx.h"

#include <vphysics/virtualmesh.h>
#include <cmodel.h>

#include "BulletCollision/CollisionDispatch/btInternalEdgeUtility.h"
#include "LinearMath/btConvexHull.h"
#include "LinearMath/btGeometryUtil.h"

#include "Physics_Collision.h"
#include "Physics_Object.h"
#include "convert.h"
#include "Physics_KeyParser.h"
#include "phydata.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

#define COLLISION_MARGIN 0.015 // 15 mm

// TODO: Use btConvexTriangleMeshShape instead of btConvexHullShape? btw we shouldn't use btConvexTriangleMeshShape, it's not performance friendly
// #define USE_CONVEX_TRIANGLES

// lol hack
extern IVPhysicsDebugOverlay *g_pDebugOverlay;

/****************************
* CLASS CCollisionQuery
****************************/

// FIXME: We don't use triangles to represent shapes internally!
// Low priority, not even used ingame
class CCollisionQuery : public ICollisionQuery {
	public:
		CCollisionQuery(CPhysCollide *pCollide) {m_pCollide = pCollide;}

		// number of convex pieces in the whole solid
		int					ConvexCount();
		// triangle count for this convex piece
		int					TriangleCount(int convexIndex);
		// get the stored game data
		uint				GetGameData(int convexIndex);
		// Gets the triangle's verts to an array
		void				GetTriangleVerts(int convexIndex, int triangleIndex, Vector *verts);
		void				SetTriangleVerts(int convexIndex, int triangleIndex, const Vector *verts);
		
		// returns the 7-bit material index
		int					GetTriangleMaterialIndex(int convexIndex, int triangleIndex);
		// sets a 7-bit material index for this triangle
		void				SetTriangleMaterialIndex(int convexIndex, int triangleIndex, int index7bits);

	private:
		CPhysCollide *	m_pCollide;
};

int CCollisionQuery::ConvexCount() {
	if (m_pCollide->IsCompound()) {
		btCompoundShape *pShape = m_pCollide->GetCompoundShape();
		return pShape->getNumChildShapes();
	}

	return 0;
}

int CCollisionQuery::TriangleCount(int convexIndex) {
#ifdef USE_CONVEX_TRIANGLES
	Assert(convexIndex < m_pCollide->GetCompoundShape()->getNumChildShapes());
	//btConvexTriangleMeshShape *pChild = (btConvexTriangleMeshShape *)m_pCollide->GetCompoundShape()->getChildShape(convexIndex);

	//return pChild->getNumVertices
	return 0;
#else
	NOT_IMPLEMENTED
	return 0;
#endif
}

unsigned int CCollisionQuery::GetGameData(int convexIndex) {
	NOT_IMPLEMENTED
	return 0;
}

void CCollisionQuery::GetTriangleVerts(int convexIndex, int triangleIndex, Vector *verts) {
	NOT_IMPLEMENTED
}

void CCollisionQuery::SetTriangleVerts(int convexIndex, int triangleIndex, const Vector *verts) {
	NOT_IMPLEMENTED
}

int CCollisionQuery::GetTriangleMaterialIndex(int convexIndex, int triangleIndex) {
	NOT_IMPLEMENTED
	return 0;
}

void CCollisionQuery::SetTriangleMaterialIndex(int convexIndex, int triangleIndex, int index7bits) {
	NOT_IMPLEMENTED
}

/****************************
* BYTESWAP DATA DESCRIPTIONS
****************************/

BEGIN_BYTESWAP_DATADESC(collideheader_t)
	DEFINE_FIELD(size, FIELD_INTEGER),
	DEFINE_FIELD(vphysicsID, FIELD_INTEGER),
	DEFINE_FIELD(version, FIELD_SHORT),
	DEFINE_FIELD(modelType, FIELD_SHORT),
END_BYTESWAP_DATADESC()

BEGIN_BYTESWAP_DATADESC(compactsurfaceheader_t)
	DEFINE_FIELD(surfaceSize, FIELD_INTEGER),
	DEFINE_FIELD(dragAxisAreas, FIELD_VECTOR),
	DEFINE_FIELD(axisMapSize, FIELD_INTEGER),
END_BYTESWAP_DATADESC()

BEGIN_BYTESWAP_DATADESC(moppsurfaceheader_t)
	DEFINE_FIELD(moppSize, FIELD_INTEGER),
END_BYTESWAP_DATADESC()

BEGIN_BYTESWAP_DATADESC(ivpcompactsurface_t)
	DEFINE_ARRAY(mass_center, FIELD_FLOAT, 3),
	DEFINE_ARRAY(rotation_inertia, FIELD_FLOAT, 3),
	DEFINE_FIELD(upper_limit_radius, FIELD_FLOAT),
	DEFINE_BITFIELD(bf1, FIELD_INTEGER, 8),
	DEFINE_BITFIELD(bf1, FIELD_INTEGER, 24),
	DEFINE_FIELD(offset_ledgetree_root, FIELD_INTEGER),
	DEFINE_ARRAY(dummy, FIELD_INTEGER, 3),
END_BYTESWAP_DATADESC()

BEGIN_BYTESWAP_DATADESC(ivpcompactmopp_t)
	DEFINE_ARRAY(mass_center, FIELD_FLOAT, 3),
	DEFINE_ARRAY(rotation_inertia, FIELD_FLOAT, 3),
	DEFINE_FIELD(upper_limit_radius, FIELD_FLOAT),
	DEFINE_BITFIELD(bf1, FIELD_INTEGER, 8),
	DEFINE_BITFIELD(bf1, FIELD_INTEGER, 24),
	DEFINE_FIELD(offset_ledgetree_root, FIELD_INTEGER),
	DEFINE_FIELD(offset_ledges, FIELD_INTEGER),
	DEFINE_FIELD(size_convex_hull, FIELD_INTEGER),
	DEFINE_FIELD(dummy, FIELD_INTEGER),
END_BYTESWAP_DATADESC()

BEGIN_BYTESWAP_DATADESC(ivpcompactledge_t)
	DEFINE_FIELD(c_point_offset, FIELD_INTEGER),
	DEFINE_FIELD(ledgetree_node_offset, FIELD_INTEGER),
	DEFINE_BITFIELD(bf1, FIELD_INTEGER, 2),
	DEFINE_BITFIELD(bf1, FIELD_INTEGER, 2),
	DEFINE_BITFIELD(bf1, FIELD_INTEGER, 4),
	DEFINE_BITFIELD(bf1, FIELD_INTEGER, 24),
	DEFINE_FIELD(n_triangles, FIELD_SHORT),
	DEFINE_FIELD(for_future_use, FIELD_SHORT),
END_BYTESWAP_DATADESC()

BEGIN_BYTESWAP_DATADESC(ivpcompactedge_t)
	DEFINE_BITFIELD(bf1, FIELD_INTEGER, 16),
	DEFINE_BITFIELD(bf1, FIELD_INTEGER, 15),
	DEFINE_BITFIELD(bf1, FIELD_INTEGER, 1),
END_BYTESWAP_DATADESC()

BEGIN_BYTESWAP_DATADESC(ivpcompacttriangle_t)
	DEFINE_BITFIELD(bf1, FIELD_INTEGER, 12),
	DEFINE_BITFIELD(bf1, FIELD_INTEGER, 12),
	DEFINE_BITFIELD(bf1, FIELD_INTEGER, 7),
	DEFINE_BITFIELD(bf1, FIELD_INTEGER, 1),

	DEFINE_EMBEDDED_ARRAY(c_three_edges, 3)
END_BYTESWAP_DATADESC()

BEGIN_BYTESWAP_DATADESC(ivpcompactledgenode_t)
	DEFINE_FIELD(offset_right_node, FIELD_INTEGER),
	DEFINE_FIELD(offset_compact_ledge, FIELD_INTEGER),
	DEFINE_ARRAY(center, FIELD_FLOAT, 3),
	DEFINE_FIELD(radius, FIELD_FLOAT),
	DEFINE_ARRAY(box_sizes, FIELD_CHARACTER, 3),
	DEFINE_FIELD(free_0, FIELD_CHARACTER),
END_BYTESWAP_DATADESC()

/****************************
* CLASS CPhysCollide
****************************/

CPhysCollide::CPhysCollide(btCollisionShape *pShape) {
	m_pShape = pShape;
	m_pShape->setUserPointer(this);

	m_massCenter.setZero();
}

/****************************
* CLASS CPhysPolySoup
****************************/

class CPhysPolysoup {
	public:
		CPhysPolysoup() {

		}

	private:
		
};

/****************************
* CLASS CPhysicsCollision
****************************/

// NOTE:
// CPhysCollide is usually a btCompoundShape
// CPhysConvex is usually a btConvexHullShape

#define VPHYSICS_ID					MAKEID('V', 'P', 'H', 'Y')
#define IVP_COMPACT_SURFACE_ID		MAKEID('I', 'V', 'P', 'S')
#define IVP_COMPACT_MOPP_ID			MAKEID('M', 'O', 'P', 'P')

CPhysicsCollision::CPhysicsCollision() {
	// Default to old behavior
	CPhysicsCollision::EnableBBoxCache(true);
}

CPhysicsCollision::~CPhysicsCollision() {
	ClearBBoxCache();
}

// FIXME: Why is it important to have an array of pointers?
CPhysConvex *CPhysicsCollision::ConvexFromVerts(Vector **ppVerts, int vertCount) {
	if (!ppVerts || vertCount == 0) return NULL;

	// Convert the array and call the function below
	Vector *pVerts = new Vector[vertCount];
	for (int i = 0; i < vertCount; i++) {
		pVerts[i] = *ppVerts[i];
	}

	CPhysConvex *pConvex = ConvexFromVerts(pVerts, vertCount);
	delete[] pVerts;

	return pConvex;
}

btConvexTriangleMeshShape *CreateTriMeshFromHull(HullResult &res) {
	btTriangleIndexVertexArray *pMesh = new btTriangleIndexVertexArray();
	btIndexedMesh mesh;
	mesh.m_numTriangles = res.mNumIndices / 3;

	// Duplicate the output vertex array
	mesh.m_numVertices = res.mNumOutputVertices;
	btVector3 *pVerts = new btVector3[res.mNumOutputVertices];
	for (uint i = 0; i < res.mNumOutputVertices; i++) {
		pVerts[i] = res.m_OutputVertices[i];
	}

	mesh.m_vertexBase = reinterpret_cast<unsigned char*>(pVerts);
	mesh.m_vertexStride = sizeof(btVector3);
	mesh.m_vertexType = PHY_FLOAT;

	// Duplicate the index array
	unsigned int *pIndices = new unsigned int[res.mNumIndices];
	for (uint i = 0; i < res.mNumIndices; i++) {
		pIndices[i] = res.m_Indices[i];
	}

	mesh.m_triangleIndexBase = (unsigned char *)pIndices;
	mesh.m_triangleIndexStride = 3 * sizeof(unsigned short);

	pMesh->addIndexedMesh(mesh, PHY_SHORT);
	btConvexTriangleMeshShape *pShape = new btConvexTriangleMeshShape(pMesh);

	return pShape;
}

// Newer version of the above (just an array, not an array of pointers)
CPhysConvex *CPhysicsCollision::ConvexFromVerts(const Vector *pVerts, int vertCount) {
	if (!pVerts || vertCount == 0) return NULL;

	btVector3 *pBullVerts = new btVector3[vertCount];

	for (int i = 0; i < vertCount; i++) {
		ConvertPosToBull(pVerts[i], pBullVerts[i]);
	}

	HullLibrary lib;

	HullResult res;
	HullDesc desc(QF_TRIANGLES, vertCount, pBullVerts);
	HullError err = lib.CreateConvexHull(desc, res);
	// A problem occurred in creating the hull :(
	if (err != QE_OK)
		return NULL;

	// Okay, create a triangle mesh with this.
	btConvexTriangleMeshShape *pMesh = CreateTriMeshFromHull(res);
	lib.ReleaseResult(res);

	return (CPhysConvex *)pMesh;
}

CPhysConvex *CPhysicsCollision::ConvexFromPlanes(float *pPlanes, int planeCount, float mergeDistance) {
	NOT_IMPLEMENTED
	return NULL;
}

float CPhysicsCollision::ConvexVolume(CPhysConvex *pConvex) {
	if (!pConvex) return 0;

	btCollisionShape *pShape = (btCollisionShape *)pConvex;

	if (pShape->getShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE) {
		btConvexTriangleMeshShape *pTriShape = (btConvexTriangleMeshShape *)pShape;
		btTriangleIndexVertexArray *pArr = (btTriangleIndexVertexArray *)pTriShape->getMeshInterface();
		btIndexedMesh &pMesh = pArr->getIndexedMeshArray()[0];

		btVector3 *pVertexArray = (btVector3 *)pMesh.m_vertexBase;
		unsigned short *pIndexArray = (unsigned short *)pMesh.m_triangleIndexBase;

		// First, let's calculate the centroid of the shape.
		btVector3 centroid(0, 0, 0);
		for (int i = 0; i < pMesh.m_numVertices; i++) {
			btVector3 vertex = ((btVector3 *)pMesh.m_vertexBase)[i];
			centroid += vertex;
		}

		if (pMesh.m_numVertices > 0)
			centroid /= (btScalar)pMesh.m_numVertices;

		// Okay, now loop through all of the triangles of the shape. We're going to make the assumption
		// that none of the triangles overlap, and it's guaranteed that the centroid is inside the shape.

		btScalar sum(0);

		for (int i = 0; i < pMesh.m_numTriangles; i++) {
			btVector3 v[3];

			for (int j = 0; j < 3; j++) {
				v[j] = pVertexArray[pIndexArray[i * 3 + j]];
			}

			// Shorten the var name
			btVector3 &c = centroid;

			// Okay. We have the 4 vertices we need to perform the volume calculation. Now let's do it.
			btMatrix3x3 mat(v[0][0] - c[0], v[1][0] - c[0], v[2][0] - c[0],
							v[0][1] - c[1], v[1][1] - c[1], v[2][1] - c[1],
							v[0][2] - c[2], v[1][2] - c[2], v[2][2] - c[2]);

			// Aaand the volume is 1/6 the determinant of the matrix
			sum += mat.determinant() / 6;
		}

		return sum;
	}

	NOT_IMPLEMENTED
	return 0;
}

float CPhysicsCollision::ConvexSurfaceArea(CPhysConvex *pConvex) {
	if (!pConvex) return 0;

	btCollisionShape *pShape = (btCollisionShape *)pConvex;

	if (pShape->getShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE) {
		btConvexTriangleMeshShape *pTriShape = (btConvexTriangleMeshShape *)pShape;
		btTriangleIndexVertexArray *pArr = (btTriangleIndexVertexArray *)pTriShape->getMeshInterface();
		btIndexedMesh &pMesh = pArr->getIndexedMeshArray()[0];

		btVector3 *pVertexArray = (btVector3 *)pMesh.m_vertexBase;
		unsigned short *pIndexArray = (unsigned short *)pMesh.m_triangleIndexBase;

		// Loop through all of the triangles in the shape and add up the surface areas
		float sum = 0.f;

		for (int i = 0; i < pMesh.m_numTriangles; i++) {
			btVector3 v[3];
			for (int j = 0; j < 3; j++) {
				v[j] = pVertexArray[pIndexArray[i * 3 + j]];
			}

			// Area of a triangle is the length of the cross product of 2 of its sides divided by 2
			btVector3 ab = v[1] - v[0];
			btVector3 ac = v[2] - v[0];
			sum += ab.cross(ac).length() / 2;
		}

		return sum;
	}

	NOT_IMPLEMENTED
	return 0;
}

void CPhysicsCollision::ConvexFree(CPhysConvex *pConvex) {
	if (!pConvex) return;

	btCollisionShape *pShape = (btCollisionShape *)pConvex;

	if (pShape->getShapeType() == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE) {
		btStridingMeshInterface *pMesh = ((btConvexTriangleMeshShape *)pShape)->getMeshInterface();
		btTriangleIndexVertexArray *pTriArr = (btTriangleIndexVertexArray *)pMesh;
		IndexedMeshArray &arr = pTriArr->getIndexedMeshArray();
		for (int i = arr.size()-1; i >= 0; i--) {
			btIndexedMesh &mesh = arr[i];

			// Delete the index and vertex arrays (that we allocated back in VCollideLoad)
			unsigned short *indexBase = (unsigned short *)mesh.m_triangleIndexBase;
			delete [] indexBase;

			btVector3 *vertexBase = (btVector3 *)mesh.m_vertexBase;
			delete [] vertexBase;

			arr.pop_back();
		}

		delete pMesh;

		delete pShape;
	} else {
		delete pShape;
	}
}

// TODO: Need this to get contents of a convex in a compound shape
void CPhysicsCollision::SetConvexGameData(CPhysConvex *pConvex, unsigned int gameData) {
	btConvexShape *pShape = (btConvexShape *)pConvex;
	pShape->setUserPointer((void *)gameData);
}

// Appears to use a polyhedron class from mathlib
CPolyhedron *CPhysicsCollision::PolyhedronFromConvex(CPhysConvex *const pConvex, bool bUseTempPolyhedron) {
	NOT_IMPLEMENTED
	return NULL;
}

CPhysConvex *CPhysicsCollision::ConvexFromConvexPolyhedron(const CPolyhedron &ConvexPolyhedron) {
	NOT_IMPLEMENTED
	return NULL;
}

void CPhysicsCollision::ConvexesFromConvexPolygon(const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **ppOutput) {
	NOT_IMPLEMENTED
}

// TODO: Support this, gmod lua Entity:PhysicsInitMultiConvex uses this!
// IVP internally used QHull to generate the convexes.
CPhysPolysoup *CPhysicsCollision::PolysoupCreate() {
	return new CPhysPolysoup();
}

void CPhysicsCollision::PolysoupDestroy(CPhysPolysoup *pSoup) {
	delete pSoup;
}

void CPhysicsCollision::PolysoupAddTriangle(CPhysPolysoup *pSoup, const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits) {
	NOT_IMPLEMENTED
}

// TODO: This will involve convex decomposition, which breaks a concave shape into multiple convex shapes.
// Too complex to write myself, will need a library to do this. QHull does not support this.
CPhysCollide *CPhysicsCollision::ConvertPolysoupToCollide(CPhysPolysoup *pSoup, bool useMOPP) {
	NOT_IMPLEMENTED
	return NULL;
}

CPhysCollide *CPhysicsCollision::ConvertConvexToCollide(CPhysConvex **ppConvex, int convexCount) {
	if (convexCount == 0) return NULL;

	btCompoundShape *pCompound = new btCompoundShape;
	for (int i = 0; i < convexCount; i++) {
		// TODO: Copy the convex and delete all of the convexes in the array, as that's how IVP worked.
		btCollisionShape *pShape = (btCollisionShape *)ppConvex[i];
		//btCollisionShape *pShape = (btCollisionShape *)btAlignedAlloc(pOldShape->getByteSize(), 16);
		//*pShape = *pOldShape; // Now copy the data

		pCompound->addChildShape(btTransform::getIdentity(), pShape);
	}

	pCompound->setMargin(COLLISION_MARGIN);

	CPhysCollide *pCollide = new CPhysCollide(pCompound);
	return pCollide;
}

CPhysCollide *CPhysicsCollision::ConvertConvexToCollideParams(CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &convertParams) {
	NOT_IMPLEMENTED
	return ConvertConvexToCollide(pConvex, convexCount);
}

void CPhysicsCollision::AddConvexToCollide(CPhysCollide *pCollide, const CPhysConvex *pConvex, const matrix3x4_t *xform) {
	if (!pCollide || !pConvex) return;

	if (pCollide->IsCompound()) {
		btCompoundShape *pCompound = pCollide->GetCompoundShape();
		btCollisionShape *pShape = (btCollisionShape *)pConvex;

		btTransform trans = btTransform::getIdentity();
		if (xform) {
			ConvertMatrixToBull(*xform, trans);
		}

		pCompound->addChildShape(trans, pShape);
	}
}

void CPhysicsCollision::RemoveConvexFromCollide(CPhysCollide *pCollide, const CPhysConvex *pConvex) {
	if (!pCollide || !pConvex) return;

	if (pCollide->IsCompound()) {
		btCompoundShape *pCompound = pCollide->GetCompoundShape();
		btCollisionShape *pShape = (btCollisionShape *)pConvex;

		// FIXME: Need to recalculate the aabb tree or something
		pCompound->removeChildShape(pShape);
	}
}

CPhysCollide *CPhysicsCollision::CreateCollide() {
	btCompoundShape *pShape = new btCompoundShape();
	return new CPhysCollide(pShape);
}

void CPhysicsCollision::DestroyCollide(CPhysCollide *pCollide) {
	if (!pCollide || IsCachedBBox(pCollide)) return;

	btCollisionShape *pShape = pCollide->GetCollisionShape();

	// Compound shape? Delete all of its children.
	if (pShape->isCompound()) {
		btCompoundShape *pCompound = (btCompoundShape *)pShape;

		for (int i = pCompound->getNumChildShapes()-1; i >= 0; i--) {
			btCollisionShape *pShape = pCompound->getChildShape(i);
			Assert(!pShape->isCompound()); // Compounds shouldn't have compound children.

			pCompound->removeChildShapeByIndex(i);
			ConvexFree((CPhysConvex *)pShape);
		}

		delete pCollide;
		delete pCompound;
	} else if (pShape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE) {
		// Delete the striding mesh interface (which we allocate)
		btStridingMeshInterface *pMesh = ((btTriangleMeshShape *)pShape)->getMeshInterface();
		if (((btBvhTriangleMeshShape *)pShape)->getTriangleInfoMap())
			delete ((btBvhTriangleMeshShape *)pShape)->getTriangleInfoMap(); // Probably shouldn't be casting this to a btBvhTriangleMeshShape. Whatever.

		btTriangleIndexVertexArray *pTriArr = (btTriangleIndexVertexArray *)pMesh;
		IndexedMeshArray &arr = pTriArr->getIndexedMeshArray();
		for (int i = arr.size() - 1; i >= 0; i--) {
			btIndexedMesh &mesh = arr[i];

			// Delete the index and vertex arrays (that we allocated back in VCollideLoad)
			unsigned short *indexBase = (unsigned short *)mesh.m_triangleIndexBase;
			delete[] indexBase;

			btVector3 *vertexBase = (btVector3 *)mesh.m_vertexBase;
			delete[] vertexBase;

			arr.pop_back();
		}

		delete pMesh;

		delete pShape;

		delete pCollide;
	} else {
		// Those dirty liars!
		ConvexFree((CPhysConvex *)pCollide);
	}
}

int CPhysicsCollision::CollideSize(CPhysCollide *pCollide) {
	NOT_IMPLEMENTED
	return 0;
}

// TODO: Design a new file format
int CPhysicsCollision::CollideWrite(char *pDest, CPhysCollide *pCollide, bool swap) {
	NOT_IMPLEMENTED
	return 0;
}

CPhysCollide *CPhysicsCollision::UnserializeCollide(char *pBuffer, int size, int index) {
	NOT_IMPLEMENTED
	return NULL;
}

float CPhysicsCollision::CollideVolume(CPhysCollide *pCollide) {
	btCompoundShape *pCompound = pCollide->GetCompoundShape();

	// Loop through the children and sum the volume
	float sum = 0.f;
	for (int i = 0; i < pCompound->getNumChildShapes(); i++) {
		btCollisionShape *pChild = pCompound->getChildShape(i);

		sum += ConvexVolume((CPhysConvex *)pChild);
	}

	return sum;
}

float CPhysicsCollision::CollideSurfaceArea(CPhysCollide *pCollide) {
	btCompoundShape *pCompound = pCollide->GetCompoundShape();

	// Loop through the children and sum the area
	float sum = 0.f;
	for (int i = 0; i < pCompound->getNumChildShapes(); i++) {
		btCollisionShape *pChild = pCompound->getChildShape(i);

		sum += ConvexSurfaceArea((CPhysConvex *)pChild);
	}

	return sum;
}

// This'll return the farthest possible vector that's still within our collision mesh.
Vector CPhysicsCollision::CollideGetExtent(const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction) {
	if (!pCollide) return collideOrigin;

	btVector3 btDirection;
	ConvertDirectionToBull(direction, btDirection);

	btVector3 origin;
	btQuaternion angles;
	ConvertPosToBull(collideOrigin, origin);
	ConvertRotationToBull(collideAngles, angles);

	btTransform trans(angles, origin);
	trans *= btTransform(btQuaternion::getIdentity(), pCollide->GetMassCenter()).inverse();

	if (pCollide->IsCompound()) {
		btVector3 maxExtents(0, 0, 0);
		btScalar maxDot = 0;

		const btCompoundShape *pCompound = pCollide->GetCompoundShape();
		for (int i = 0; i < pCompound->getNumChildShapes(); i++) {
			const btCollisionShape *pShape = pCompound->getChildShape(i);
			const btTransform &childTrans = pCompound->getChildTransform(i);
			if (pShape->isConvex()) {
				const btConvexShape *pConvex = (const btConvexShape *)pShape;
				btVector3 extents = pConvex->localGetSupportingVertex(btDirection);
				extents = childTrans * extents; // Move the extents by the object's transform

				if (extents.dot(btDirection) > maxDot) {
					maxExtents = extents;
					maxDot = extents.dot(btDirection);
				}
			}
		}

		btVector3 vec = trans * maxExtents;
		Vector hlVec;
		ConvertPosToHL(vec, hlVec);

		/*
		if (g_pDebugOverlay) {
			g_pDebugOverlay->AddLineOverlay(collideOrigin, hlVec, 0, 255, 0, true, 5.f);
			g_pDebugOverlay->AddBoxOverlay(hlVec, Vector(-4, -4, -4), Vector(4, 4, 4), QAngle(0, 0, 0), 255, 0, 0, 255, 5.f);
		}
		*/

		return hlVec;
	} else if (pCollide->IsConvex()) {
		const btConvexShape *pConvex = pCollide->GetConvexShape();
		btVector3 maxExtents = pConvex->localGetSupportingVertex(btDirection);

		btVector3 vec = trans * maxExtents;
		Vector hlVec;
		ConvertPosToHL(vec, hlVec);

		/*
		if (g_pDebugOverlay) {
			g_pDebugOverlay->AddLineOverlay(collideOrigin, hlVec, 0, 255, 0, true, 5.f);
			g_pDebugOverlay->AddBoxOverlay(hlVec, Vector(-4, -4, -4), Vector(4, 4, 4), QAngle(0, 0, 0), 255, 0, 0, 255, 5.f);
		}
		*/

		return hlVec;
	}

	return collideOrigin;
}

void CPhysicsCollision::CollideGetAABB(Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles) {
	if (!pCollide || (!pMins && !pMaxs)) return;

	// Bullet returns very different AABBs than Havok.
	const btCollisionShape *shape = pCollide->GetCollisionShape();

	btVector3 pos, mins, maxs;
	btMatrix3x3 rot;

	ConvertPosToBull(collideOrigin, pos);
	ConvertRotationToBull(collideAngles, rot);
	btTransform transform(rot, pos);

	transform *= btTransform(btQuaternion::getIdentity(), pCollide->GetMassCenter());

	shape->getAabb(transform, mins, maxs);

	Vector tMins, tMaxs;
	ConvertAABBToHL(mins, maxs, tMins, tMaxs);

	if (pMins)
		*pMins = tMins;

	if (pMaxs)
		*pMaxs = tMaxs;
}

void CPhysicsCollision::CollideGetMassCenter(CPhysCollide *pCollide, Vector *pOutMassCenter) {
	if (!pCollide || !pOutMassCenter) return;

	ConvertPosToHL(pCollide->GetMassCenter(), *pOutMassCenter);
}

void CPhysicsCollision::CollideSetMassCenter(CPhysCollide *pCollide, const Vector &massCenter) {
	if (!pCollide) return;

	btCollisionShape *pShape = pCollide->GetCollisionShape();

	btVector3 bullMassCenter;
	ConvertPosToBull(massCenter, bullMassCenter);

	// Since mass centers are kind of a hack in our implementation, take care of updating the compound shape's children.
	// FIXME: May cause some issues with rigid bodies "moving" around, or something.
	btVector3 offset = bullMassCenter - pCollide->GetMassCenter();
	if (pShape->isCompound()) {
		btCompoundShape *pCompound = (btCompoundShape *)pShape;
		for (int i = 0; i < pCompound->getNumChildShapes(); i++) {
			btTransform childTrans = pCompound->getChildTransform(i);
			childTrans.setOrigin(childTrans.getOrigin() + offset);
			pCompound->updateChildTransform(i, childTrans);
		}
	}

	pCollide->SetMassCenter(bullMassCenter);
}

Vector CPhysicsCollision::CollideGetOrthographicAreas(const CPhysCollide *pCollide) {
	// What is this?
	NOT_IMPLEMENTED
	return Vector(1, 1, 1); // Documentation says we will return 1,1,1 if ortho areas undefined
}

void CPhysicsCollision::CollideSetOrthographicAreas(CPhysCollide *pCollide, const Vector &areas) {
	NOT_IMPLEMENTED
}

void CPhysicsCollision::CollideSetScale(CPhysCollide *pCollide, const Vector &scale) {
	if (!pCollide) return;

	if (pCollide->IsCompound()) {
		btCompoundShape *pCompound = pCollide->GetCompoundShape();

		btVector3 bullScale;
		bullScale.setX(scale.x);
		bullScale.setY(scale.z);
		bullScale.setZ(scale.y);

		pCompound->setLocalScaling(bullScale);
	}
}

void CPhysicsCollision::CollideGetScale(const CPhysCollide *pCollide, Vector &out) {
	if (!pCollide) return;

	if (pCollide->IsCompound()) {
		const btCompoundShape *pCompound = pCollide->GetCompoundShape();

		btVector3 scale = pCompound->getLocalScaling();
		out.x = scale.getX();
		out.y = scale.getZ();
		out.z = scale.getY();
	}
}

int CPhysicsCollision::CollideIndex(const CPhysCollide *pCollide) {
	NOT_IMPLEMENTED
	return 0;
}

int CPhysicsCollision::GetConvexesUsedInCollideable(const CPhysCollide *pCollideable, CPhysConvex **pOutputArray, int iOutputArrayLimit) {
	if (!pCollideable->IsCompound()) return 0;

	const btCompoundShape *pCompound = pCollideable->GetCompoundShape();
	int numSolids = pCompound->getNumChildShapes();
	for (int i = 0; i < numSolids && i < iOutputArrayLimit; i++) {
		const btCollisionShape *pConvex = pCompound->getChildShape(i);
		pOutputArray[i] = (CPhysConvex *)pConvex;
	}

	return numSolids > iOutputArrayLimit ? iOutputArrayLimit : numSolids;
}

CPhysCollide *CPhysicsCollision::GetCachedBBox(const Vector &mins, const Vector &maxs) {
	for (int i = 0; i < m_bboxCache.Count(); i++) {
		bboxcache_t &cache = m_bboxCache[i];

		if (cache.mins == mins && cache.maxs == maxs)
			return cache.pCollide;
	}

	return NULL;
}

void CPhysicsCollision::AddCachedBBox(CPhysCollide *pModel, const Vector &mins, const Vector &maxs) {
	int idx = m_bboxCache.AddToTail();
	bboxcache_t &cache = m_bboxCache[idx];
	cache.pCollide = pModel;
	cache.mins = mins;
	cache.maxs = maxs;
}

bool CPhysicsCollision::IsCachedBBox(CPhysCollide *pModel) {
	for (int i = 0; i < m_bboxCache.Count(); i++) {
		bboxcache_t &cache = m_bboxCache[i];

		if (cache.pCollide == pModel)
			return true;
	}

	return false;
}

void CPhysicsCollision::ClearBBoxCache() {
	for (int i = m_bboxCache.Count() - 1; i >= 0; i--) {
		bboxcache_t &cache = m_bboxCache[i];

		// Remove the cache first so DestroyCollide doesn't stop.
		CPhysCollide *pCollide = cache.pCollide;
		m_bboxCache.Remove(i);

		DestroyCollide(pCollide);
	}
}

bool CPhysicsCollision::GetBBoxCacheSize(int *pCachedSize, int *pCachedCount) {
	// pCachedSize is size in bytes
	if (pCachedSize)
		*pCachedSize = m_bboxCache.Count() * sizeof(bboxcache_t);

	if (pCachedCount)
		*pCachedCount = m_bboxCache.Count();

	// Bool return value is never used.
	return false;
}

void CPhysicsCollision::EnableBBoxCache(bool enable) {
	m_enableBBoxCache = enable;
}

bool CPhysicsCollision::IsBBoxCacheEnabled() {
	return m_enableBBoxCache;
}

CPhysConvex *CPhysicsCollision::BBoxToConvex(const Vector &mins, const Vector &maxs) {
	if (mins == maxs) return NULL;

	btVector3 btmins, btmaxs;
	ConvertAABBToBull(mins, maxs, btmins, btmaxs);
	btVector3 halfExtents = (btmaxs - btmins) / 2;

	btBoxShape *box = new btBoxShape(halfExtents);

	return (CPhysConvex *)box;
}

CPhysCollide *CPhysicsCollision::BBoxToCollide(const Vector &mins, const Vector &maxs) {
	// consult with the bbox cache first (this is old vphysics behavior)
	if (m_enableBBoxCache) {
		CPhysCollide *pCached = GetCachedBBox(mins, maxs);
		if (pCached)
			return pCached;
	}

	CPhysConvex *pConvex = BBoxToConvex(mins, maxs);
	if (!pConvex) return NULL;

	btCompoundShape *pCompound = new btCompoundShape;
	pCompound->setMargin(COLLISION_MARGIN);

	btVector3 btmins, btmaxs;
	ConvertAABBToBull(mins, maxs, btmins, btmaxs);

	btVector3 halfExtents = (btmaxs - btmins) / 2;
	pCompound->addChildShape(btTransform(btMatrix3x3::getIdentity(), btmins + halfExtents), (btCollisionShape *)pConvex);

	CPhysCollide *pCollide = new CPhysCollide(pCompound);

	if (m_enableBBoxCache)
		AddCachedBBox(pCollide, mins, maxs);

	return pCollide;
}

CPhysConvex *CPhysicsCollision::CylinderToConvex(const Vector &mins, const Vector &maxs) {
	if (mins == maxs) return NULL;

	btVector3 btmins, btmaxs;
	ConvertAABBToBull(mins, maxs, btmins, btmaxs);
	btVector3 halfSize = (btmaxs - btmins) / 2;

	btCylinderShape *pShape = new btCylinderShape(halfSize);

	return (CPhysConvex *)pShape;
}

CPhysConvex *CPhysicsCollision::ConeToConvex(const float radius, const float height) {
	btConeShape *pShape = new btConeShape(ConvertDistanceToBull(radius), ConvertDistanceToBull(height));
	return (CPhysConvex *)pShape;
}

CPhysConvex *CPhysicsCollision::SphereToConvex(const float radius) {
	if (radius <= 0) return NULL;

	btSphereShape *pShape = new btSphereShape(ConvertDistanceToBull(radius));
	return (CPhysConvex *)pShape;
}

void CPhysicsCollision::TraceBox(const Vector &start, const Vector &end, const Vector &mins, const Vector &maxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	Ray_t ray;
	ray.Init(start, end, mins, maxs);
	return TraceBox(ray, pCollide, collideOrigin, collideAngles, ptr);
}

void CPhysicsCollision::TraceBox(const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	return TraceBox(ray, MASK_ALL, NULL, pCollide, collideOrigin, collideAngles, ptr);
}

class CFilteredRayResultCallback : public btCollisionWorld::ClosestRayResultCallback {
	public:
		CFilteredRayResultCallback(btVector3 &rayFromWorld, btVector3 &rayToWorld, btCollisionShape *pShape, int contentsMask, IConvexInfo *pConvexInfo): btCollisionWorld::ClosestRayResultCallback(rayFromWorld, rayToWorld) {
			m_contentsMask = contentsMask;
			m_pConvexInfo = pConvexInfo;
			m_pShape = pShape;
		}

		virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult &rayResult, bool normalInWorldSpace) {
			// Test the convex's contents before we do anything else.
			// The hit convex ID comes in from convex result's local shape info's triangle ID (stupid but whatever)
			if (m_pConvexInfo && rayResult.m_localShapeInfo && rayResult.m_localShapeInfo->m_triangleIndex >= 0) {
				btCollisionShape *pShape = ((btCompoundShape *)m_pShape)->getChildShape(rayResult.m_localShapeInfo->m_triangleIndex);
				if (pShape) {
					int contents = m_pConvexInfo->GetContents(pShape->getUserIndex());

					// If none of the contents are within the mask, abort!
					if (!(contents & m_contentsMask)) {
						return 1;
					}
				}
			}

			return btCollisionWorld::ClosestRayResultCallback::addSingleResult(rayResult, normalInWorldSpace);
		}

	private:
		int				m_contentsMask;
		IConvexInfo *	m_pConvexInfo;
		btCollisionShape *m_pShape;
};

class CFilteredConvexResultCallback : public btCollisionWorld::ClosestConvexResultCallback {
	public:
		CFilteredConvexResultCallback(btVector3 &start, btVector3 &end, btCollisionShape *pShape, int contentsMask, IConvexInfo *pConvexInfo) : btCollisionWorld::ClosestConvexResultCallback(start, end) {
			m_contentsMask = contentsMask;
			m_pConvexInfo = pConvexInfo;
			m_pShape = pShape;
		}

		virtual	btScalar addSingleResult(btCollisionWorld::LocalConvexResult &convexResult, bool normalInWorldSpace) {
			// Test the convex's contents before we do anything else.
			// The hit convex ID comes in from convex result's local shape info's triangle ID (stupid but whatever)
			if (m_pConvexInfo && convexResult.m_localShapeInfo && convexResult.m_localShapeInfo->m_triangleIndex >= 0) {
				btCollisionShape *pShape = ((btCompoundShape *)m_pShape)->getChildShape(convexResult.m_localShapeInfo->m_triangleIndex);
				if (pShape) {
					int contents = m_pConvexInfo->GetContents(pShape->getUserIndex());

					// If none of the contents are within the mask, abort!
					if (!(contents & m_contentsMask)) {
						return 1;
					}
				}
			}

			return btCollisionWorld::ClosestConvexResultCallback::addSingleResult(convexResult, normalInWorldSpace);
		}

	private:
		int				m_contentsMask;
		IConvexInfo *	m_pConvexInfo;
		btCollisionShape *m_pShape;
};

static ConVar bt_visualizetraces("bt_visualizetraces", "0", FCVAR_CHEAT, "Visualize physics traces");

void CPhysicsCollision::TraceBox(const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr) {
	if (!pCollide || !ptr) return;

	// Clear the trace (appears engine does not do this every time)
	memset(ptr, 0, sizeof(trace_t));
	ptr->fraction = 1.f;
	ptr->fractionleftsolid = 0;
	ptr->surface.flags = 0;
	ptr->surface.name = "**empty**";

	// 2 Variables used mainly for converting units.
	btVector3 btvec;
	btMatrix3x3 btmatrix;

	btCollisionObject *object = new btCollisionObject;
	btCollisionShape *shape = (btCollisionShape *)pCollide->GetCollisionShape();
	object->setCollisionShape(shape);

	// Set the object's transform
	ConvertPosToBull(collideOrigin, btvec);
	ConvertRotationToBull(collideAngles, btmatrix);
	btTransform transform(btmatrix, btvec);

	// Offset it by the mass center (bullet obj centers are at the center of mass)
	transform *= btTransform(btMatrix3x3::getIdentity(), pCollide->GetMassCenter());
	object->setWorldTransform(transform);

	// Setup the start and end positions
	btVector3 startv, endv;
	ConvertPosToBull(ray.m_Start, startv);
	ConvertPosToBull(ray.m_Start + ray.m_Delta, endv);

	ptr->startpos = ray.m_Start + ray.m_StartOffset;

	// Convert the positions to transforms (with no rotation because source doesn't support that)
	btTransform startt(btMatrix3x3::getIdentity(), startv);
	btTransform endt(btMatrix3x3::getIdentity(), endv);

	// Single line trace must be supported in TraceBox? Yep, you betcha.
	// FIXME: We can't use frac == 0 to determine if the trace was started in a solid! Need to detect this separately.
	if (ray.m_IsRay) {
		CFilteredRayResultCallback cb(startv, endv, shape, contentsMask, pConvexInfo);
		btCollisionWorld::rayTestSingle(startt, endt, object, shape, transform, cb);

		ptr->fraction = cb.m_closestHitFraction;

		// Data is uninitialized if frac is 1
		if (cb.m_closestHitFraction < 1.0) {
			if (cb.m_closestHitFraction == 0.f) {
				ptr->startsolid = true;
				ptr->allsolid = true;

				ptr->endpos = ptr->startpos;
			} else {
				ConvertDirectionToHL(cb.m_hitNormalWorld, ptr->plane.normal);
				ptr->endpos = ptr->startpos + (ray.m_Delta * ptr->fraction);

				ptr->startsolid = false;
				ptr->allsolid = false;
			}
		} else {
			ptr->endpos = ptr->startpos + ray.m_Delta;
		}

		if (bt_visualizetraces.GetBool() && g_pDebugOverlay) {
			g_pDebugOverlay->AddLineOverlay(ptr->startpos, ptr->endpos, 0, 0, 255, false, 0.f);

			if (ptr->fraction < 1.f) {
				btVector3 lineEnd = cb.m_hitPointWorld + (cb.m_hitNormalWorld * 1);
				Vector hlEnd, hlStart;
				ConvertPosToHL(lineEnd, hlEnd);
				ConvertPosToHL(cb.m_hitPointWorld, hlStart);

				g_pDebugOverlay->AddLineOverlay(hlStart, hlEnd, 0, 255, 0, false, 0.0f);
			}
		}
	} else if (ray.m_IsSwept) { // Box trace!
		if (bt_visualizetraces.GetBool() && g_pDebugOverlay) {
			// Trace start box (red)
			g_pDebugOverlay->AddBoxOverlay(ray.m_Start, -ray.m_Extents, ray.m_Extents, QAngle(0, 0, 0), 255, 0, 0, 10, 0.0f);

			// End trace box (blue)
			g_pDebugOverlay->AddBoxOverlay(ray.m_Start + ray.m_Delta, -ray.m_Extents, ray.m_Extents, QAngle(0, 0, 0), 0, 0, 255, 10, 0.0f);
		}

		// extents are half extents, compatible with bullet.
		ConvertPosToBull(ray.m_Extents, btvec);
		btBoxShape *box = new btBoxShape(btvec.absolute());

		CFilteredConvexResultCallback cb(startv, endv, shape, contentsMask, pConvexInfo);
		btCollisionWorld::objectQuerySingle(box, startt, endt, object, shape, transform, cb, 0.f);

		ptr->fraction = cb.m_closestHitFraction;

		// Data is uninitialized if frac is 1
		if (cb.m_closestHitFraction < 1.0) {
			// Penetration dist is the amount the last ray trace went through the object in meters (neg = penetration)
			// Allow penetrations up to 1 centimeter
			if (cb.m_closestHitFraction != 0.f) {
				ConvertDirectionToHL(cb.m_hitNormalWorld, ptr->plane.normal);

				ptr->startsolid = false;
				ptr->allsolid = false;
			//} else if (cb.m_closestHitFraction == 0.f && cb.m_penetrationDist >= -0.01f) {
			//lwss: This threshold was too low for kisak-strike. Kept getting stuck on things.
			} else if (cb.m_closestHitFraction == 0.f && cb.m_penetrationDist >= -0.05f) {
            //lwss end
				ConvertDirectionToHL(cb.m_hitNormalWorld, ptr->plane.normal);

				// HACK:
				// We're here because the penetration distance is within tolerable levels (aka on surface of object)

				// If the ray's delta is perpendicular to the hit normal, allow the fraction to be 1
				// (AKA: If the ray isn't traveling into the object, allow it to keep going)

				// FIXME: What if a separate convex is at the end of the ray?
				btVector3 direction;
				ConvertPosToBull(ray.m_Delta, direction);
				if (!btFuzzyZero(direction.length2())) {
					direction.normalize();
					if (direction.dot(cb.m_hitNormalWorld) >= -0.0005) {
						// Run another trace with an allowed penetration of something above 0.01

						ptr->fraction = 1;
					}
				} else {
					// Zero-length trace.
					ptr->fraction = 1.f;
					ptr->startsolid = false;
					ptr->allsolid = false;
				}
			} else {
				// Trace started and ended instantly (fraction of 0). We're stuck in a solid!
				// TODO: Need to properly hook up allsolid (tells source if the trace exits the solid,
				// and if so, at what fraction it exits the solid)
				ptr->startsolid = true;
				ptr->allsolid = true;
			}

			// Debug trace visualizing
			if (bt_visualizetraces.GetBool() && g_pDebugOverlay) {
				if (!ptr->allsolid) {
					btVector3 lineEnd = cb.m_hitPointWorld + (cb.m_hitNormalWorld * 1);
					Vector hlEnd, hlStart;
					ConvertPosToHL(lineEnd, hlEnd);
					ConvertPosToHL(cb.m_hitPointWorld, hlStart);

					g_pDebugOverlay->AddLineOverlay(hlStart, hlEnd, 0, 255, 0, false, 0.0f);
				}
			}
		}

		ptr->endpos = ptr->startpos + (ray.m_Delta * ptr->fraction);

		if (bt_visualizetraces.GetBool() && g_pDebugOverlay) {
			if (ptr->startsolid) {
				g_pDebugOverlay->AddTextOverlay(ptr->endpos, 0, 0.f, "Trace started in solid!");
			}
		}

		delete box;
	}

	delete object;
}

void CPhysicsCollision::TraceCollide(const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace) {
	/*
	btVector3 bullVec;
	btMatrix3x3 bullMatrix;

	// Create the collision object (object to be traced against)
	btCollisionObject *object = new btCollisionObject;
	btCollisionShape *shape = (btCollisionShape *)pCollide->GetCollisionShape();
	object->setCollisionShape(shape);
	ConvertRotationToBull(collideAngles, bullMatrix);
	ConvertPosToBull(collideOrigin, bullVec);
	btTransform transform(bullMatrix, bullVec);

	PhysicsShapeInfo *shapeInfo = (PhysicsShapeInfo *)shape->getUserPointer();
	if (shapeInfo)
		transform *= btTransform(btMatrix3x3::getIdentity(), shapeInfo->massCenter);
	object->setWorldTransform(transform);

	btVector3 bullStartVec, bullEndVec;
	ConvertPosToBull(start, bullStartVec);
	ConvertPosToBull(end, bullEndVec);
	btTransform bullStartT(btMatrix3x3::getIdentity(), bullStartVec);
	btTransform bullEndT(btMatrix3x3::getIdentity(), bullEndVec);

	btCollisionShape *pSweepShape = (btCollisionShape *)pSweepCollide->GetCollisionShape();
	if (pSweepShape->isCompound()) {

	}

	pTrace->fraction = cb.m_closestHitFraction;
	if (cb.m_closestHitFraction < 1.f) {
		ConvertPosToHL(cb.m_hitPointWorld, pTrace->endpos);
		ConvertDirectionToHL(cb.m_hitNormalWorld, pTrace->plane.normal);
	}

	// Cleanup
	delete object;
	*/

	NOT_IMPLEMENTED
}

bool CPhysicsCollision::IsBoxIntersectingCone(const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &truncatedCone) {
	btVector3 boxMins, boxMaxs;
	ConvertAABBToBull(boxAbsMins, boxAbsMaxs, boxMins, boxMaxs);
	btVector3 boxHalfExtents = (boxMaxs - boxMins) / 2;

	btBoxShape *box = new btBoxShape(boxHalfExtents);
	btTransform boxTrans = btTransform::getIdentity();
	boxTrans.setOrigin(boxMins + boxHalfExtents);
	
	// cone
	btScalar coneHeight = ConvertDistanceToBull(truncatedCone.h);
	btScalar coneRadius = btTan(ConvertAngleToBull(truncatedCone.theta / 2)) * coneHeight; // FIXME: Does the theta correspond to the radius or diameter of the bottom?

	btConeShape *cone = new btConeShape(coneRadius, coneHeight);

	// TODO: Find a static function to do a contact pair test

	// Cleanup
	delete box;
	delete cone;

	NOT_IMPLEMENTED
	return false;
}

static int CompareFunc (const uint16 * a, const uint16 * b) {
   return ( *a - *b );
}

static btConvexShape *LedgeToConvex(const ivpcompactledge_t *ledge) {
	btConvexShape *pConvexOut = NULL;

	// Large array of all the vertices
	const char *vertices = (const char *)ledge + ledge->c_point_offset;

	if (ledge->n_triangles > 0) {
#ifdef USE_CONVEX_TRIANGLES
		btTriangleIndexVertexArray *pMesh = new btTriangleIndexVertexArray;

		const ivpcompacttriangle_t *tris = (ivpcompacttriangle_t *)(ledge + 1); // Triangles start right after the ledge

		// TODO: We need a new way to do this! IVP surfaces store all vertices in a large pool
		// after the ledge array. We need to create a list of indices, create another list of unique
		// indices, build a vertex array from the unique indices, and convert the first list
		// of indices to refer to the vertex array made from the unique list.

		// Make an index array
		unsigned short *indices = new unsigned short[ledge->n_triangles * 3];
		int curIdx = 0;
		for (int j = 0; j < ledge->n_triangles; j++) {
			Assert(j == tris[j].tri_index);

			for (int k = 0; k < 3; k++) {
				// Make sure it isn't out of bounds
				if (curIdx < ledge->n_triangles * 3)
					indices[curIdx++] = tris[j].c_three_edges[k].start_point_index;
			}
		}

		// Find the maximum index (number of vertices)
		unsigned short maxIdx = 0;
		for (int j = 0; j < ledge->n_triangles * 3; j++) {
			if (indices[j] > maxIdx) maxIdx = indices[j];
		}

		// Convert IVP vertices
		btVector3 *vertexArray = new btVector3[maxIdx + 1];
		for (int j = 0; j <= maxIdx; j++) {
			float *ivpVert = (float *)(vertices + j * 16);
			ConvertIVPPosToBull(ivpVert, vertexArray[j]);
		}

		// Now set up the mesh
		btIndexedMesh mesh;
		mesh.m_numTriangles = ledge->n_triangles;

		mesh.m_numVertices = maxIdx;
		mesh.m_vertexBase = (unsigned char *)vertexArray;
		mesh.m_vertexStride = sizeof(btVector3);
		mesh.m_vertexType = PHY_FLOAT;

		mesh.m_triangleIndexBase = (unsigned char *)indices;
		mesh.m_triangleIndexStride = 3 * sizeof(unsigned short);

		pMesh->addIndexedMesh(mesh, PHY_SHORT); // And add it (with index type of PHY_SHORT)

		btConvexTriangleMeshShape *pShape = new btConvexTriangleMeshShape(pMesh);

		pConvexOut = pShape;
#else
        btAlignedObjectArray<btVector3> vertexes;
        btConvexHullShape *pConvex = new btConvexHullShape;
		pConvex->setMargin(CONVEX_DISTANCE_MARGIN);

		const ivpcompacttriangle_t *tris = (ivpcompacttriangle_t *)(ledge + 1);

		// This code will find all unique indexes and add them to an array. This avoids
		// adding duplicate points to the convex hull shape (triangle edges can share a vertex)
		// If you find a better way you can replace this!
		CUtlVector<uint16> indices;

		for (int j = 0; j < ledge->n_triangles; j++) 
		{
			Assert((uint)j == tris[j].tri_index); // Sanity check
			for (int k = 0; k < 3; k++) 
			{
				indices.AddToTail(tris[j].c_three_edges[k].start_point_index);
			}
		}

		indices.Sort(CompareFunc);

		for (int j = 0; j < indices.Count(); j++) 
		{
			if(j + 1 != indices.Count() && indices[j] == indices[j+1])
			{
				continue;
			}
			
			uint16 index = indices[j];
			float *ivpvert = (float *)(vertices + index * 16); // 16 is sizeof(ivp aligned vector)

			btVector3 vertex;
			ConvertIVPPosToBull(ivpvert, vertex);
			vertexes.push_back( vertex );
		}

		// If an object is smaller than the margin, it will obviously float.
		// Here we will check if the shape is too small for our default margin
		// There is a way to shrink the hull according the margin, but I don't think it's needed.
		// https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=2358
        btAlignedObjectArray<btVector3> planes;
        btGeometryUtil::getPlaneEquationsFromVertices(vertexes, planes);
        int sz = planes.size();
        for (int i=0 ; i<sz ; ++i) {
            if( planes[i][3] += pConvex->getMargin() >= 0 )
            {
                // object is too small for our desired Margin. Instead, we will give it a small one.
                pConvex->setMargin( 0.005f );
                break;
            }
        }

        // add the vertexes to the convex hull shape
        sz = vertexes.size();
        for (int i=0 ; i<sz ; ++i) {
            pConvex->addPoint(vertexes[i]);
        }

		// Optimize the convex hull
		pConvex->optimizeConvexHull();

		pConvexOut = pConvex;
#endif

		// Transfer over the ledge's user data (data from Source)
		pConvexOut->setUserIndex(ledge->client_data);
	}

	return pConvexOut;
}

static void GetAllMOPPLedges(const ivpcompactmopp_t *mopp, CUtlVector<const ivpcompactledge_t *> *vecOut) {
	char *ledge = (char *)mopp + mopp->offset_ledges + mopp->size_convex_hull;
	char *points = ledge + ((ivpcompactledge_t *)ledge)->c_point_offset;

	// Loop until we hit the vertex array
	while (ledge < points) {
		Assert(((ivpcompactledge_t *)ledge)->for_future_use == 0); // Validity check

		vecOut->AddToTail((ivpcompactledge_t *)ledge);
		ledge += ((ivpcompactledge_t *)ledge)->size_div_16 * 16;
	}
}

// A MOPP is a collection of ivpcompactledges
static CPhysCollide *LoadMOPP(void *pSolid, bool swap) {
	// Parse MOPP surface header
	//const moppsurfaceheader_t *moppSurface = (moppsurfaceheader_t *)((char *)pSolid + sizeof(collideheader_t));
	const ivpcompactmopp_t *ivpmopp = (ivpcompactmopp_t *)((char *)pSolid + sizeof(collideheader_t) + sizeof(moppsurfaceheader_t));

	if (ivpmopp->dummy != IVP_COMPACT_MOPP_ID) {
		return NULL;
	}

	CUtlVector<const ivpcompactledge_t *> ledges;
	GetAllMOPPLedges(ivpmopp, &ledges);
	DevMsg("MOPP with %d ledges\n", ledges.Count());

	btCompoundShape *pCompound = NULL;
	
	if (ledges.Count() == 1)
		pCompound = new btCompoundShape(false); // Pointless for an AABB tree if it's just one convex
	else
		pCompound = new btCompoundShape();

	CPhysCollide *pCollide = new CPhysCollide(pCompound);

	btVector3 massCenter;
	ConvertIVPPosToBull(ivpmopp->mass_center, massCenter);
	pCollide->SetMassCenter(massCenter);

	pCompound->setMargin(COLLISION_MARGIN);

	for (int i = 0; i < ledges.Count(); i++) {
		const ivpcompactledge_t *ledge = ledges[i];

		btTransform offsetTrans(btMatrix3x3::getIdentity(), -pCollide->GetMassCenter());
		pCompound->addChildShape(offsetTrans, LedgeToConvex(ledge));
	}

	return pCollide;
}

// Purpose: Recursive function that goes through the entire ledge tree and adds ledges
static void GetAllIVPSLedges(const ivpcompactledgenode_t *node, CUtlVector<const ivpcompactledge_t *> *vecOut) {
	if (!node || !vecOut) return;

	if (node->IsTerminal()) {
		vecOut->AddToTail(node->GetCompactLedge());
	} else {
		const ivpcompactledgenode_t *rs = node->GetRightSon();
		const ivpcompactledgenode_t *ls = node->GetLeftSon();
		GetAllIVPSLedges(rs, vecOut);
		GetAllIVPSLedges(ls, vecOut);
	}
}

static CPhysCollide *LoadIVPS(void *pSolid, bool swap) {
	// Parse IVP Surface header (which is right after the compact surface header)
	//const compactsurfaceheader_t *compactSurface = (compactsurfaceheader_t *)((char *)pSolid + sizeof(collideheader_t));
	const ivpcompactsurface_t *ivpsurface = (ivpcompactsurface_t *)((char *)pSolid + sizeof(collideheader_t) + sizeof(compactsurfaceheader_t));

	if (ivpsurface->dummy[2] != IVP_COMPACT_SURFACE_ID) {
		return NULL;
	}

	// Add all of the ledges up
	CUtlVector<const ivpcompactledge_t *> ledges;
	GetAllIVPSLedges((const ivpcompactledgenode_t *)((char *)ivpsurface + ivpsurface->offset_ledgetree_root), &ledges);

	btCompoundShape *pCompound = NULL;
	
	if (ledges.Count() == 1)
		pCompound = new btCompoundShape(false); // Pointless for an AABB tree if it's just one convex
	else
		pCompound = new btCompoundShape();

	CPhysCollide *pCollide = new CPhysCollide(pCompound);

	btVector3 massCenter;
	ConvertIVPPosToBull(ivpsurface->mass_center, massCenter);
	pCollide->SetMassCenter(massCenter);

	// No conversion necessary (IVP in meters and we don't need to flip any axes)
	pCollide->SetRotationInertia(btVector3(ivpsurface->rotation_inertia[0], ivpsurface->rotation_inertia[1], ivpsurface->rotation_inertia[2]));

	pCompound->setMargin(COLLISION_MARGIN);

	for (int i = 0; i < ledges.Count(); i++) {
		const ivpcompactledge_t *ledge = ledges[i];

		btTransform offsetTrans(btMatrix3x3::getIdentity(), -pCollide->GetMassCenter());
		pCompound->addChildShape(offsetTrans, LedgeToConvex(ledge));
	}

	return pCollide;
}

// Purpose: Loads and converts an ivp mesh to a bullet mesh.
void CPhysicsCollision::VCollideLoad(vcollide_t *pOutput, int solidCount, const char *pBuffer, int bufferSize, bool swap) {
	memset(pOutput, 0, sizeof(*pOutput));

	// TODO: This
	// In practice, this is never true on a Windows PC (and most likely not a linux dedicated server either)
	if (swap) {
		Warning("VCollideLoad - Abort loading, swap is true\n");
		Assert(0);
		return;
	}

	pOutput->solidCount = solidCount;
	pOutput->solids = new CPhysCollide *[solidCount];

	int position = 0;
	for (int i = 0; i < solidCount; i++) {
		// Size of this solid, excluding the size int itself
		int size = *(int *)(pBuffer + position);

		pOutput->solids[i] = (CPhysCollide *)(pBuffer + position);
		position += size + 4;

		// May fail if we're reading a corrupted file.
		Assert(position < bufferSize && position >= 0);
	}

	Assert(bufferSize - position > 0);

	// The rest of the buffer is the key values for the collision mesh
	pOutput->pKeyValues = new char[bufferSize - position];
	memcpy(pOutput->pKeyValues, pBuffer + position, bufferSize - position);

	// swap argument means byte swap - we must byte swap all of the collision shapes before loading them if true!
	// DevMsg("VPhysics: VCollideLoad with %d solids, swap is %s\n", solidCount, swap ? "true" : "false");

	// Now for the fun part:
	// We must convert all of the ivp shapes into something we can use.
	for (int i = 0; i < solidCount; i++) {
		const collideheader_t &surfaceheader = *(collideheader_t *)pOutput->solids[i];

		if (surfaceheader.vphysicsID	!= VPHYSICS_ID
		 || surfaceheader.version		!= 0x100) {
			pOutput->solids[i] = NULL;
			Warning("VCollideLoad: Skipped solid %d due to invalid id/version (magic: %d version: %d)", i+1, surfaceheader.vphysicsID, surfaceheader.version);
			continue;
		}

		CPhysCollide *pShape = NULL;

		// NOTE: modelType 0 is IVPS, 1 is (mostly unused) MOPP format
		if (surfaceheader.modelType == 0x0) {
			pShape = LoadIVPS(pOutput->solids[i], swap);
		} else if (surfaceheader.modelType == 0x1) {
			// One big use of mopps is in old map displacement data
			// The use is terribly unoptimized (each triangle is its own convex shape)
			//pShape = LoadMOPP(pOutput->solids[i], swap);

			// If we leave this as NULL, the game will use CreateVirtualMesh instead.
		} else {
			Warning("VCollideLoad: Unknown modelType %d (solid %d). Skipped!", surfaceheader.modelType, i+1);
		}

		pOutput->solids[i] = pShape;
	}
}

void CPhysicsCollision::VCollideUnload(vcollide_t *pVCollide) {
	for (int i = 0; i < pVCollide->solidCount; i++) {
		DestroyCollide(pVCollide->solids[i]);
	}

	delete [] pVCollide->solids;
	pVCollide->solids = NULL;

	delete [] pVCollide->pKeyValues;
	pVCollide->pKeyValues = NULL;
}

IVPhysicsKeyParser *CPhysicsCollision::VPhysicsKeyParserCreate(const char *pKeyData) {
	return new CPhysicsKeyParser(pKeyData);
}
//lwss - added this
IVPhysicsKeyParser *CPhysicsCollision::VPhysicsKeyParserCreate(vcollide_t *pVCollide)
{
    return new CPhysicsKeyParser( pVCollide->pKeyValues );
}
//lwss end

void CPhysicsCollision::VPhysicsKeyParserDestroy(IVPhysicsKeyParser *pParser) {
	delete (CPhysicsKeyParser *)pParser;
}

int CPhysicsCollision::CreateDebugMesh(CPhysCollide const *pCollisionModel, Vector **outVerts) {
	if (!pCollisionModel || !outVerts) return 0;

	const btCollisionShape *pShape = pCollisionModel->GetCollisionShape();
	int count = 0;

	if (pShape->isCompound()) {
		const btCompoundShape *pCompound = (btCompoundShape *)pShape;
		for (int i = 0; i < pCompound->getNumChildShapes(); i++) {
			int shapeType = pCompound->getChildShape(i)->getShapeType();

			if (shapeType == CONVEX_HULL_SHAPE_PROXYTYPE) {
				count += ((btConvexHullShape *)pCompound->getChildShape(i))->getNumVertices();
			} else if (shapeType == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE) {
				count += ((btConvexTriangleMeshShape *)pCompound->getChildShape(i))->getNumVertices();
			}
		}
		
		if (count >= 0) {
			*outVerts = new Vector[count];
			int curVert = 0;

			for (int i = 0; i < pCompound->getNumChildShapes(); i++) {
				int shapeType = pCompound->getChildShape(i)->getShapeType();

				if (shapeType == CONVEX_HULL_SHAPE_PROXYTYPE) {
					btConvexHullShape *pConvex = (btConvexHullShape *)pCompound->getChildShape(i);

					// Source requires vertices in reverse order
					for (int j = pConvex->getNumVertices()-1; j >= 0; j--) {
						btVector3 pos;
						pConvex->getVertex(j, pos);
						ConvertPosToHL(pos, (*outVerts)[curVert++]);
					}
				} else if (shapeType == CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE) {
					// FYI: Currently unsupported in convex tri meshes
					btConvexTriangleMeshShape *pConvex = (btConvexTriangleMeshShape *)pCompound->getChildShape(i);

					for (int j = pConvex->getNumVertices()-1; j >= 0; j--) {
						btVector3 pos;
						pConvex->getVertex(j, pos);
						ConvertPosToHL(pos, (*outVerts)[curVert++]);
					}
				}
			}
		}
	}

	return count;
}

void CPhysicsCollision::DestroyDebugMesh(int vertCount, Vector *outVerts) {
	delete [] outVerts;
}

ICollisionQuery *CPhysicsCollision::CreateQueryModel(CPhysCollide *pCollide) {
	return new CCollisionQuery(pCollide);
}

void CPhysicsCollision::DestroyQueryModel(ICollisionQuery *pQuery) {
	delete pQuery;
}

IPhysicsCollision *CPhysicsCollision::ThreadContextCreate() {
	return new CPhysicsCollision;
}

void CPhysicsCollision::ThreadContextDestroy(IPhysicsCollision *pThreadContext) {
	delete pThreadContext;
}

// lwss - Recreated these functions
float CPhysicsCollision::CollideGetRadius(const CPhysCollide *pCollide)
{
    const btCollisionShape *shape = pCollide->GetCollisionShape();
    if (shape->getShapeType() != SPHERE_SHAPE_PROXYTYPE)
        return 0.0f;
    btSphereShape *sphere = (btSphereShape *)shape;
    return ConvertDistanceToHL( sphere->getRadius() );
}
void* CPhysicsCollision::VCollideAllocUserData(vcollide_t *pVCollide, size_t userDataSize)
{
    void *ret = nullptr;

    if( pVCollide->pUserData )
    {
        g_pMemAlloc->Free( pVCollide->pUserData );
        pVCollide->pUserData = nullptr;
    }

    if( userDataSize )
    {
        ret = g_pMemAlloc->Alloc( userDataSize );
        pVCollide->pUserData = ret;
    }
    return ret;
}

void CPhysicsCollision::VCollideFreeUserData(vcollide_t *pVCollide)
{
    if( pVCollide->pUserData )
    {
        g_pMemAlloc->Free( pVCollide->pUserData );
        pVCollide->pUserData = nullptr;
    }
}
void CPhysicsCollision::VCollideCheck( vcollide_t *pVCollide, const char *pName )
{
    Warning("LWSS did not implement CPhysicsCollision::VCollideCheck()!\n");
}
bool CPhysicsCollision::TraceBoxAA( const Ray_t &ray, const CPhysCollide *pCollide, trace_t *ptr )
{
    Warning( "LWSS did not implement CPhysicsCollision::TraceBoxAA - it's only used in Portal!\n" );
    return false;
}

// lwss end

// BUG: Weird collisions with these, sometimes phys objs fall through the displacement mesh
// Might be a bullet issue
CPhysCollide *CPhysicsCollision::CreateVirtualMesh(const virtualmeshparams_t &params) {
	IVirtualMeshEvent *pHandler = params.pMeshEventHandler;
	if (!pHandler) return NULL;

	// TODO: if params.buildOuterHull is true, what do we do?

	virtualmeshlist_t list;
	pHandler->GetVirtualMesh(params.userData, &list);

	btTriangleIndexVertexArray *pArray = new btTriangleIndexVertexArray;

	btIndexedMesh mesh;
	mesh.m_numVertices = list.vertexCount;
	mesh.m_numTriangles = list.triangleCount;

	// Copy the array (because this is needed to exist for the lifetime of the triangle)
	unsigned short *indexArray = new unsigned short[list.indexCount];
	mesh.m_triangleIndexBase = (unsigned char *)indexArray;
	mesh.m_triangleIndexStride = 3 * sizeof(unsigned short);

	for (int i = 0; i < list.indexCount; i++) {
		indexArray[i] = list.indices[i];
	}

	btVector3 *vertexArray = new btVector3[list.vertexCount];
	mesh.m_vertexBase = (unsigned char *)vertexArray;
	mesh.m_vertexStride = sizeof(btVector3);

	for (int i = 0; i < list.vertexCount; i++) {
		ConvertPosToBull(list.pVerts[i], vertexArray[i]);
	}

	pArray->addIndexedMesh(mesh, PHY_SHORT);

	btBvhTriangleMeshShape *bull = new btBvhTriangleMeshShape(pArray, true);
	bull->setMargin(COLLISION_MARGIN);

	btTriangleInfoMap *pMap = new btTriangleInfoMap;
	btGenerateInternalEdgeInfo(bull, pMap);

	return new CPhysCollide(bull);
}

bool CPhysicsCollision::SupportsVirtualMesh() {
	return true;
}

void CPhysicsCollision::OutputDebugInfo(const CPhysCollide *pCollide) {
	Assert(pCollide);

	const btCollisionShape *pShape = pCollide->GetCollisionShape();

	Msg("Type: %s\n", pShape->getName());
	if (pShape->isCompound()) {
		btCompoundShape *pCompound = (btCompoundShape *)pShape;
		Msg("Num child shapes: %d\n", pCompound->getNumChildShapes());
	} else if (pShape->isConvex()) {
		if (pShape->getShapeType() == CONVEX_HULL_SHAPE_PROXYTYPE) {
			btConvexHullShape *pConvex = (btConvexHullShape *)pShape;
			Msg("Margin: %f\n", pConvex->getMargin());
			Msg("Num points: %d\n", pConvex->getNumPoints());
		}
	}
}

// What is this?
// Returns 0 in valve vphysics as well.
unsigned int CPhysicsCollision::ReadStat(int statID) {
	NOT_IMPLEMENTED
	return 0;
}

CPhysicsCollision g_PhysicsCollision;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(CPhysicsCollision, IPhysicsCollision, VPHYSICS_COLLISION_INTERFACE_VERSION, g_PhysicsCollision);

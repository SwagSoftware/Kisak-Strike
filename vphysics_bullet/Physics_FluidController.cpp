#include "StdAfx.h"

#include "convert.h"
#include "Physics_FluidController.h"
#include "Physics_Object.h"
#include "Physics_Environment.h"
#include "Physics_SurfaceProps.h"
#include "Physics_Collision.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

/********************************
* CLASS CPhysicsFluidCallback
********************************/

class CPhysicsFluidCallback : public btGhostObjectCallback {
	public:
		CPhysicsFluidCallback(CPhysicsFluidController *pController) {
			m_pController = pController;
		}

		void addedOverlappingObject(btCollisionObject *pObject) {
			CPhysicsObject *pPhys = (CPhysicsObject *)pObject->getUserPointer();
			if (!pPhys) return;

			m_pController->ObjectAdded(pPhys);
		}

		void removedOverlappingObject(btCollisionObject *pObject) {
			CPhysicsObject *pPhys = (CPhysicsObject *)pObject->getUserPointer();
			if (!pPhys) return;

			m_pController->ObjectRemoved(pPhys);
		}

	private:
		CPhysicsFluidController *m_pController;
};

/********************************
* CLASS CPhysicsFluidController
********************************/

CPhysicsFluidController::CPhysicsFluidController(CPhysicsEnvironment *pEnv, CPhysicsObject *pFluidObject, fluidparams_t *pParams) {
	m_pEnv = pEnv;
	m_pGameData = NULL;
	m_iContents = 0;
	m_vSurfacePlane = Vector4D(0, 0, 0, 0);

	if (pParams) {
		m_pGameData = pParams->pGameData;
		m_iContents = pParams->contents;
		m_vSurfacePlane = pParams->surfacePlane;
		ConvertPosToBull(pParams->currentVelocity, m_currentVelocity);
	}

	int matIndex = pFluidObject->GetMaterialIndex();
	surfacedata_t *pSurface = g_SurfaceDatabase.GetSurfaceData(matIndex);
	if (pSurface) {
		m_fDensity = pSurface->physics.density;
	}

	pFluidObject->EnableCollisions(false);
	pFluidObject->SetContents(m_iContents);
	pFluidObject->SetFluidController(this);

	m_pCallback = new CPhysicsFluidCallback(this);

	m_pGhostObject = new btGhostObject;
	m_pGhostObject->setUserPointer(pFluidObject);
	m_pGhostObject->setCallback(m_pCallback);
	m_pGhostObject->setCollisionShape(pFluidObject->GetObject()->getCollisionShape());
	m_pGhostObject->setWorldTransform(pFluidObject->GetObject()->getWorldTransform());
	m_pGhostObject->setCollisionFlags(m_pGhostObject->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE | btCollisionObject::CF_STATIC_OBJECT | btCollisionObject::CF_DISABLE_VISUALIZE_OBJECT);
	m_pEnv->GetBulletEnvironment()->addCollisionObject(m_pGhostObject, COLGROUP_WORLD, ~COLGROUP_WORLD);
}

CPhysicsFluidController::~CPhysicsFluidController() {
	m_pEnv->GetBulletEnvironment()->removeCollisionObject(m_pGhostObject);
	delete m_pGhostObject;
	delete m_pCallback;
}

void CPhysicsFluidController::WakeAllSleepingObjects() {
	int count = m_pGhostObject->getNumOverlappingObjects();
	for (int i = 0; i < count; i++) {
		btRigidBody *body = btRigidBody::upcast(m_pGhostObject->getOverlappingObject(i));
		if (!body)
			continue;

		body->activate(true);
	}
}

void CPhysicsFluidController::SetGameData(void *pGameData) {
	m_pGameData = pGameData;
}

void *CPhysicsFluidController::GetGameData() const {
	return m_pGameData;
}

void CPhysicsFluidController::GetSurfacePlane(Vector *pNormal, float *pDist) const {
	if (!pNormal && !pDist) return;

	if (pNormal)
		*pNormal = m_vSurfacePlane.AsVector3D();

	if (pDist)
		*pDist = m_vSurfacePlane.w;
}

float CPhysicsFluidController::GetDensity() const {
	return m_fDensity;
}

int	CPhysicsFluidController::GetContents() const {
	return m_iContents;
}

static btVector3 calcConvexCenter(btConvexHullShape *pShape, btVector3 &planePos, btVector3 &planeNorm) {
	// Basic average
	btVector3 sum(0, 0, 0);

	if (pShape->getNumPoints() > 0) {
		for (int i = 0; i < pShape->getNumPoints(); i++) {
			btVector3 point = pShape->getScaledPoint(i);
			point -= planePos;

			// Only add the point if it's submerged
			if (point.dot(planeNorm) < 0)
				sum += pShape->getScaledPoint(i);
		}

		sum /= static_cast<btScalar>(pShape->getNumPoints());
	}

	return sum;
}

// Find the object's center of buoyancy
// This would be the center of all submerged points
// You can bisect the object by the plane of the water surface to get all points
// submerged, then calculate the center of all those points
// planePos: World space plane position
// planeNorm: World space plane normal
static btVector3 calculateBuoyantCenter(btRigidBody *pBody, btVector3 &planePos, btVector3 &planeNorm) {
	btVector3 center(0, 0, 0);

	btVector3 relPlanePos = pBody->getWorldTransform().inverse() * planePos;
	btVector3 relNorm = quatRotate(pBody->getWorldTransform().getRotation().inverse(), planeNorm);

	btCollisionShape *pShape = pBody->getCollisionShape();
	if (pShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE) {
		btCompoundShape *pCompound = (btCompoundShape *)pShape;
		for (int i = 0; i < pCompound->getNumChildShapes(); i++) {
			btCollisionShape *pChild = pCompound->getChildShape(i);
			if (pChild->getShapeType() == CONVEX_HULL_SHAPE_PROXYTYPE) {
				center += calcConvexCenter((btConvexHullShape *)pChild, relPlanePos, relNorm);
			}
		}

		if (pCompound->getNumChildShapes() > 0)
			center /= static_cast<btScalar>(pCompound->getNumChildShapes());
	}

	return center;
}

// TODO: Refactor this code to be less messy.
void CPhysicsFluidController::Tick(float dt) {
	// TODO: Buoyancy calculation
	int numObjects = m_pGhostObject->getNumOverlappingObjects();
	for (int i = 0; i < numObjects; i++) {
		btRigidBody *body = btRigidBody::upcast(m_pGhostObject->getOverlappingObject(i));
		Assert(body);
		if (!body) continue;

		CPhysicsObject *pObject = (CPhysicsObject *)body->getUserPointer();
		Assert(pObject);

		// Find the surface plane's world pos (center at the very top)
		btVector3 surfPos = m_pGhostObject->getWorldTransform().getOrigin();
		btVector3 surfNorm;
		ConvertDirectionToBull(m_vSurfacePlane.AsVector3D(), surfNorm);

		// btScalar fluidLen = ConvertDistanceToBull(m_vSurfacePlane.w);

		btVector3 omins, omaxs;
		m_pGhostObject->getCollisionShape()->getAabb(m_pGhostObject->getWorldTransform(), omins, omaxs);
		btScalar height = omaxs.y() - omins.y();

		surfPos += surfNorm * (height / 2);

		btVector3 center = calculateBuoyantCenter(body, surfPos, surfNorm);
		center = body->getWorldTransform() * center; // Convert back into world space

		btVector3 offset = center - surfPos;
		btScalar submerged = surfNorm.dot(offset);

#ifdef _DEBUG
		IVPhysicsDebugOverlay *pOverlay = m_pEnv->GetDebugOverlay();
		if (pOverlay) {
			Vector pos;
			ConvertPosToHL(surfPos, pos);
			pOverlay->AddBoxOverlay(pos, Vector(-8,-8,-8), Vector(8,8,8), QAngle(0, 0, 0), 255, 0, 0, 255, 0.f);
			pOverlay->AddLineOverlay(pos, pos + m_vSurfacePlane.AsVector3D() * 32, 255, 0, 0, false, 0.f);

			ConvertPosToHL(center, pos);
			pOverlay->AddBoxOverlay(pos, Vector(-8,-8,-8), Vector(8,8,8), QAngle(0, 0, 0), 0, 0, 255, 255, 0.f);

			if (submerged < 0) {
				pOverlay->AddTextOverlay(pos, 0.f, "submerged %+04.2f", submerged);
			}
		}
#endif

		if (submerged < 0) {
			btVector3 mins, maxs;
			body->getAabb(mins, maxs);

			// Calc the volume coefficient
			btScalar dist = -submerged;
			float p = clamp(dist / 1.2f, 0.f, 1.f);
			btScalar vol = p * pObject->GetVolume();

			// TODO: Need a way to calculate this properly
			// Force should be exactly equal to -gravity when submerged distance is 0
			// density units kg/m^3
			btVector3 force = (m_fDensity * -body->getGravity() * vol) * pObject->GetBuoyancyRatio();

			btVector3 relPos = center - body->getWorldTransform().getOrigin();
			body->applyForce(force, relPos);
		}

		// Old code that actually works better
		/*
		btVector3 mins, maxs, omins, omaxs;
		body->getAabb(mins, maxs);
		m_pGhostObject->getCollisionShape()->getAabb(m_pGhostObject->getWorldTransform(), omins, omaxs);

		float height = maxs.y() - mins.y(); // If the plane for the surface can be non-upwards I'm going to murder something
		//float height = abs(ConvertDistanceToBull(m_vSurfacePlane.w));
		float dist = omaxs.y() - mins.y(); // Distance between top of water and bottom of object (how much of object is in water)
		float p = clamp(dist / height, 0.0f, 1.0f);
		float vol = (pObject->GetVolume() * p); // / 64; // Submerged volume

		// TODO: We need to calculate this force at several points on the object (How do we determine what points?).
		// Maybe have the points determined by the extents of the shape at 4 directions parallel to our surface
		// Simulate buoyant force per convex or just on the compound?

		// Simulate buoyant force per point submerged
		// Divide it over all of the points that are submerged

		// IVP calculates this force per triangle
		btVector3 force = (m_fDensity * -body->getGravity() * vol) * pObject->GetBuoyancyRatio();
		body->applyCentralForce(force);

		// Damping
		// FIXME: Damping would be way too much for an object only partially touching our surface (ex. giant fucking bridge with like 1 pixel in the water)
		body->setLinearVelocity(body->getLinearVelocity() * (1.0f - (0.75f * dt)));
		body->setAngularVelocity(body->getAngularVelocity() * (1.0f - (0.75f * dt)));
		*/
	}
}

// UNEXPOSED
void CPhysicsFluidController::ObjectAdded(CPhysicsObject *pObject) {
	m_pEnv->HandleFluidStartTouch(this, pObject);
}

// UNEXPOSED
void CPhysicsFluidController::ObjectRemoved(CPhysicsObject *pObject) {
	// Don't send the callback on objects that are being removed
	if (!pObject->IsBeingRemoved())
		m_pEnv->HandleFluidEndTouch(this, pObject);
}

void CPhysicsFluidController::TransferToEnvironment(CPhysicsEnvironment *pDest) {

}

/************************
* CREATION FUNCTIONS
************************/

CPhysicsFluidController *CreateFluidController(CPhysicsEnvironment *pEnv, CPhysicsObject *pFluidObject, fluidparams_t *pParams) {
	if (!pEnv || !pFluidObject) return NULL;
	CPhysicsFluidController *pFluid = new CPhysicsFluidController(pEnv, pFluidObject, pParams);
	return pFluid;
}
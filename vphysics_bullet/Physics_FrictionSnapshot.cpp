#include "StdAfx.h"

#include "Physics_FrictionSnapshot.h"
#include "Physics_Object.h"
#include "Physics_Environment.h"

#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

CPhysicsFrictionSnapshot::CPhysicsFrictionSnapshot(CPhysicsObject *pObject) {
	m_pObject = pObject;
	m_iCurContactPoint = 0;
	m_iCurManifold = 0;

	CPhysicsEnvironment *pEnv = pObject->GetVPhysicsEnvironment();
	btRigidBody *pBody = pObject->GetObject();
	int numManifolds = pEnv->GetBulletEnvironment()->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold *pManifold = pEnv->GetBulletEnvironment()->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject *pObjA = pManifold->getBody0();
		const btCollisionObject *pObjB = pManifold->getBody1();

		if (pManifold->getNumContacts() <= 0)
			continue;

		if (pObjA == pBody || pObjB == pBody) {
			m_manifolds.AddToTail(pManifold);
		}
	}
}

CPhysicsFrictionSnapshot::~CPhysicsFrictionSnapshot() {
	m_manifolds.RemoveAll();
}

bool CPhysicsFrictionSnapshot::IsValid() {
	return m_iCurManifold < m_manifolds.Count();
}

IPhysicsObject *CPhysicsFrictionSnapshot::GetObject(int index) {
	const btCollisionObject *colObjA = m_manifolds[m_iCurManifold]->getBody0();
	const btCollisionObject *colObjB = m_manifolds[m_iCurManifold]->getBody1();
	const CPhysicsObject *pObjA = (CPhysicsObject *)colObjA->getUserPointer();
	const CPhysicsObject *pObjB = (CPhysicsObject *)colObjB->getUserPointer();

	// index 0 is this object, 1 is other object
	if (index == 0)
		return (pObjA == m_pObject) ? (IPhysicsObject *)pObjA : (IPhysicsObject *)pObjB;
	else
		return (pObjA != m_pObject) ? (IPhysicsObject *)pObjA : (IPhysicsObject *)pObjB;
}

int CPhysicsFrictionSnapshot::GetMaterial(int index) {
	const btCollisionObject *colObjA = m_manifolds[m_iCurManifold]->getBody0();
	const btCollisionObject *colObjB = m_manifolds[m_iCurManifold]->getBody1();
	const CPhysicsObject *pObjA = (CPhysicsObject *)colObjA->getUserPointer();
	const CPhysicsObject *pObjB = (CPhysicsObject *)colObjB->getUserPointer();

	// index 0 is this object, 1 is other object
	if (index == 0)
		return (pObjA == m_pObject) ? pObjA->GetMaterialIndex() : pObjB->GetMaterialIndex();
	else
		return (pObjA != m_pObject) ? pObjA->GetMaterialIndex() : pObjB->GetMaterialIndex();
}

void CPhysicsFrictionSnapshot::GetContactPoint(Vector &out) {
	btManifoldPoint bullManifoldPoint = m_manifolds[m_iCurManifold]->getContactPoint(m_iCurContactPoint);

	btVector3 bullPos = bullManifoldPoint.getPositionWorldOnA();
	if (m_pObject->GetObject() == m_manifolds[m_iCurManifold]->getBody1()) {
		bullPos = bullManifoldPoint.getPositionWorldOnB(); // Our object is object B, so get the pos for B
	}

	ConvertPosToHL(bullPos, out);
}

void CPhysicsFrictionSnapshot::GetSurfaceNormal(Vector &out) {
	btManifoldPoint bullManifoldPoint = m_manifolds[m_iCurManifold]->getContactPoint(m_iCurContactPoint);
	btVector3 norm = bullManifoldPoint.m_normalWorldOnB;

	// Flip the normal so it's world on A (needs to be pointed away from m_pObject)
	if (m_pObject->GetObject() == m_manifolds[m_iCurManifold]->getBody0())
		norm *= -1;

	ConvertDirectionToHL(norm, out); // The game expects the normal to point away from our object
}

float CPhysicsFrictionSnapshot::GetNormalForce() {
	btManifoldPoint bullManifoldPoint = m_manifolds[m_iCurManifold]->getContactPoint(m_iCurContactPoint);
	return BULL2HL(bullManifoldPoint.m_appliedImpulse); // Force impulse to HL (kg * m/s) -> (kg * in/s)
}

float CPhysicsFrictionSnapshot::GetEnergyAbsorbed() {
	NOT_IMPLEMENTED
	return 0;
}

void CPhysicsFrictionSnapshot::RecomputeFriction() {
	NOT_IMPLEMENTED
}

void CPhysicsFrictionSnapshot::ClearFrictionForce() {
	NOT_IMPLEMENTED
}

void CPhysicsFrictionSnapshot::MarkContactForDelete() {
	NOT_IMPLEMENTED
}

void CPhysicsFrictionSnapshot::DeleteAllMarkedContacts(bool wakeObjects) {
	NOT_IMPLEMENTED
}

void CPhysicsFrictionSnapshot::NextFrictionData() {
	m_iCurContactPoint++;
	if (m_iCurContactPoint >= m_manifolds[m_iCurManifold]->getNumContacts()) {
		m_iCurManifold++;
		m_iCurContactPoint = 0;
	}
}

float CPhysicsFrictionSnapshot::GetFrictionCoefficient() {
	btManifoldPoint bullManifoldPoint = m_manifolds[m_iCurManifold]->getContactPoint(m_iCurContactPoint);
	return bullManifoldPoint.m_combinedFriction;
}

/***********************
* CREATION FUNCTIONS
***********************/

CPhysicsFrictionSnapshot *CreateFrictionSnapshot(CPhysicsObject *pObject) {
	if (!pObject) return NULL;

	return new CPhysicsFrictionSnapshot(pObject);
}
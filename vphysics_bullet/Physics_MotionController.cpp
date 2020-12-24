#include "StdAfx.h"

#include "Physics_MotionController.h"
#include "Physics_Object.h"
#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

IPhysicsMotionController *CreateMotionController(CPhysicsEnvironment *pEnv, IMotionEvent *pHandler) {
	if (!pEnv) return NULL;
	return new CPhysicsMotionController(pHandler, pEnv);
}

/***********************************
* CLASS CPhysicsMotionController
***********************************/

// This class can totally be moved back to game code.

CPhysicsMotionController::CPhysicsMotionController(IMotionEvent *pHandler, CPhysicsEnvironment *pEnv) {
	m_handler = pHandler;
	m_pEnv = pEnv;
}

CPhysicsMotionController::~CPhysicsMotionController() {
	for (int i = m_objectList.Count()-1; i >= 0; i--) {
		CPhysicsMotionController::DetachObject(m_objectList[i]);
	}
}

void CPhysicsMotionController::Tick(float deltaTime) {
	if (!m_handler) return;

	for (int i = 0; i < m_objectList.Count(); i++) {
		Vector speed;
		AngularImpulse rot;
		
		CPhysicsObject *pObject = m_objectList[i];
		IMotionEvent::simresult_e ret = m_handler->Simulate(this, pObject, deltaTime, speed, rot);

		speed *= deltaTime;
		rot *= deltaTime;

		Vector curVel, curAngVel;
		pObject->GetVelocity(&curVel, &curAngVel);

		switch (ret) {
			case IMotionEvent::SIM_NOTHING: {
				break;
			}
			case IMotionEvent::SIM_LOCAL_ACCELERATION: {
				// Convert velocity to world space
				Vector newVel;
				pObject->LocalToWorldVector(&newVel, speed);

				pObject->AddVelocity(&newVel, &rot); // Rotation already in local space.
				break;
			}
			case IMotionEvent::SIM_LOCAL_FORCE: {
				Vector newVel;
				pObject->LocalToWorldVector(&newVel, speed);

				pObject->ApplyForceCenter(newVel);
				pObject->ApplyTorqueCenter(rot);
				break;
			}
			case IMotionEvent::SIM_GLOBAL_ACCELERATION: {
				pObject->AddVelocity(&speed, &rot);
				break;
			}
			case IMotionEvent::SIM_GLOBAL_FORCE: {
				pObject->ApplyForceCenter(speed);
				pObject->ApplyTorqueCenter(rot);
				break;
			}
			default: {
				DevWarning("VPhysics: Invalid motion controller event type returned (%d)\n", ret);
			}
		}
	}
}

void CPhysicsMotionController::ObjectDestroyed(CPhysicsObject *pObject) {
	m_objectList.FindAndRemove(pObject);
}

void CPhysicsMotionController::SetEventHandler(IMotionEvent *handler) {
	m_handler = handler;
}

void CPhysicsMotionController::AttachObject(IPhysicsObject *pObject, bool checkIfAlreadyAttached) {
	Assert(pObject);
	if (!pObject || pObject->IsStatic()) return;

	CPhysicsObject *pPhys = (CPhysicsObject *)pObject;

	if (m_objectList.Find(pPhys) != -1 && checkIfAlreadyAttached)
		return;

	pPhys->AttachedToController(this);
	pPhys->AttachEventListener(this);
	m_objectList.AddToTail(pPhys);
}

void CPhysicsMotionController::DetachObject(IPhysicsObject *pObject) {
	CPhysicsObject *pPhys = (CPhysicsObject *)pObject;

	int index = m_objectList.Find(pPhys);
	if (!m_objectList.IsValidIndex(index)) return;

	pPhys->DetachedFromController(this);
	pPhys->DetachEventListener(this);
	m_objectList.Remove(index);
}

int CPhysicsMotionController::CountObjects() {
	return m_objectList.Count();
}

void CPhysicsMotionController::GetObjects(IPhysicsObject **pObjectList) {
	if (!pObjectList) return;

	for (int i = 0; i < m_objectList.Count(); i++) {
		pObjectList[i] = (IPhysicsObject *)m_objectList[i];
	}
}

void CPhysicsMotionController::ClearObjects() {
	m_objectList.Purge();
}

void CPhysicsMotionController::WakeObjects() {
	for (int i = 0; i < m_objectList.Count(); i++) {
		m_objectList[i]->GetObject()->setActivationState(ACTIVE_TAG);
	}
}

void CPhysicsMotionController::SetPriority(priority_t priority) {
	// IVP Controllers had a priority. Since bullet doesn't have controllers, this function is useless.
}

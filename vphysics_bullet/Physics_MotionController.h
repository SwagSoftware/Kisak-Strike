#ifndef PHYSICS_MOTIONCONTROLLER_H
#define PHYSICS_MOTIONCONTROLLER_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

#include "IController.h"

#include "Physics_Object.h"

class CPhysicsEnvironment;

class CPhysicsMotionController : public IController, public IPhysicsMotionController, public IObjectEventListener
{
	public:
										CPhysicsMotionController(IMotionEvent *pHandler, CPhysicsEnvironment *pEnv);
										~CPhysicsMotionController();

		void							SetEventHandler(IMotionEvent *handler);
		void							AttachObject(IPhysicsObject *pObject, bool checkIfAlreadyAttached);
		void							DetachObject(IPhysicsObject *pObject);

		int								CountObjects();
		void							GetObjects(IPhysicsObject **pObjectList);
		void							ClearObjects();
		void							WakeObjects();

		void							SetPriority(priority_t priority);
	public:
		void							Tick(float deltaTime);
		void							ObjectDestroyed(CPhysicsObject *pObject);

	private:
		IMotionEvent *					m_handler;
		CUtlVector<CPhysicsObject *>	m_objectList;
		CPhysicsEnvironment *			m_pEnv;

		int								m_priority;
};

IPhysicsMotionController *CreateMotionController(CPhysicsEnvironment *pEnv, IMotionEvent *pHandler);

#endif // PHYSICS_MOTIONCONTROLLER_H

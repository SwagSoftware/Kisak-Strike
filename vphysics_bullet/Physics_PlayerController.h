#ifndef PHYSICS_PLAYERCONTROLLER_H
#define PHYSICS_PLAYERCONTROLLER_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

#include <vphysics/player_controller.h>
#include "IController.h"

#include "Physics_Object.h"

class CPhysicsEnvironment;
class CPhysicsObject;
class IPhysicsObject;

class CPlayerControllerEventListener;

class CPlayerController : public IController, public IPhysicsPlayerController, public IObjectEventListener
{
	public:
		CPlayerController(CPhysicsEnvironment *pEnv, CPhysicsObject *pObject);
		~CPlayerController();

        //lwss add missing func
        // returns bitfield e.g. 0 (no contacts), 1 (has physics contact), 2 (contact matching nGameFlags), 3 (both 1 & 2)
        virtual uint32 GetContactState( uint16 nGameFlags )
        {
            return 0;
        }
        //lwss end
		void							Update(const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground);
		void							SetEventHandler(IPhysicsPlayerControllerEvent *handler);
		bool							IsInContact();
		void							MaxSpeed(const Vector &maxVelocity);

		void							SetObject(IPhysicsObject *pObject);
		int								GetShadowPosition(Vector *position, QAngle *angles);
		void							StepUp(float height);
		void							Jump();
		void							GetShadowVelocity(Vector *velocity);
		IPhysicsObject *				GetObject();
		void							GetLastImpulse(Vector *pOut);

		void							SetPushMassLimit(float maxPushMass);
		void							SetPushSpeedLimit(float maxPushSpeed);
	
		float							GetPushMassLimit();
		float							GetPushSpeedLimit();
		bool							WasFrozen();
	
		// Unexposed functions
	public:
		CPhysicsObject *				GetGroundObject();

		void							Tick(float deltaTime);
		void							ObjectDestroyed(CPhysicsObject *pObject);

	private:
		void							AttachObject();
		void							DetachObject();
		bool							TryTeleportObject();

		void							CalculateVelocity(float dt);

		bool							m_enable;

		bool							m_onground;
		CPhysicsObject *				m_pGround;
		btVector3						m_groundPos;

		CPhysicsObject *				m_pObject;
		CPhysicsEnvironment *			m_pEnv;
		btVector3						m_saveRot;
		IPhysicsPlayerControllerEvent *	m_handler;
		float							m_maxDeltaPosition;
		float							m_dampFactor;
		float							m_secondsToArrival;
		btVector3						m_maxSpeed; // Maximum acceleration speed.
		btVector3						m_currentSpeed;
		btVector3						m_lastImpulse;
		btVector3						m_inputVelocity;

		float							m_pushMassLimit;
		float							m_pushSpeedLimit;

		btVector3						m_targetPosition;
		btVector3						m_maxVelocity;

		int								m_ticksSinceUpdate;
};

void ComputeController(btVector3 &currentSpeed, const btVector3 &delta, const btVector3 &maxSpeed, float scaleDelta, float damping, btVector3 *accelOut = NULL);

CPlayerController *CreatePlayerController(CPhysicsEnvironment *pEnv, IPhysicsObject *pObject);

#endif // PHYSICS_PLAYERCONTROLLER_H

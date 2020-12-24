#include "StdAfx.h"

#include "Physics_PlayerController.h"
#include "Physics_Object.h"
#include "Physics_Environment.h"
#include "convert.h"
#include "miscmath.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

void ComputeController(btVector3 &currentSpeed, const btVector3 &delta, const btVector3 &maxSpeed, float scaleDelta, float damping, btVector3 *accelOut) {
	// Timestep scale
	btVector3 acceleration = delta * scaleDelta;

	if (currentSpeed.fuzzyZero() && !currentSpeed.isZero()) {
		currentSpeed.setZero();
	}
	acceleration += currentSpeed * -damping;

	// Clamp the acceleration to max speed
	for (int i = 2; i >= 0; i--) {
		if (fabs(acceleration[i]) < maxSpeed[i]) continue;
		acceleration[i] = (acceleration[i] < 0) ? -maxSpeed[i] : maxSpeed[i];
	}

	currentSpeed += acceleration;

	if (accelOut) {
		*accelOut = acceleration;
	}
}

/****************************
* CLASS CPlayerController
****************************/

CPlayerController::CPlayerController(CPhysicsEnvironment *pEnv, CPhysicsObject *pObject) {
	m_pObject = pObject;
	m_pGround = NULL;
	m_pEnv = pEnv;
	m_handler = NULL;
	m_maxDeltaPosition = ConvertDistanceToBull(24);
	m_dampFactor = 1.f;
	m_ticksSinceUpdate = 0;
	m_lastImpulse = btVector3(0, 0, 0);
	m_secondsToArrival = 0;

	AttachObject();
}

CPlayerController::~CPlayerController() {
	DetachObject();
}

// FIXME: Jumping does not work because as soon as the player leaves the object, the target position delta is exactly
// zero and his velocity gets completely emptied!
void CPlayerController::Update(const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *pGround) {
	btVector3 bullTargetPosition, bullMaxVelocity;

	ConvertPosToBull(position, bullTargetPosition);
	ConvertPosToBull(velocity, bullMaxVelocity);

	// Reset the ticks since update counter
	m_ticksSinceUpdate = 0;

	// If the targets haven't changed, abort.
	if (bullMaxVelocity.distance2(m_maxVelocity) < FLT_EPSILON && bullTargetPosition.distance2(m_targetPosition) < FLT_EPSILON) {
		return;
	}

	// FIXME: If we're walking on a physics object, the game's input position DOES NOT FACTOR IN the base velocity of the
	// physics object!
	// Target position is just the target velocity integrated into our current position via the timestep
	m_targetPosition = bullTargetPosition;
	m_maxVelocity = bullMaxVelocity;

	// FYI: The onground stuff includes any props we may be standing on as well as the world.
	// The ground object is non-NULL only if it's significantly heavier than our object ("Rideable physics" > our mass * 2)
	m_onground = onground;

	m_enable = true;
	if (velocity.LengthSqr() <= 0.1f) {
		m_enable = false; // No input velocity, just go where physics takes you
		pGround = NULL;
	} else {
		MaxSpeed(velocity);
	}
	
	ConvertPosToBull(velocity, m_inputVelocity);

	m_secondsToArrival = secondsToArrival;

	// Detach ourselves from any existing ground
	if (m_pGround)
		m_pGround->DetachEventListener(this);

	m_pGround = (CPhysicsObject *)pGround;

	if (m_pGround) {
		// Attach ourself to the ground so we can listen to see if it gets destroyed
		m_pGround->AttachEventListener(this);

		// Where we are relative to the ground
		m_groundPos = m_pGround->GetObject()->getWorldTransform().inverse() * m_targetPosition;
	}
}

void CPlayerController::SetEventHandler(IPhysicsPlayerControllerEvent *handler) {
	m_handler = handler;
}

bool CPlayerController::IsInContact() {
	btDispatcher *pDispatcher = m_pEnv->GetBulletEnvironment()->getDispatcher();

	int numManifolds = pDispatcher->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold *contactManifold = pDispatcher->getManifoldByIndexInternal(i);
		const btCollisionObject *obA = contactManifold->getBody0();
		const btCollisionObject *obB = contactManifold->getBody1();
		CPhysicsObject *pPhysUs = NULL;
		CPhysicsObject *pPhysOther = NULL;

		if (contactManifold->getNumContacts() > 0 && (obA == m_pObject->GetObject() || obB == m_pObject->GetObject())) {
			if (obA == m_pObject->GetObject()) {
				pPhysUs = (CPhysicsObject *)obA->getUserPointer();
				pPhysOther = (CPhysicsObject *)obB->getUserPointer();
			} else if (obB == m_pObject->GetObject()) {
				pPhysUs = (CPhysicsObject *)obB->getUserPointer();
				pPhysOther = (CPhysicsObject *)obA->getUserPointer();
			}

			// If it's static or controlled by the game
			if (pPhysOther->IsStatic() || !pPhysOther->IsMotionEnabled() || (pPhysOther->GetCallbackFlags() & CALLBACK_SHADOW_COLLISION))
				continue;

			return true;
		}
	}

	return false;
}

// Purpose: Calculate the maximum speed we can accelerate.
void CPlayerController::MaxSpeed(const Vector &hlMaxVelocity) {
	btVector3 maxVel;
	ConvertPosToBull(hlMaxVelocity, maxVel);
	btVector3 available = maxVel;
	
	float maxVelLen = maxVel.length();
	maxVel.normalize();
	
	// Only if we're headed in the same direction as maxVelocity
	float dot = maxVel.dot(m_pObject->GetObject()->getLinearVelocity()); // Magnitude of our speed in the same direction as maxVel
	if (dot > 0) {
		maxVel *= dot * maxVelLen; // maxVel(normalized) *= dot(maxVel(norm), linVel) * len(maxVel)
		available -= maxVel; // Now subtract the magnitude of our current speed along the maxVelocity vector
	}
	
	m_maxSpeed = available.absolute();
}

// Called when the game wants to swap hulls (such as from standing to crouching)
void CPlayerController::SetObject(IPhysicsObject *pObject) {
	if (pObject == m_pObject)
		return;

	// HACK: Freeze our current object (so it doesn't fall through the world and create nastiness when the AABB overflows)
	m_pObject->EnableMotion(false);

	DetachObject();
	m_pObject = (CPhysicsObject *)pObject;
	AttachObject();

	// Enable motion for our new object (which was probably previously frozen by the above code)
	m_pObject->EnableMotion(true);
}

// Amazing! It absolutely does not matter what values we return here (except for the return value)
// The one implementation in the 2013 SDK that calls this discards the position we return!
// Also the angles parameter is unused because our controller cannot rotate
int CPlayerController::GetShadowPosition(Vector *position, QAngle *angles) {
	btRigidBody *pObject = m_pObject->GetObject();

	btTransform transform;
	((btMassCenterMotionState *)pObject->getMotionState())->getGraphicTransform(transform);

	if (position) ConvertPosToHL(transform.getOrigin(), *position);
	if (angles) ConvertRotationToHL(transform.getBasis(), *angles);

	// Yep. We're returning a variable totally unrelated to the shadow's position.
	// Returns whether the physics object was updated this frame or not
	return 1;
}

void CPlayerController::GetShadowVelocity(Vector *velocity) {
	if (!velocity) return;

	btRigidBody *body = m_pObject->GetObject();
	btVector3 linVel = body->getLinearVelocity();

	// Velocity is relative to the ground velocity
	if (m_pGround) {
		linVel -= m_pGround->GetObject()->getVelocityInLocalPoint(m_groundPos);
	}

	ConvertPosToHL(linVel, *velocity);
}

void CPlayerController::StepUp(float height) {
	btVector3 step;
	ConvertPosToBull(Vector(0, 0, height), step);

	btRigidBody *pObject = m_pObject->GetObject();
	btTransform transform = pObject->getWorldTransform();
	transform.setOrigin(transform.getOrigin() + step);

	pObject->getMotionState()->setWorldTransform(transform);
}

void CPlayerController::Jump() {
	// This function does absolutely nothing!
	return;
}

IPhysicsObject *CPlayerController::GetObject() {
	return m_pObject;
}

void CPlayerController::GetLastImpulse(Vector *pOut) {
	if (!pOut) return;

	ConvertForceImpulseToHL(m_lastImpulse, *pOut);
}

// Purpose: Loop through all of our contact points and see if we're standing on ground anywhere
// Returns NULL if we're not standing on ground or if we're standing on a static/frozen object (or game physics object)
CPhysicsObject *CPlayerController::GetGroundObject() {
	btDispatcher *pDispatcher = m_pEnv->GetBulletEnvironment()->getDispatcher();

	// Loop through the collision pair manifolds
	int numManifolds = pDispatcher->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold *pManifold = pDispatcher->getManifoldByIndexInternal(i);
		if (pManifold->getNumContacts() <= 0)
			continue;

		const btCollisionObject *objA = pManifold->getBody0();
		const btCollisionObject *objB = pManifold->getBody1();

		// Skip if one object is static/kinematic
		if (objA->isStaticOrKinematicObject() || objB->isStaticOrKinematicObject())
			continue;

		CPhysicsObject *pPhysA = (CPhysicsObject *)objA->getUserPointer();
		CPhysicsObject *pPhysB = (CPhysicsObject *)objB->getUserPointer();

		// Collision that involves us!
		if (objA == m_pObject->GetObject() || objB == m_pObject->GetObject()) {
			int ourID = m_pObject->GetObject() == objA ? 0 : 1;

			for (int i = 0; i < pManifold->getNumContacts(); i++) {
				btManifoldPoint &point = pManifold->getContactPoint(i);

				btVector3 norm = point.m_normalWorldOnB; // Normal worldspace A->B
				if (ourID == 1) {
					// Flip it because we're object B and we need norm B->A.
					norm *= -1;
				}

				// HACK: Guessing which way is up (as currently defined in our implementation y is up)
				// If the normal is up enough then assume it's some sort of ground
				if (norm.y() > 0.8) {
					return ourID == 0 ? pPhysB : pPhysA;
				}
			}
		}
	}

	return NULL;
}

void CPlayerController::Tick(float deltaTime) {
	if (!m_enable)
		return;

	// HACK: Only run this controller once per step (until I can figure out the math to fix per-tick simulation)
	if (m_pEnv->GetCurSubStep() != 0)
		return;
	deltaTime *= m_pEnv->GetNumSubSteps();

	btRigidBody *body = m_pObject->GetObject();
	btMassCenterMotionState *motionState = (btMassCenterMotionState *)body->getMotionState();

	// Don't let the player controller travel too far away from the target position.
	btTransform transform;
	motionState->getGraphicTransform(transform);
	btVector3 delta_position = m_targetPosition - transform.getOrigin();

	btScalar qdist = delta_position.length2();
	if (qdist > m_maxDeltaPosition * m_maxDeltaPosition && TryTeleportObject()) {
		// Teleported the controller, so no need to calculate velocity
		return;
	}

	CalculateVelocity(deltaTime);

	m_ticksSinceUpdate++;
}

void CPlayerController::CalculateVelocity(float dt) {
	btRigidBody *body = m_pObject->GetObject();

	// Fraction of the movement we need to complete this tick
	float fraction = 1.f;
	if (m_secondsToArrival > 0) {
		fraction = dt / m_secondsToArrival;
		if (fraction > 1) fraction = 1;
	}

	// FIXME: We're trying to do a move in too small of a timespan, so the move completes this tick
	// but it needed a crazy high velocity to work. 
	float scale = SAFE_DIVIDE(fraction, dt);

	m_secondsToArrival -= dt;
	if (m_secondsToArrival < 0) m_secondsToArrival = 0;

	// Float to allow stepping
	btVector3 gravDt = m_pEnv->GetBulletEnvironment()->getGravity() * dt;
	if (m_onground) {
		body->setLinearVelocity(body->getLinearVelocity() - gravDt);
	}

	btTransform transform;
	((btMassCenterMotionState *)body->getMotionState())->getGraphicTransform(transform);
	btVector3 deltaPos = m_targetPosition - transform.getOrigin();

	btVector3 baseVelocity(0);

	// Are we walking on some sort of vphysics ground? Add their velocity in as a base then
	// because the game doesn't do this for us!
	CPhysicsObject *pGround = GetGroundObject();
	if (pGround) {
		btTransform relTrans = pGround->GetObject()->getWorldTransform().inverse() * m_pObject->GetObject()->getWorldTransform();
		btVector3 relPos = relTrans.getOrigin();

		baseVelocity = pGround->GetObject()->getVelocityInLocalPoint(relPos);
	}

	btVector3 linVel = body->getLinearVelocity() - baseVelocity;
	if (m_ticksSinceUpdate == 0) {
		// TODO: We're applying too high acceleration when we get closer to the target position!
		ComputeController(linVel, deltaPos, m_maxSpeed, scale, m_dampFactor, &m_lastImpulse);
	} else {
		btScalar len = m_lastImpulse.length();
		btVector3 limit(len, len, len);
		
		ComputeController(linVel, deltaPos, limit, scale, m_dampFactor);
	}

	// TODO: Clamp the velocity based on collisions (using variables such as push max mass, max speed etc)
	// btScalar velLen = linVel.length2();

	body->setLinearVelocity(linVel + baseVelocity);
}

bool CPlayerController::TryTeleportObject() {
	if (m_handler) {
		Vector hlPosition;
		ConvertPosToHL(m_targetPosition, hlPosition);
		if (!m_handler->ShouldMoveTo(m_pObject, hlPosition)) return false;
	}

	btRigidBody *body = m_pObject->GetObject();

	btTransform trans = body->getWorldTransform();
	trans.setOrigin(m_targetPosition);

	body->setWorldTransform(trans * ((btMassCenterMotionState *)body->getMotionState())->m_centerOfMassOffset);
	((btMassCenterMotionState *)body->getMotionState())->setGraphicTransform(trans);

	// Kill the velocity
	body->setLinearVelocity(btVector3(0));

	return true;
}

void CPlayerController::ObjectDestroyed(CPhysicsObject *pObject) {
	if (pObject == m_pObject)
		DetachObject();
	else if (pObject == m_pGround)
		m_pGround = NULL;

	Assert(0); // Object isn't related to us, why are we getting this notification?
}

void CPlayerController::AttachObject() {
	btRigidBody *body = btRigidBody::upcast(m_pObject->GetObject());
	m_saveRot = body->getAngularFactor();
	body->setAngularFactor(0);

	m_pObject->AddCallbackFlags(CALLBACK_IS_PLAYER_CONTROLLER);

	body->setActivationState(DISABLE_DEACTIVATION, true);
}

void CPlayerController::DetachObject() {
	btRigidBody *body = btRigidBody::upcast(m_pObject->GetObject());
	body->setAngularFactor(m_saveRot);
	body->setActivationState(ACTIVE_TAG, true);

	m_pObject->RemoveCallbackFlags(CALLBACK_IS_PLAYER_CONTROLLER);

	m_pObject = NULL;
}

void CPlayerController::SetPushMassLimit(float maxPushMass) {
	m_pushMassLimit = maxPushMass;
}

void CPlayerController::SetPushSpeedLimit(float maxPushSpeed) {
	m_pushSpeedLimit = maxPushSpeed;
}

float CPlayerController::GetPushMassLimit() {
	return m_pushMassLimit;
}

float CPlayerController::GetPushSpeedLimit() {
	return m_pushSpeedLimit;
}

bool CPlayerController::WasFrozen() {
	// Appears that if we were frozen, the game will try and update our position to the player's current position.
	// Probably used for when the controller object is frozen due to performance limits (max collisions per timestep, etc)
	// TODO: Implement this if we ever implement performance limitations

	//NOT_IMPLEMENTED
	return false;
}

/***********************
* CREATION FUNCTIONS
***********************/

CPlayerController *CreatePlayerController(CPhysicsEnvironment *pEnv, IPhysicsObject *pObject) {
	if (!pObject) return NULL;

	return new CPlayerController(pEnv, (CPhysicsObject *)pObject);
}

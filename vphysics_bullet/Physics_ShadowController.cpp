#include "StdAfx.h"

#include "Physics_Environment.h"
#include "Physics_ShadowController.h"
#include "Physics_PlayerController.h"
#include "Physics_Object.h"
#include "Physics_SurfaceProps.h"

#include "convert.h"
#include "miscmath.h"

#include <math.h>

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

float ComputeShadowControllerBull(btRigidBody *object, shadowcontrol_params_t &params, float secondsToArrival, float dt) {
	// Fraction of the movement we need to complete by this tick
	float fraction = 1;
	if (secondsToArrival > 0) {
		fraction = dt / secondsToArrival;
		if (fraction > 1) fraction = 1;
	}

	secondsToArrival -= dt;
	if (secondsToArrival < 0) secondsToArrival = 0;

	if (fraction <= 0) return secondsToArrival;
	float scale = SAFE_DIVIDE(fraction, dt);

	btTransform transform = object->getWorldTransform();
	transform *= ((btMassCenterMotionState *)object->getMotionState())->m_centerOfMassOffset.inverse();

	//-------------------
	// Translation
	//-------------------

	btVector3 posbull = transform.getOrigin();
	btVector3 delta_position = params.targetPosition - posbull;

	// Teleportation
	// If our distance is greater than teleport distance, teleport instead.
	if (params.teleportDistance > 0) {
		btScalar qdist;
		if (!params.lastPosition.isZero()) {
			btVector3 tmpDelta = posbull - params.lastPosition;
			qdist = tmpDelta.length2();
		} else {
			qdist = delta_position.length2();
		}

		if (qdist > params.teleportDistance * params.teleportDistance) {
			transform.setOrigin(params.targetPosition);
			transform.setRotation(params.targetRotation);
			object->setWorldTransform(transform * ((btMassCenterMotionState *)object->getMotionState())->m_centerOfMassOffset);
		}
	}

	btVector3 speed = object->getLinearVelocity();
	ComputeController(speed, delta_position, btVector3(params.maxSpeed, params.maxSpeed, params.maxSpeed), scale, params.dampFactor);
	object->setLinearVelocity(speed);

	params.lastPosition = posbull + (speed * dt);

	//-------------------
	// Rotation
	//-------------------

	btVector3 axis;
	btScalar angle;
	btTransformUtil::calculateDiffAxisAngleQuaternion(transform.getRotation(), params.targetRotation, axis, angle);

	// So we don't end up having a huge delta angle (such as instead of doing 379 deg turn, do a -1 deg turn)
	if (angle > M_PI) {
		angle -= btScalar(2 * M_PI);
	}

	btVector3 deltaAngles = axis * angle;
	btVector3 rot_speed = object->getAngularVelocity();
	ComputeController(rot_speed, deltaAngles, btVector3(params.maxAngular, params.maxAngular, params.maxAngular), scale, params.dampFactor);
	object->setAngularVelocity(rot_speed);

	object->activate();

	return secondsToArrival;
}

void ConvertShadowControllerToBull(const hlshadowcontrol_params_t &in, shadowcontrol_params_t &out) {
	ConvertPosToBull(in.targetPosition, out.targetPosition);
	ConvertRotationToBull(in.targetRotation, out.targetRotation);
	out.teleportDistance = ConvertDistanceToBull(in.teleportDistance);

	out.maxSpeed = ConvertDistanceToBull(in.maxSpeed);
	out.maxDampSpeed = ConvertDistanceToBull(in.maxDampSpeed);
	out.maxAngular = ConvertAngleToBull(in.maxAngular);
	out.maxDampAngular = ConvertAngleToBull(in.maxDampAngular);
	out.dampFactor = in.dampFactor;
}

float ComputeShadowControllerHL(CPhysicsObject *pObject, const hlshadowcontrol_params_t &params, float secondsToArrival, float dt) {
	shadowcontrol_params_t bullParams;
	ConvertShadowControllerToBull(params, bullParams);
	return ComputeShadowControllerBull(pObject->GetObject(), bullParams, secondsToArrival, dt);
}

static bool IsEqual(const btQuaternion &pt0, const btQuaternion &pt1a) {
	btQuaternion pt1 = pt0.nearest(pt1a);

	btScalar dot = pt0.normalized().dot(pt1.normalized());
	return dot >= 1 - SIMD_EPSILON || dot <= -1 + SIMD_EPSILON;
}

static bool IsEqual(const btVector3 &pt0, const btVector3 &pt1) {
	return pt0.distance2(pt1) < 1e-8f;
}

/***************************
* CLASS CShadowController
***************************/

CShadowController::CShadowController(CPhysicsObject *pObject, bool allowTranslation, bool allowRotation) {
	m_pObject = pObject;
	m_shadow.dampFactor = 1.0f;
	m_shadow.teleportDistance = 0;
	m_shadow.targetPosition.setZero();
	m_shadow.targetRotation = btQuaternion::getIdentity();
	m_flags = 0;
	m_ticksSinceUpdate = 0;

	SetAllowsTranslation(allowTranslation);
	SetAllowsRotation(allowRotation);
	AttachObject();
}

CShadowController::~CShadowController() {
	DetachObject();
}

// UNEXPOSED
void CShadowController::Tick(float deltaTime) {
	if (m_enable) {
		if (IsPhysicallyControlled()) {
			ComputeShadowControllerBull(m_pObject->GetObject(), m_shadow, m_secondsToArrival, deltaTime);
			m_secondsToArrival -= deltaTime;
			if (m_secondsToArrival < 0) m_secondsToArrival = 0;
		} else {
			// TODO: Need to use secondsToArrival

			btTransform target(m_shadow.targetRotation, m_shadow.targetPosition);
			target *= ((btMassCenterMotionState *)m_pObject->GetObject()->getMotionState())->m_centerOfMassOffset;
			m_pObject->GetObject()->setWorldTransform(target);
		}
	} else {
		m_shadow.lastPosition.setZero();
	}

	m_ticksSinceUpdate++;
}

void CShadowController::ObjectDestroyed(CPhysicsObject *pObject) {
	if (pObject == m_pObject)
		DetachObject();
}

void CShadowController::Update(const Vector &position, const QAngle &angles, float timeOffset) {
	btVector3 targetPosition = m_shadow.targetPosition;
	btQuaternion targetRotation = m_shadow.targetRotation;

	ConvertPosToBull(position, m_shadow.targetPosition);
	ConvertRotationToBull(angles, m_shadow.targetRotation);
	m_secondsToArrival = timeOffset < 0 ? 0 : timeOffset;

	m_enable = true;
	m_timeOffset = timeOffset;
	m_ticksSinceUpdate = 0;

	if (IsEqual(targetPosition, m_shadow.targetPosition) && IsEqual(targetRotation, m_shadow.targetRotation)) return;

	//m_pObject->Wake();
}

void CShadowController::MaxSpeed(float maxSpeed, float maxAngularSpeed) {
	btRigidBody *body = m_pObject->GetObject();

	//----------------
	// Linear
	//----------------

	btVector3 bullSpeed;
	ConvertPosToBull(Vector(maxSpeed, maxSpeed, maxSpeed), bullSpeed);
	btVector3 available = bullSpeed;

	// m_currentSpeed = bullSpeed;

	float length = bullSpeed.length();
	bullSpeed.normalize();

	float dot = bullSpeed.dot(body->getLinearVelocity());
	if (dot > 0) {
		bullSpeed *= dot * length;
		available -= bullSpeed;
	}

	// FIXME: This is wrong. Rewrite this later.
	m_shadow.maxSpeed = available.length();

	//----------------
	// Angular
	//----------------

	btVector3 bullAngular;
	ConvertAngularImpulseToBull(Vector(maxAngularSpeed, maxAngularSpeed, maxAngularSpeed), bullAngular);
	btVector3 availableAngular;

	float lengthAngular = bullAngular.length();
	bullAngular.normalize();

	float dotAngular = bullAngular.dot(body->getAngularVelocity());
	if (dotAngular > 0) {
		bullAngular *= dotAngular * lengthAngular;
		availableAngular -= bullAngular;
	}

	// FIXME: This is wrong. Rewrite this later.
	m_shadow.maxAngular = availableAngular.length();
}

void CShadowController::StepUp(float height) {
	btVector3 step;
	ConvertPosToBull(Vector(0, 0, height), step);

	btRigidBody *pObject = m_pObject->GetObject();
	btTransform transform = pObject->getWorldTransform();
	transform.setOrigin(transform.getOrigin() + step);

	pObject->setWorldTransform(transform);
}

void CShadowController::SetTeleportDistance(float teleportDistance) {
	m_shadow.teleportDistance = ConvertDistanceToBull(teleportDistance);
}

bool CShadowController::AllowsTranslation() {
	return (m_flags & FLAG_ALLOWPHYSICSMOVEMENT) != 0;
}

bool CShadowController::AllowsRotation() {
	return (m_flags & FLAG_ALLOWPHYSICSROTATION) != 0;
}

// There are two classes of shadow objects:
// 1) Game physics controlled, shadow follows game physics (this is the default)
// 2) Physically controlled - shadow position is a target, but the game hasn't guaranteed that the space can be occupied by this object
bool CShadowController::IsPhysicallyControlled() {
	return (m_flags & FLAG_PHYSICALLYCONTROLLED) != 0;
}

void CShadowController::SetAllowsTranslation(bool enable) {
	enable ? m_flags |= FLAG_ALLOWPHYSICSMOVEMENT : m_flags &= ~(FLAG_ALLOWPHYSICSMOVEMENT);
}

void CShadowController::SetAllowsRotation(bool enable) {
	enable ? m_flags |= FLAG_ALLOWPHYSICSROTATION : m_flags &= ~(FLAG_ALLOWPHYSICSROTATION);
}

void CShadowController::SetPhysicallyControlled(bool enable) {
	if (IsPhysicallyControlled() == enable)
		return;

	if (enable) {
		m_flags |= FLAG_PHYSICALLYCONTROLLED;

		btRigidBody *body = m_pObject->GetObject();
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	} else {
		m_flags &= ~(FLAG_PHYSICALLYCONTROLLED);

		btRigidBody *body = m_pObject->GetObject();
		body->setCollisionFlags(body->getCollisionFlags() & ~(btCollisionObject::CF_KINEMATIC_OBJECT));
	}
}

// NPCs call this
void CShadowController::GetLastImpulse(Vector *pOut) {
	if (!pOut) return;

	//NOT_IMPLEMENTED
	*pOut = Vector(0,0,0);
}

void CShadowController::UseShadowMaterial(bool enable) {
	enable ? m_flags |= FLAG_USESHADOWMATERIAL : m_flags &= ~(FLAG_USESHADOWMATERIAL);
}

void CShadowController::ObjectMaterialChanged(int materialIndex) {
	// (assumed)
	// if (m_bUseShadowMaterial) {
	//		m_iObjectMaterial = materialIndex
	// }

	NOT_IMPLEMENTED
}

// Basically get the last inputs to IPhysicsShadowController::Update(), returns last input to timeOffset in Update()
float CShadowController::GetTargetPosition(Vector *pPositionOut, QAngle *pAnglesOut) {
	if (pPositionOut)
		ConvertPosToHL(m_shadow.targetPosition, *pPositionOut);

	if (pAnglesOut)
		ConvertRotationToHL(m_shadow.targetRotation, *pAnglesOut);

	return m_timeOffset;
}

float CShadowController::GetTeleportDistance() {
	return ConvertDistanceToHL(m_shadow.teleportDistance);
}

void CShadowController::GetMaxSpeed(float *pMaxSpeedOut, float *pMaxAngularSpeedOut) {
	if (!pMaxSpeedOut && !pMaxAngularSpeedOut) return;
	NOT_IMPLEMENTED
}

void CShadowController::AttachObject() {
	if (!m_pObject)
		return;

	btRigidBody *body = btRigidBody::upcast(m_pObject->GetObject());
	m_savedMass = SAFE_DIVIDE(1, body->getInvMass());
	m_savedMaterialIndex = m_pObject->GetMaterialIndex();

	m_pObject->SetMaterialIndex(MATERIAL_INDEX_SHADOW);

	if (!AllowsTranslation()) {
		m_pObject->SetMass(0);
		m_pObject->EnableGravity(false);
	}

	body->setActivationState(DISABLE_DEACTIVATION);

	m_pObject->AttachEventListener(this);
}

void CShadowController::DetachObject() {
	if (!m_pObject)
		return;

	btRigidBody *body = btRigidBody::upcast(m_pObject->GetObject());
	btVector3 btvec = body->getInvInertiaDiagLocal();
	btvec.setX(SAFE_DIVIDE(1.0f, btvec.x()));
	btvec.setY(SAFE_DIVIDE(1.0f, btvec.y()));
	btvec.setZ(SAFE_DIVIDE(1.0f, btvec.z()));
	body->setMassProps(m_savedMass, btvec);
	m_pObject->SetMaterialIndex(m_savedMaterialIndex);

	body->setActivationState(ACTIVE_TAG);

	m_pObject->DetachEventListener(this);
	m_pObject = NULL;
}

int CShadowController::GetTicksSinceUpdate() {
	return m_ticksSinceUpdate;
}

/*************************
* CREATION FUNCTIONS
*************************/

CShadowController *CreateShadowController(IPhysicsObject *pObject, bool allowPhysicsMovement, bool allowPhysicsRotation) {
	if (!pObject)
		return NULL;

	return new CShadowController((CPhysicsObject *)pObject, allowPhysicsMovement, allowPhysicsRotation);
}
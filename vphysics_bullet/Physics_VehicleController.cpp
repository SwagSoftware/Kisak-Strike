#include "StdAfx.h"

#include <cmodel.h>

#include "Physics_Object.h"
#include "Physics_VehicleController.h"
#include "Physics_Environment.h"
#include "convert.h"

#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "BulletDynamics/Vehicle/btWheeledVehicle.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

// MPH2MS: 1(miles/hr) = 0.44704(meters/sec)
#define MPH2MS(x) ((x) * 0.44704f)
// KMH2MS: 1(km/h) = 0.27777778(meters/sec)
#define KMH2MS(x) ((x) * 0.27777778f)

/*********************************
* MISC CLASSES
*********************************/

class CDefaultCarWheelTracer : public IPhysicsVehicleWheelTrace {
	public:
		CDefaultCarWheelTracer(CPhysicsVehicleController *pVehicle, IPhysicsGameTrace *pGameTrace):
		m_pVehicle(pVehicle),
		m_pGameTrace(pGameTrace) {

		}

		IPhysicsObject *CastRay(int wheelIndex, const Vector &start, const Vector &end, trace_t &result) {
			Ray_t ray;
			ray.m_Start = start;
			ray.m_Delta = end - start;
			m_pGameTrace->VehicleTraceRay(ray, (void *)m_pVehicle, &result);
			//lwss: Added nullptr here, needs to return something!
			return nullptr;
		}

	private:
		CPhysicsVehicleController *m_pVehicle;
		IPhysicsGameTrace *m_pGameTrace;
};

// Purpose: This ray will ignore a body AND detect water for use in airboats.
struct CDetectWaterRayResultCallback : public btCollisionWorld::ClosestRayResultCallback {
	CDetectWaterRayResultCallback(const btRigidBody *pIgnoreObject, const btVector3 &from, const btVector3 &to)
	: ClosestRayResultCallback(from, to) {
		m_pIgnoreObject = pIgnoreObject;
	}

	bool needsCollision(btBroadphaseProxy *proxy0) const {
		btRigidBody *pBody = (btRigidBody *)proxy0->m_clientObject;
		if (pBody) {
			if (pBody == m_pIgnoreObject)
				return false;

			CPhysicsObject *pPhys = (CPhysicsObject *)pBody->getUserPointer();

			if (pPhys) {
				if (pPhys->GetCallbackFlags() & CALLBACK_FLUID_TOUCH || pPhys->GetContents() & MASK_WATER)
					return true;
			}
		}

		return btCollisionWorld::ClosestRayResultCallback::needsCollision(proxy0);
	}

	const btRigidBody *m_pIgnoreObject;
};

struct CIgnoreObjectRayResultCallback : public btCollisionWorld::ClosestRayResultCallback {
	CIgnoreObjectRayResultCallback(const btRigidBody *pIgnoreObject, const btVector3 &from, const btVector3 &to):
	ClosestRayResultCallback(from, to) {
		m_pIgnoreObject = pIgnoreObject;
	}

	bool needsCollision(btBroadphaseProxy *proxy0) const {
		btRigidBody *pBody = (btRigidBody *)proxy0->m_clientObject;
		CPhysicsObject *pPhys = (CPhysicsObject *)pBody->getUserPointer();

		// Cheating is allowed, right? Gonna guess this is our chassis just because.
		CPhysicsObject *pChassis = (CPhysicsObject *)m_pIgnoreObject->getUserPointer();

		if (pBody && pBody == m_pIgnoreObject) {
			return false;
		} else if (pPhys && !pPhys->IsCollisionEnabled()) {
			return false;
		} else if (!pChassis->GetVPhysicsEnvironment()->GetCollisionSolver()->NeedsCollision(pChassis, pPhys)) {
			return false;
		}

		return btCollisionWorld::ClosestRayResultCallback::needsCollision(proxy0);
	}

	const btRigidBody *m_pIgnoreObject;
};

// Purpose: This raycaster will cast a ray ignoring the vehicle's body.
class CCarRaycaster : public btVehicleRaycaster {
	public:
		CCarRaycaster(btDynamicsWorld *pWorld, CPhysicsVehicleController *pController) {
			m_pWorld = pWorld;
			m_pController = pController;
		}

		void *castRay(btWheelInfo *wheel, const btVector3 &from, const btVector3 &to, btVehicleRaycasterResult &result) {
			CIgnoreObjectRayResultCallback rayCallback(m_pController->GetBody()->GetObject(), from, to);
			rayCallback.m_flags |= btTriangleRaycastCallback::kF_UseSubSimplexConvexCastRaytest; // GJK has an issue of going through triangles
			
			m_pWorld->rayTest(from, to, rayCallback);
			
			if (rayCallback.hasHit()) {
				const btRigidBody *body = btRigidBody::upcast(rayCallback.m_collisionObject);
				if (body && body->hasContactResponse()) {
					result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
					result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
					result.m_hitNormalInWorld.normalize();
					result.m_distFraction = rayCallback.m_closestHitFraction;
					return (void *)body;
				}
			}

			return NULL;
		}

	private:
		btDynamicsWorld *			m_pWorld;
		CPhysicsVehicleController *	m_pController;
};

// Purpose: Airboat raycaster
// DEPRECATED: To be replaced...
class CAirboatRaycaster : public btVehicleRaycaster {
	public:
		CAirboatRaycaster(btDynamicsWorld *pWorld, btRigidBody *pBody) {
			m_pWorld = pWorld;
			m_pBody = pBody;
		}

		// Returns the rigid body the ray hits
		void *castRay(btWheelInfo *wheel, const btVector3 &from, const btVector3 &to, btVehicleRaycasterResult &result) {
			CDetectWaterRayResultCallback rayCallback(m_pBody, from, to);

			m_pWorld->rayTest(from, to, rayCallback);

			if (rayCallback.hasHit()) {
				const btRigidBody *body = btRigidBody::upcast(rayCallback.m_collisionObject);
				if (body) {
					result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
					result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
					result.m_hitNormalInWorld.normalize();
					result.m_distFraction = rayCallback.m_closestHitFraction;
					return (void *)body;
				}
			}

			return NULL;
		}

	private:
		btDynamicsWorld *	m_pWorld;
		btRigidBody *		m_pBody;
};

/*********************************
* CLASS CPhysicsVehicleController
*********************************/

CPhysicsVehicleController::CPhysicsVehicleController(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace) {
	m_pEnv				= pEnv;
	m_pBody				= pBody;
	m_iVehicleType		= nVehicleType;
	m_pGameTrace		= pGameTrace;
	m_bEngineDisabled	= false;

	memset(&m_vehicleState, 0, sizeof(m_vehicleState));
	InitVehicleParams(params);

	for (int i = 0; i < VEHICLE_MAX_WHEEL_COUNT; i++) {
		m_pWheels[i] = NULL;
	}

	// Keep the vehicle active (or it might go to sleep while a player's driving it and make it unmoveable)
	m_pBody->GetObject()->setActivationState(DISABLE_DEACTIVATION);
	if (m_pBody->GetVehicleController() != NULL) {
		// Although this is a warning, it's not fatal because we can't set the vehicle controller to NULL
		// when we're destroyed, because the game typically destroys the body before destroying the vehicle controller
		Warning("VPhysics: Attaching a vehicle controller to object \"%s\" that already has one attached!\n", pBody->GetName());
	}

	m_pBody->SetVehicleController(this);

	m_iWheelCount = m_vehicleParams.axleCount * m_vehicleParams.wheelsPerAxle;

	// Initialization and setup
	InitBullVehicle();
}

CPhysicsVehicleController::~CPhysicsVehicleController() {
	ShutdownBullVehicle();
}

void CPhysicsVehicleController::InitVehicleParams(const vehicleparams_t &params) {
	m_vehicleParams = params;
}

void CPhysicsVehicleController::InitBullVehicle() {
	// NOTE: We're faking the car wheels for now because bullet does not offer a vehicle with physical wheels.
	// TODO: Simulate the car wheels to a degree
	// See: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?p=&f=&t=1817
#ifndef USE_WHEELED_VEHICLE
	if (m_iVehicleType == VEHICLE_TYPE_CAR_WHEELS)
		m_pRaycaster = new CCarRaycaster(m_pEnv->GetBulletEnvironment(), this);
	else if (m_iVehicleType == VEHICLE_TYPE_CAR_RAYCAST)
		m_pRaycaster = new CCarRaycaster(m_pEnv->GetBulletEnvironment(), this);
	else if (m_iVehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST)
		m_pRaycaster = new CAirboatRaycaster(m_pEnv->GetBulletEnvironment(), m_pBody->GetObject());
	else
		Assert(0);

	m_pVehicle = new btRaycastVehicle(m_tuning, m_pBody->GetObject(), m_pRaycaster);
	m_pVehicle->setCoordinateSystem(0, 1, 2);
#else
	m_pVehicle = new btWheeledVehicle(m_pEnv->GetBulletEnvironment(), m_pBody->GetObject());
#endif

	m_pEnv->GetBulletEnvironment()->addAction(m_pVehicle);

	InitCarWheels();
}

void CPhysicsVehicleController::ShutdownBullVehicle() {
	m_pEnv->GetBulletEnvironment()->removeAction(m_pVehicle);

	// Delete the vehicle before the wheels so the constraints are freed
#ifdef USE_WHEELED_VEHICLE
	delete m_pVehicle;
#else
	delete m_pVehicle;
	delete m_pRaycaster;
#endif

	for (int i = 0; i < m_iWheelCount; i++) {
		m_pEnv->DestroyObject(m_pWheels[i]);
		m_pWheels[i] = NULL;
	}
}

void CPhysicsVehicleController::InitCarWheels() {
	int wheelIndex = 0;

	for (int i = 0; i < m_vehicleParams.axleCount; i++) {
		for (int w = 0; w < m_vehicleParams.wheelsPerAxle; w++, wheelIndex++) {
			CPhysicsObject *pWheel = CreateWheel(wheelIndex, m_vehicleParams.axles[i]);
			if (pWheel) {
				m_pWheels[wheelIndex] = pWheel;
			}
		}
	}

	for (int i = 0; i < m_iWheelCount; i++) {
		m_pWheels[i]->EnableGravity(false);	// Otherwise they slowly sink.
	}
}

void CPhysicsVehicleController::DestroyCarWheels() {
	for (int i = 0; i < m_iWheelCount; i++) {
		m_pEnv->DestroyObject(m_pWheels[i]);
		m_pWheels[i] = NULL;
	}
}

// Purpose: Create wheel on source side (CPhysicsObject *) and add a wheel to the raycaster.
CPhysicsObject *CPhysicsVehicleController::CreateWheel(int wheelIndex, vehicle_axleparams_t &axle) {
	if (wheelIndex >= VEHICLE_MAX_WHEEL_COUNT)
		return NULL;

	Vector position = axle.offset;
	Vector bodyPosition;
	QAngle bodyAngles;
	m_pBody->GetPosition(&bodyPosition, &bodyAngles);
	matrix3x4_t matrix;
	AngleMatrix(bodyAngles, bodyPosition, matrix);

	// HACK: This will only work with vehicles that have 2 wheels per axle
	if (wheelIndex & 1) {
		position += axle.wheelOffset;
	} else {
		position -= axle.wheelOffset;
	}

	QAngle angles = vec3_angle;
	Vector wheelPositionHL;
	VectorTransform(position, matrix, wheelPositionHL);

	objectparams_t params;
	memset(&params, 0, sizeof(params));

	params.damping = axle.wheels.damping;
	params.dragCoefficient = 0;
#ifdef USE_WHEELED_VEHICLE
	params.enableCollisions = true;
#else
	params.enableCollisions = false;
#endif
	params.inertia = axle.wheels.inertia;
	params.mass = axle.wheels.mass;
	params.pGameData = m_pBody->GetGameData();
	params.pName = "VehicleWheel";
	params.rotdamping = axle.wheels.rotdamping;
	params.rotInertiaLimit = 0;

	// needs to be in HL units because we're calling through the "outer" interface to create
	// the wheels
	float radius = axle.wheels.radius;
	float r3 = radius * radius * radius;
	params.volume = (4.0f / 3.0f) * M_PI_F * r3;

	// TODO: Change this to a cylinder!
	CPhysicsObject *pWheel = (CPhysicsObject *)m_pEnv->CreateSphereObject(radius, axle.wheels.materialIndex, wheelPositionHL, angles, &params);
	pWheel->Wake();
	pWheel->AddCallbackFlags(CALLBACK_IS_VEHICLE_WHEEL);

	pWheel->GetObject()->setActivationState(DISABLE_DEACTIVATION);

	// Create it in bullet now
#ifdef USE_WHEELED_VEHICLE
	btVector3 wheelOffset;
	ConvertPosToBull(position, wheelOffset);
	wheelOffset -= ((btMassCenterMotionState *)m_pBody->GetObject()->getMotionState())->m_centerOfMassOffset.getOrigin();

	btVehicleWheel &wheel = m_pVehicle->addWheel(pWheel->GetObject(), wheelOffset, btMatrix3x3::getIdentity(), btVector3(0, -1, 0), btVector3(-1, 0, 0));
	wheel.suspensionRestLength = ConvertDistanceToBull(axle.wheels.springAdditionalLength);
	wheel.suspensionConstant = axle.suspension.springConstant;
	wheel.suspensionDamping = axle.suspension.springDamping;
	wheel.maxSuspensionForce = axle.suspension.maxBodyForce * m_pBody->GetMass();
#else
	// Create the wheel in bullet
	btVector3 bullConnectionPointCS0;
	btScalar bullSuspensionRestLength, bullWheelRadius;

	// This parameter is unused by bullet.
	bool bIsFrontWheel = (wheelIndex < 2);		// HACK: Only works with 2 front wheels

	btVector3 bullWheelDirectionCS0(0, -1, 0);	// Straight down
	btVector3 bullWheelAxleCS(1, 0, 0);			// Left

	ConvertPosToBull(position, bullConnectionPointCS0);
	bullConnectionPointCS0 -= ((btMassCenterMotionState *)m_pBody->GetObject()->getMotionState())->m_centerOfMassOffset.getOrigin();

	bullSuspensionRestLength = ConvertDistanceToBull(axle.wheels.springAdditionalLength);
	bullWheelRadius = ConvertDistanceToBull(axle.wheels.radius);

	btWheelInfo &wheelInfo = m_pVehicle->addWheel(bullConnectionPointCS0, bullWheelDirectionCS0, bullWheelAxleCS, bullSuspensionRestLength, bullWheelRadius, m_tuning, bIsFrontWheel);

	// FIXME: frictionScale is UNUSED (or we're not parsing something correctly)!
	//wheelInfo.m_frictionSlip = axle.wheels.frictionScale;
	wheelInfo.m_frictionSlip = 1.5f; // debug value
	wheelInfo.m_maxSuspensionForce = axle.suspension.maxBodyForce * m_pBody->GetMass();
	wheelInfo.m_suspensionStiffness = axle.suspension.springConstant;
	
	wheelInfo.m_clientInfo = pWheel;
#endif

	return pWheel;
}

// Bullet appears to take force in newtons. Correct this if it's wrong.
void CPhysicsVehicleController::SetWheelForce(int wheelIndex, float force) {
	if (wheelIndex >= m_iWheelCount || wheelIndex < 0) {
		Assert(0);
		return;
	}

	// convert from kg*in/s to kg*m/s
	force *= METERS_PER_INCH;
	//m_pVehicle->applyEngineForce(force, wheelIndex);
}

void CPhysicsVehicleController::SetWheelBrake(int wheelIndex, float brakeVal) {
	if (wheelIndex >= m_iWheelCount || wheelIndex < 0) {
		Assert(0);
		return;
	}

	// convert from kg*in/s to kg*m/s
	brakeVal *= METERS_PER_INCH;
	//m_pVehicle->setBrake(brakeVal, wheelIndex);
}

void CPhysicsVehicleController::SetWheelSteering(int wheelIndex, float steerVal) {
	if (wheelIndex >= m_iWheelCount || wheelIndex < 0) {
		Assert(0);
		return;
	}

#ifdef USE_WHEELED_VEHICLE
	m_pVehicle->getWheel(wheelIndex).steering = ConvertAngleToBull(steerVal);
#else
	//m_pVehicle->setSteeringValue(ConvertAngleToBull(steerVal), wheelIndex);
#endif
}

void CPhysicsVehicleController::Update(float dt, vehicle_controlparams_t &controls) {
	if (controls.handbrake) {
		controls.throttle = 0.0f;
	}

	if (controls.throttle == 0.0f && controls.brake == 0.0f && !controls.handbrake) {
		controls.brake = 0.1f;
	}

#ifndef USE_WHEELED_VEHICLE
	UpdateSteering(controls, dt);
	UpdateEngine(controls, dt);
	UpdateWheels(controls, dt);
#endif
}

#ifndef USE_WHEELED_VEHICLE
void CPhysicsVehicleController::UpdateSteering(vehicle_controlparams_t &controls, float dt) {
	float steeringVal = controls.steering;

	// TODO: Calculate for degreesBoost
	float speed = -KMH2MS(m_pVehicle->getCurrentSpeedKmHour());
	if (speed <= MPH2MS(m_vehicleParams.steering.speedSlow)) {
		steeringVal *= m_vehicleParams.steering.degreesSlow;
	} else if (speed >= MPH2MS(m_vehicleParams.steering.speedFast)) {
		steeringVal *= m_vehicleParams.steering.degreesFast;
	} else {
		// Don't allow a division by zero (or negativity)
		if (m_vehicleParams.steering.speedSlow < m_vehicleParams.steering.speedFast) {
			// Inbetween, interpolate
			float val = (MPH2MS(m_vehicleParams.steering.speedFast) - speed) / (MPH2MS(m_vehicleParams.steering.speedFast) - MPH2MS(m_vehicleParams.steering.speedSlow));

			steeringVal *= m_vehicleParams.steering.degreesFast + (val * m_vehicleParams.steering.degreesSlow);
		}
	}

	m_vehicleState.steeringAngle = steeringVal;

	for (int i = 0; i < m_iWheelCount; i++) {
		if (m_pVehicle->getWheelInfo(i).m_bIsFrontWheel)
			m_pVehicle->setSteeringValue(ConvertAngleToBull(-steeringVal), i);
	}
}

void CPhysicsVehicleController::UpdateEngine(vehicle_controlparams_t &controls, float dt) {
	// Update the operating params
	// If speed is high negative, the brake will be applied!
	// FIXME: This does not report velocity relative to the ground! Should we calculate this from wheel rotation speed?
	float fSpeed = m_pVehicle->getCurrentSpeedKmHour();
	m_vehicleState.speed = ConvertDistanceToHL(KMH2MS(-fSpeed));

	CalcEngineTransmission(controls, dt);
	CalcEngine(controls, dt);
}

void CPhysicsVehicleController::UpdateWheels(vehicle_controlparams_t &controls, float dt) {
	m_vehicleState.wheelsInContact = 0;
	m_vehicleState.wheelsNotInContact = 0;
	m_vehicleState.skidSpeed = 0;

	for (int i = 0; i < m_iWheelCount; i++) {
		btTransform bullTransform = m_pVehicle->getWheelTransformWS(i);
		m_pWheels[i]->GetObject()->setWorldTransform(bullTransform);

		btTransform deltaTrans = m_pBody->GetObject()->getWorldTransform().inverse() * bullTransform;

		// Linear velocity for interpolation
		m_pWheels[i]->GetObject()->setLinearVelocity(m_pBody->GetObject()->getVelocityInLocalPoint(deltaTrans.getOrigin()));

		// Update wheels on ground
		btWheelInfo &wheel = m_pVehicle->getWheelInfo(i);
		if (wheel.m_raycastInfo.m_groundObject)
			m_vehicleState.wheelsInContact++;
		else
			m_vehicleState.wheelsNotInContact++;

		// TODO: Skidding
		// Wheel velocity at the contact point with the ground should be 0 relative to ground velocity
		// Largest of the four wheel's deltas is put into the state's skidSpeed and the surface props of the ground are put in skidMaterial
		if (wheel.m_raycastInfo.m_groundObject) {
			btVector3 wheelRelPos = wheel.m_raycastInfo.m_hardPointWS - btVector3(0, wheel.m_raycastInfo.m_suspensionLength + wheel.m_wheelsRadius, 0);

			btVector3 velAtGround = m_pBody->GetObject()->getVelocityInLocalPoint(wheelRelPos);

			// We're cheating a bit here, since this is a raycast vehicle with unrealistic math the wheels absolutely cannot skid
			// in the forward direction, so we'll only count sidewards speed as skidding
			float skidSpeed = fabs(wheel.m_raycastInfo.m_wheelAxleWS.dot(velAtGround));
			if (ConvertDistanceToHL(skidSpeed) > m_vehicleState.skidSpeed) {
				m_vehicleState.skidSpeed = ConvertDistanceToHL(skidSpeed);
			}
		}
	}
}
#endif

float CPhysicsVehicleController::UpdateBooster(float dt) {
	NOT_IMPLEMENTED
	return 0.0f; // Return boost delay.
}

#ifndef USE_WHEELED_VEHICLE
void CPhysicsVehicleController::CalcEngineTransmission(vehicle_controlparams_t &controls, float dt) {
	// throttle goes forward and backward, [-1, 1]
	// brake_val [0..1]

	//float absSpeed = fabs(KMH2MS(m_pVehicle->getCurrentSpeedKmHour()));

	const static int secondsPerMinute = 60;

	if (m_vehicleParams.engine.isAutoTransmission) {
		// Estimate the engine RPM
		float avgRotSpeed = 0;
		for (int i = 0; i < m_iWheelCount; i++) {
			btWheelInfo wheelInfo = m_pVehicle->getWheelInfo(i);

			float rotSpeed = fabs(wheelInfo.m_deltaRotation * wheelInfo.m_wheelsRadius);
			avgRotSpeed += rotSpeed;
		}

		//avgRotSpeed *= 0.5f / M_PI / m_iWheelCount;
		avgRotSpeed /= m_iWheelCount;

		float estEngineRPM = avgRotSpeed * m_vehicleParams.engine.axleRatio * m_vehicleParams.engine.gearRatio[m_vehicleState.gear] * secondsPerMinute;

		// only shift up when going forward
		if (controls.throttle > 0) {
			// check for higher gear, top gear is gearcount-1 (0 based)
			while (estEngineRPM > m_vehicleParams.engine.shiftUpRPM && m_vehicleState.gear < m_vehicleParams.engine.gearCount-1) {
				m_vehicleState.gear++;
				estEngineRPM = avgRotSpeed * m_vehicleParams.engine.axleRatio * m_vehicleParams.engine.gearRatio[m_vehicleState.gear] * secondsPerMinute;
			}
		}

		// check for lower gear
		while (estEngineRPM < m_vehicleParams.engine.shiftDownRPM && m_vehicleState.gear > 0) {
			m_vehicleState.gear--;
			estEngineRPM = avgRotSpeed * m_vehicleParams.engine.axleRatio * m_vehicleParams.engine.gearRatio[m_vehicleState.gear] * secondsPerMinute;
		}

		m_vehicleState.engineRPM = estEngineRPM;
	}
}

void CPhysicsVehicleController::CalcEngine(vehicle_controlparams_t &controls, float dt) {
	// Speed governor
	// TODO: These were apparently scrapped when vphysics was shipped. Figure out new speed governors by disassembly.

	// Apply our forces!
	// FIXME: Forces are in NEWTONS!
	if (fabs(controls.throttle) > 1e-4) {
		for (int i = 0; i < m_iWheelCount; i++) {
			m_pVehicle->setBrake(0, i);
		}

		const static int watt_per_hp = 745;
		const static int seconds_per_minute = 60;

		// TODO: Convert to NEWTONS!
		float force = controls.throttle * 
			m_vehicleParams.engine.horsepower * (watt_per_hp * seconds_per_minute) * 
			m_vehicleParams.engine.gearRatio[m_vehicleState.gear]  * m_vehicleParams.engine.axleRatio / 
			(m_vehicleParams.engine.maxRPM * (2 * M_PI_F));

		int wheelIndex = 0;
		for (int i = 0; i < m_vehicleParams.axleCount; i++) {
			float wheelForce = force * m_vehicleParams.axles[i].torqueFactor * ConvertDistanceToBull(m_vehicleParams.axles[i].wheels.radius);

			for (int w = 0; w < m_vehicleParams.wheelsPerAxle; w++, wheelIndex++) {
				m_pVehicle->applyEngineForce(wheelForce, wheelIndex);
			}
		}
	} else if (fabs(controls.brake) > 1e-4) {
		for (int i = 0; i < m_iWheelCount; i++) {
			m_pVehicle->applyEngineForce(0, i);
		}

		// Calculate the brake impulse
		// float wheel_force_by_brake = brake_val * m_gravityLength * ( m_bodyMass + m_totalWheelMass );
		float brakeImpulse = controls.brake * m_pBody->GetMass() * dt;

		int wheelIndex = 0;
		for (int i = 0; i < m_vehicleParams.axleCount; i++) {
			float wheelForce = brakeImpulse * m_vehicleParams.axles[i].brakeFactor;
			for (int w = 0; w < m_vehicleParams.wheelsPerAxle; w++, wheelIndex++) {
				m_pVehicle->setBrake(wheelForce, wheelIndex);
			}
		}
	} else {
		for (int i = 0; i < m_iWheelCount; i++) {
			m_pVehicle->applyEngineForce(0, i);
		}
	}

	if (controls.handbrake) {
		// IVP will freeze the wheel rotations, but we'll have to apply full friction on ground vs vehicle
		float normalForce = m_pBody->GetMass() * m_pEnv->GetBulletEnvironment()->getGravity().length();

		for (int i = 0; i < m_iWheelCount; i++) {
			float fricCoeff = m_pWheels[i]->GetObject()->getFriction();
			float wheelForce = (fricCoeff * normalForce) / m_iWheelCount;

			m_pVehicle->setBrake(wheelForce * dt, i);
			m_pVehicle->applyEngineForce(0, i);
		}
	}
}
#endif

CPhysicsObject *CPhysicsVehicleController::GetBody() {
	return m_pBody;
}

int CPhysicsVehicleController::GetWheelCount() {
	return m_iWheelCount;
}

IPhysicsObject *CPhysicsVehicleController::GetWheel(int index) {
	if (index >= m_iWheelCount || index < 0) return NULL;

	return m_pWheels[index];
}

bool CPhysicsVehicleController::GetWheelContactPoint(int index, Vector *pContactPoint, int *pSurfaceProps) {
	if ((index >= m_iWheelCount || index < 0) || (!pContactPoint && !pSurfaceProps)) return false;

#ifndef USE_WHEELED_VEHICLE
	btWheelInfo wheelInfo = m_pVehicle->getWheelInfo(index);
	if (wheelInfo.m_raycastInfo.m_isInContact) {
		btVector3 bullContactVec = wheelInfo.m_raycastInfo.m_contactPointWS;
		btCollisionObject *body = (btCollisionObject *)wheelInfo.m_raycastInfo.m_groundObject;
		CPhysicsObject *pObject = (CPhysicsObject *)body->getUserPointer();

		if (pContactPoint)
			ConvertPosToHL(bullContactVec, *pContactPoint);

		if (pSurfaceProps)
			*pSurfaceProps = pObject->GetMaterialIndex();

		return true;
	}
#endif

	return false;
}

void CPhysicsVehicleController::SetSpringLength(int wheelIndex, float length) {
	Assert(wheelIndex >= m_iWheelCount || wheelIndex < 0);
	if (wheelIndex >= m_iWheelCount || wheelIndex < 0) {
		return;
	}

	btScalar bullDist = ConvertDistanceToBull(length);
#ifdef USE_WHEELED_VEHICLE
	m_pVehicle->getWheel(wheelIndex).suspensionRestLength = bullDist;
#else
	m_pVehicle->getWheelInfo(wheelIndex).m_suspensionRestLength1 = bullDist;
#endif
}

void CPhysicsVehicleController::SetWheelFriction(int wheelIndex, float friction) {
	Assert(wheelIndex >= m_iWheelCount || wheelIndex < 0);
	if (wheelIndex >= m_iWheelCount || wheelIndex < 0) {
		return;
	}

#ifndef USE_WHEELED_VEHICLE
	m_pVehicle->getWheelInfo(wheelIndex).m_frictionSlip = friction;
#endif
}

void CPhysicsVehicleController::SetPosition(const Vector *pos, const QAngle *ang) {
	if (!pos && !ang) return;

	const btTransform oldTrans = m_pBody->GetObject()->getWorldTransform();
	btTransform &newTrans = m_pBody->GetObject()->getWorldTransform();
	if (pos) {
		btVector3 btPos;
		ConvertPosToBull(*pos, btPos);

		newTrans.setOrigin(btPos);
	}

	if (ang) {
		btMatrix3x3 btAng;
		ConvertRotationToBull(*ang, btAng);

		newTrans.setBasis(btAng);
	}

#ifdef USE_WHEELED_VEHICLE
	btTransform deltaTrans = newTrans.inverse() * oldTrans;
	for (int i = 0; i < m_iWheelCount; i++) {
		CPhysicsObject *pWheel = m_pWheels[i];
		btTransform &trans = pWheel->GetObject()->getWorldTransform();
		trans *= deltaTrans;
	}
#endif
}

void CPhysicsVehicleController::GetCarSystemDebugData(vehicle_debugcarsystem_t &debugCarSystem) {
	memset(&debugCarSystem, 0, sizeof(debugCarSystem));

	// NOTE: Durr valve are silly and expect us to return data in ivp coordinates. (See: line 619 of fourwheelvehiclephysics.cpp)
	// This means we have to convert all of our positions we return to IVP positions.
	// Luckily this is easy (us: Forward Up Right) (IVP: Forward Down Left)

	// wheel stuff
#ifndef USE_WHEELED_VEHICLE
	for (int i = 0; i < m_iWheelCount; i++) {
		btWheelInfo &wheelInfo = m_pVehicle->getWheelInfo(i);

		btVector3 wheelPos = wheelInfo.m_worldTransform.getOrigin();
		debugCarSystem.vecWheelPos[i].x = wheelPos.x();
		debugCarSystem.vecWheelPos[i].y = -wheelPos.y();
		debugCarSystem.vecWheelPos[i].z = -wheelPos.z();

		// raycast hitpos
		btVector3 rayContact = wheelInfo.m_raycastInfo.m_contactPointWS;
		debugCarSystem.vecWheelRaycastImpacts[i].x = rayContact.x();
		debugCarSystem.vecWheelRaycastImpacts[i].y = -rayContact.y();
		debugCarSystem.vecWheelRaycastImpacts[i].z = -rayContact.z();
	}
#endif
}

// Purpose: Reload vehicle params
void CPhysicsVehicleController::VehicleDataReload() {
	// Destroy the wheels first
	DestroyCarWheels();

	// Re-init
	InitVehicleParams(m_vehicleParams);
	InitCarWheels();
}

/****************************
* CREATION FUNCTIONS
****************************/

IPhysicsVehicleController *CreateVehicleController(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace) {
	if (!pBody) return NULL;

	CPhysicsVehicleController *pController = new CPhysicsVehicleController(pEnv, pBody, params, nVehicleType, pGameTrace);
	return pController;
}
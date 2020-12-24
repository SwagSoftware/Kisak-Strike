#include "StdAfx.h"

#include "Physics_Environment.h"
#include "Physics_Object.h"
#include "Physics_VehicleControllerCustom.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

class CWheelVehicle : public btActionInterface {
	public:
		CWheelVehicle(btRigidBody *chassis);

		void updateAction(btCollisionWorld *collisionWorld, btScalar dt);
		void debugDraw(btIDebugDraw *pDebugDraw);

	private:
		btRigidBody	*	m_pChassis;
};

CWheelVehicle::CWheelVehicle(btRigidBody *chassis) {
	m_pChassis = chassis;
}

void CWheelVehicle::updateAction(btCollisionWorld *collisionWorld, btScalar dt) {

}

void CWheelVehicle::debugDraw(btIDebugDraw *pDebugDraw) {

}

/*****************************************
 * CLASS CPhysicsVehicleControllerCustom
 *****************************************/

CPhysicsVehicleControllerCustom::CPhysicsVehicleControllerCustom(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, IPhysicsGameTrace *pGameTrace) {
	m_pEnv = pEnv;
	m_pBody = pBody;
	m_pGameTrace = pGameTrace;
}

CPhysicsVehicleControllerCustom::~CPhysicsVehicleControllerCustom() {

}

void CPhysicsVehicleControllerCustom::SetWheelForce(int wheelIndex, float force) {
	NOT_IMPLEMENTED
}

void CPhysicsVehicleControllerCustom::SetWheelBrake(int wheelIndex, float brakeVal) {
	NOT_IMPLEMENTED
}

void CPhysicsVehicleControllerCustom::SetWheelSteering(int wheelIndex, float steerVal) {
	NOT_IMPLEMENTED
}

void CPhysicsVehicleControllerCustom::Update(float dt) {
	NOT_IMPLEMENTED
}

void CPhysicsVehicleControllerCustom::CreateWheel(const vehicle_customwheelparams_t &params) {
	NOT_IMPLEMENTED
}

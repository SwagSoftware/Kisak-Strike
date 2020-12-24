#ifndef PHYSICS_VEHICLECONTROLLER_H
#define PHYSICS_VEHICLECONTROLLER_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

#include <vphysics/vehicles.h>
#include "vphysics_bullet/vehiclesV32.h"

class IPhysicsObject;
class CPhysicsObject;
class CPhysicsEnvironment;

struct btVehicleRaycaster;
class btRaycastVehicle;
class btWheeledVehicle;

// Toggle wheeled/raycast car (wheeled is not complete yet)
//#define USE_WHEELED_VEHICLE

// TODO: Implement this class and move it to the public interface.
// The game can implement this class to override wheel ray traces.
class IPhysicsVehicleWheelTrace {
	public:
		// Return the object if the ray hits, otherwise NULL
		// Also, be sure to fill the result trace.
		virtual IPhysicsObject *					CastRay(int wheelIndex, const Vector &start, const Vector &end, trace_t &result) = 0;
};

class CPhysicsVehicleController : public IPhysicsVehicleController32 {
	public:
		CPhysicsVehicleController(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace);
		~CPhysicsVehicleController();

		const vehicle_operatingparams_t &	GetOperatingParams() { return m_vehicleState; };
		const vehicleparams_t &				GetVehicleParams() { return m_vehicleParams; }
		vehicleparams_t &					GetVehicleParamsForChange() { return m_vehicleParams; }

		// force in kg*in/s
		void								SetWheelForce(int wheelIndex, float force);
		void								SetWheelBrake(int wheelIndex, float brakeVal);
		// steerVal is in degrees!
		void								SetWheelSteering(int wheelIndex, float steerVal);

		// Default vehicle handling...
		void								Update(float dt, vehicle_controlparams_t &controls);
		float								UpdateBooster(float dt);
		int									GetWheelCount();
		IPhysicsObject *					GetWheel(int index);
		bool								GetWheelContactPoint(int index, Vector *pContactPoint, int *pSurfaceProps);
		void								SetSpringLength(int wheelIndex, float length);
		void								SetWheelFriction(int wheelIndex, float friction);

		void								OnVehicleEnter() { m_bOccupied = true; }
		void								OnVehicleExit() { m_bOccupied = false; }

		void								SetEngineDisabled(bool bDisable) { m_bEngineDisabled = bDisable; }
		bool								IsEngineDisabled() { return m_bEngineDisabled; }

		// Set the position of the vehicle controller and its wheels (wheels relative to vehicle pos).
		// Use this instead of calling SetPosition on the chassis.
		void								SetPosition(const Vector *pos, const QAngle *ang);

		// Debug
		void								GetCarSystemDebugData(vehicle_debugcarsystem_t &debugCarSystem);
		void								VehicleDataReload();

	public:
		// Unexposed functions

		void								InitVehicleParams(const vehicleparams_t &params);
		void								InitBullVehicle();

		void								InitCarWheels();
		void								DestroyCarWheels();
		CPhysicsObject *					CreateWheel(int wheelIndex, vehicle_axleparams_t &axle);

		CPhysicsObject *					GetBody();

		void								UpdateSteering(vehicle_controlparams_t &controls, float dt);
		void								UpdateEngine(vehicle_controlparams_t &controls, float dt);
		void								UpdateWheels(vehicle_controlparams_t &controls, float dt);

		void								CalcEngineTransmission(vehicle_controlparams_t &controls, float dt);
		void								CalcEngine(vehicle_controlparams_t &controls, float dt);

		void								ShutdownBullVehicle();

		// To be exposed functions
		CPhysicsObject *					AddWheel();
	private:
		vehicleparams_t				m_vehicleParams;
		vehicle_operatingparams_t	m_vehicleState;
		CPhysicsObject *			m_pBody;
		CPhysicsEnvironment *		m_pEnv;
		IPhysicsGameTrace *			m_pGameTrace;
		unsigned int				m_iVehicleType;
		bool						m_bEngineDisabled;
		bool						m_bOccupied;
		CPhysicsObject *			m_pWheels[VEHICLE_MAX_WHEEL_COUNT];
		int							m_iWheelCount;
		bool						m_bSlipperyWheels;

#ifdef USE_WHEELED_VEHICLE
		btWheeledVehicle *			m_pVehicle;
#else
		btRaycastVehicle *			m_pVehicle;
		btVehicleRaycaster *		m_pRaycaster;
		btRaycastVehicle::btVehicleTuning	m_tuning;
#endif
};

IPhysicsVehicleController *CreateVehicleController(CPhysicsEnvironment *pEnv, CPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace);

#endif // PHYSICS_VEHICLECONTROLLER_H

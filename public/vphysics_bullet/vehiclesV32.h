#ifndef VEHICLESV32_H
#define VEHICLESV32_h
#ifdef _MSC_VER
#pragma once
#endif

#include "vphysics/vehicles.h"

// THIS INTERFACE IS NOT FINALIZED! FUNCTIONS MAY CHANGE!

class IPhysicsVehicleController32 : public IPhysicsVehicleController {
	public:
		// force in kg*in/s
		virtual void					SetWheelForce(int wheelIndex, float force) = 0;
		virtual void					SetWheelBrake(int wheelIndex, float brakeVal) = 0;
		// steerVal is in degrees!
		virtual void					SetWheelSteering(int wheelIndex, float steerVal) = 0;
};

#endif // VEHICLESV32_H
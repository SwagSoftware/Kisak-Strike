#ifndef CONSTRAINTSV32_H
#define CONSTRAINTSV32_h
#ifdef _MSC_VER
#pragma once
#endif

#include "vphysics/constraints.h"

struct physconstraintinfo_t {
	int numConstraintRows; // Amount of constraint rows
	int nub; // Unknown. Usually 6 - numConstraintRows. Appears unused in bullet.
};

struct physconstraintsolveinfo_t {
	Vector J1linearAxis, J1angularAxis; // Linear axis to solve on (normalized), Angular axis is rel pos cross normal.
	Vector J2linearAxis, J2angularAxis; // Same as above for 2nd rigid body, may be NULL if there is no 2nd rigid body.

	// Right hand sides of the equation (J*v = c + cfm * lambda)
	// FIXME: Not converted to/from HL units!
	float constraintError, cfm;

	// TODO: Description
	// FIXME: Not converted to/from HL units!
	float lowerLimit, upperLimit;

	inline void Defaults() {
		J1linearAxis.Zero();
		J1angularAxis.Zero();
		J2linearAxis.Zero();
		J2angularAxis.Zero();

		constraintError = 0;
		cfm = 0;

		lowerLimit = -FLT_MAX;
		upperLimit =  FLT_MAX;
	}
};

class IPhysicsUserConstraint {
public:
	virtual ~IPhysicsUserConstraint() {}

	// Optional function. Called when a constraint is destroyed that was attached to this constraint.
	// No other functions on this user constraint will be called afterwards from that constraint.
	virtual void			ConstraintDestroyed(IPhysicsConstraint *pConstraint) {}

	// WARNING: The following functions may not be called on the main thread! Do not do any non-threadsafe operations here!

	// Basic constraint info for internal setup
	virtual void			GetConstraintInfo(IPhysicsObject32 *pObjA, IPhysicsObject32 *pObjB, physconstraintinfo_t &info) = 0;

	// Info for constraint solve (an array of size numRows)
	// FPS - frames per second (1/stepsize), erp - default error reduction parameter (0..1)
	virtual void			GetConstraintSolveInfo(IPhysicsObject32 *pObjA, IPhysicsObject32 *pObjB, physconstraintsolveinfo_t *info, int numRows, float fps, float erp) = 0;
};

struct constraint_gearparams_t {
	constraint_breakableparams_t	constraint; // TODO: Will be supported in the future
	Vector	objectLocalAxes[2]; // Local axis in objects
	float	ratio; // Gear ratio

	inline void Defaults() {
		constraint.Defaults();
		objectLocalAxes[0].Init();
		objectLocalAxes[1].Init();
		ratio = 1;
	}
};

#endif // CONSTRAINTSV32_h
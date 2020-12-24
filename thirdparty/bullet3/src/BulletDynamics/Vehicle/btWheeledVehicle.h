/*
 * Copyright (c) 2014 Dr. Chat
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Dr. Chat makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/
#ifndef BT_WHEELEDVEHICLE_H
#define BT_WHEELEDVEHICLE_H

#include "BulletDynamics/Dynamics/btActionInterface.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"

class btDynamicsWorld;
class btWheelConstraint;

struct btVehicleWheel
{
	btRigidBody *pBody;

	// Offsets
	btVector3 bodyOffset;  // Offset from chassis in chassis space (up/down ignored for suspension)
	btMatrix3x3 basis;     // Wheel basis in chassis space (roll is ignored)
	btVector3 down;        // Down direction in chassis space
	btVector3 axle;        // Axle direction in chassis space (wheel rotates around this)

	// Values
	btScalar steering;     // Steering in radians (-left/+right)
	btScalar torque;       // Engine torque on wheel
	btScalar brakeTorque;  // Braking torque

	// Suspension
	btScalar suspensionConstant;
	btScalar suspensionRestLength;  // Up/down offset from bodyOffset
	btScalar suspensionDamping;
	btScalar maxSuspensionForce;

	// Internal constraint (will be created in addWheel)
	btWheelConstraint *pConstraint;
};

class btWheeledVehicle : public btActionInterface
{
public:
	btWheeledVehicle(btDynamicsWorld *pWorld, btRigidBody *pChassis);
	~btWheeledVehicle();

	btVehicleWheel &addWheel(btRigidBody *pBody, const btVector3 &bodyOffsetpos, const btMatrix3x3 &basis, const btVector3 &down, const btVector3 &axle);
	void removeWheel(int index);

	// Destroy/create the special constraint required by this
	void createConstraint(btVehicleWheel &wheel);
	void destroyConstraint(btVehicleWheel &wheel);

	const btVehicleWheel &getWheel(int idx) const;
	btVehicleWheel &getWheel(int idx);

	void updateAction(btCollisionWorld *pWorld, btScalar step);
	void debugDraw(btIDebugDraw *pDebugDrawer);

private:
	SIMD_FORCE_INLINE bool checkWheelIndex(int idx) const
	{
		btAssert(idx >= 0 && idx < m_wheels.size());
		return (idx >= 0 && idx < m_wheels.size());
	}

private:
	btAlignedObjectArray<btVehicleWheel> m_wheels;

	btRigidBody *m_pChassis;
	btDynamicsWorld *m_pWorld;
};

#endif  // BT_WHEELEDVEHICLE_H
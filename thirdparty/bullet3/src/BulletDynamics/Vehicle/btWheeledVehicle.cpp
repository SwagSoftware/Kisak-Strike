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

// For memset
#include <string.h>

#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "btWheeledVehicle.h"

class btWheelConstraint : public btTypedConstraint
{
public:
	btWheelConstraint(btRigidBody &chassis, btVehicleWheel &wheel) : btTypedConstraint(CONSTRAINT_TYPE_VEHICLEWHEEL, chassis, *wheel.pBody)
	{
		m_pWheel = &wheel;
	}

	void getInfo1(btConstraintInfo1 *info)
	{
		info->m_numConstraintRows = 0;

		btVector3 wheelPosWS = m_rbA.getWorldTransform() * m_pWheel->bodyOffset;
		btVector3 diff = wheelPosWS - m_rbB.getWorldTransform().getOrigin();
		btVector3 downWS = m_rbA.getWorldTransform().getBasis() * m_pWheel->down;
		downWS.normalize();

		// Simulate suspension and engine torque
		info->m_numConstraintRows++;  // suspension

		diff -= downWS * downWS.dot(diff);  // Minus up/down difference
		if (diff.length2() > 0.0005 * 0.0005)
			info->m_numConstraintRows += 2;  // x and z limits
	}

	void getInfo2(btConstraintInfo2 *info)
	{
		int offset = 0;

		btVector3 wheelPosWS = m_rbA.getWorldTransform() * m_pWheel->bodyOffset;
		btVector3 diff = m_rbB.getWorldTransform().getOrigin() - wheelPosWS;

		btVector3 downWS = m_rbA.getWorldTransform().getBasis() * m_pWheel->down;
		downWS.normalize();

		btVector3 axleWS = m_rbA.getWorldTransform().getBasis() * m_pWheel->axle;
		axleWS.normalize();

		btVector3 fwdWS = axleWS.cross(downWS);
		fwdWS.normalize();

		// Solve suspension
		{
			// Dot product between downWS and diff will get us a delta length
			btScalar dist = downWS.dot(diff);
			diff -= downWS * dist;  // Now get rid of this difference

			btScalar suspensionDiff = dist - m_pWheel->suspensionRestLength;

			// Angular axis (rel pos cross normal)
			btVector3 ortho = downWS;
			btVector3 relA = m_pWheel->bodyOffset;
			btVector3 p = ortho.cross(relA);

			(*(btVector3 *)&info->m_J1angularAxis[offset]) = p;

			(*(btVector3 *)&info->m_J1linearAxis[offset]) = downWS;
			(*(btVector3 *)&info->m_J2linearAxis[offset]) = -downWS;

			btScalar rhs = 0;
			if (dist > 0)
			{
				btScalar suspensionForce = m_pWheel->suspensionConstant * suspensionDiff;
				btScalar velFactor = info->fps * m_pWheel->suspensionDamping / btScalar(info->m_numIterations);
				rhs = velFactor * suspensionForce;

				info->m_lowerLimit[offset] = btMax(-m_pWheel->maxSuspensionForce, suspensionForce);
				info->m_upperLimit[offset] = btMin(m_pWheel->maxSuspensionForce, suspensionForce);
			}
			else
			{
				rhs = dist * info->fps * info->erp;
			}

			info->m_constraintError[offset] = rhs;

			offset += info->rowskip;
		}

		// x and z limits
		if (diff.length2() > 0.0005 * 0.0005)
		{
			btScalar fwddiff = fwdWS.dot(diff);
			(*(btVector3 *)&info->m_J1linearAxis[offset]) = fwdWS.normalized();
			(*(btVector3 *)&info->m_J2linearAxis[offset]) = -fwdWS.normalized();

			info->m_constraintError[offset] = fwddiff * info->fps * info->erp / info->m_numIterations;
			info->m_lowerLimit[offset] = -SIMD_INFINITY;
			info->m_upperLimit[offset] = SIMD_INFINITY;

			offset += info->rowskip;

			btScalar sideDiff = axleWS.dot(diff);
			(*(btVector3 *)&info->m_J1linearAxis[offset]) = axleWS.normalized();
			(*(btVector3 *)&info->m_J2linearAxis[offset]) = -axleWS.normalized();

			info->m_constraintError[offset] = sideDiff * info->fps * info->erp / info->m_numIterations;
			info->m_lowerLimit[offset] = -SIMD_INFINITY;
			info->m_upperLimit[offset] = SIMD_INFINITY;

			offset += info->rowskip;
		}
	}

	// Dummy functions
	void setParam(int num, btScalar value, int axis)
	{
	}

	btScalar getParam(int num, int axis) const
	{
		return 666;
	}

	void setWheel(btVehicleWheel *pWheel)
	{
		m_pWheel = pWheel;
	}

	void debugDraw(btIDebugDraw *drawer)
	{
	}

private:
	btVehicleWheel *m_pWheel;
};

btWheeledVehicle::btWheeledVehicle(btDynamicsWorld *pWorld, btRigidBody *pChassis)
{
	btAssert(pWorld);
	btAssert(pChassis);

	m_pWorld = pWorld;
	m_pChassis = pChassis;
}

btWheeledVehicle::~btWheeledVehicle()
{
	for (int i = 0; i < m_wheels.size(); i++)
	{
		m_pWorld->removeConstraint(m_wheels[i].pConstraint);

		m_wheels[i].pConstraint->~btWheelConstraint();
		btAlignedFree(m_wheels[i].pConstraint);
	}
}

btVehicleWheel &btWheeledVehicle::addWheel(btRigidBody *pBody, const btVector3 &bodyOffset, const btMatrix3x3 &basis, const btVector3 &down, const btVector3 &axle)
{
	btVehicleWheel &wheel = m_wheels.expandNonInitializing();
	memset(&wheel, 0, sizeof(btVehicleWheel));

	// We just expanded the array so it may have been realloced, inform the existing constraints
	// (also size-1 because we can ignore the one we just made)
	for (int i = 0; i < m_wheels.size() - 1; i++)
	{
		m_wheels[i].pConstraint->setWheel(&m_wheels[i]);
	}

	wheel.bodyOffset = bodyOffset;
	wheel.basis = basis;
	wheel.down = down;
	wheel.axle = axle;
	wheel.pBody = pBody;

	btWheelConstraint *pConstraint = new (btAlignedAlloc(sizeof(btWheelConstraint), 16)) btWheelConstraint(*m_pChassis, wheel);
	wheel.pConstraint = pConstraint;

	m_pWorld->addConstraint(pConstraint, true);  // Disable collisions between the chassis and the wheel
	return wheel;
}

void btWheeledVehicle::removeWheel(int idx)
{
	if (!checkWheelIndex(idx)) return;

	btVehicleWheel &wheel = m_wheels[idx];
	m_pWorld->removeConstraint(wheel.pConstraint);
}

const btVehicleWheel &btWheeledVehicle::getWheel(int idx) const
{
	checkWheelIndex(idx);

	return m_wheels[idx];
}

btVehicleWheel &btWheeledVehicle::getWheel(int idx)
{
	checkWheelIndex(idx);

	return m_wheels[idx];
}

void btWheeledVehicle::updateAction(btCollisionWorld *pWorld, btScalar step)
{
}

void btWheeledVehicle::debugDraw(btIDebugDraw *pDebugDraw)
{
}
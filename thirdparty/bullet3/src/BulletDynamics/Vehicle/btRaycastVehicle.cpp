/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/

#include "LinearMath/btDefines.h"

#if defined(BT_USE_SSE) && !defined(BT_DEBUG)
// Allow SSE operations because we don't directly use user data.
#define BT_USE_SSE_IN_API
#endif

#include "LinearMath/btVector3.h"
#include "btRaycastVehicle.h"

#include "BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.h"
#include "BulletDynamics/ConstraintSolver/btJacobianEntry.h"
#include "LinearMath/btQuaternion.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "btVehicleRaycaster.h"
#include "btWheelInfo.h"
#include "LinearMath/btMinMax.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletDynamics/ConstraintSolver/btContactConstraint.h"

#define ROLLING_INFLUENCE_FIX

btRaycastVehicle::btRaycastVehicle(const btVehicleTuning& tuning, btRigidBody* chassis, btVehicleRaycaster* raycaster)
	: m_vehicleRaycaster(raycaster),
	  m_pitchControl(btScalar(0.))
{
	m_chassisBody = chassis;
	btAssert(m_chassisBody->getInvMass() != 0);  // You can't use a static object for a vehicle body!

	m_indexForwardAxis = 0;
	m_indexUpAxis = 1;
	m_indexRightAxis = 2;
	defaultInit(tuning);
}

void btRaycastVehicle::defaultInit(const btVehicleTuning& tuning)
{
	(void)tuning;
	m_currentVehicleSpeedKmHour = btScalar(0.);
	m_steeringValue = btScalar(0.);
}

btRaycastVehicle::~btRaycastVehicle()
{
}

//
// basically most of the code is general for 2 or 4 wheel vehicles, but some of it needs to be reviewed
//
btWheelInfo& btRaycastVehicle::addWheel(const btVector3& connectionPointCS, const btVector3& wheelDirectionCS0, const btVector3& wheelAxleCS,
										btScalar suspensionRestLength, btScalar wheelRadius, const btVehicleTuning& tuning, bool isFrontWheel)
{
	btWheelInfoConstructionInfo ci;

	ci.m_chassisConnectionCS = connectionPointCS;
	ci.m_wheelDirectionCS = wheelDirectionCS0;
	ci.m_wheelAxleCS = wheelAxleCS;
	ci.m_suspensionRestLength = suspensionRestLength;
	ci.m_wheelRadius = wheelRadius;
	ci.m_suspensionStiffness = tuning.m_suspensionStiffness;
	ci.m_wheelsDampingCompression = tuning.m_suspensionCompression;
	ci.m_wheelsDampingRelaxation = tuning.m_suspensionDamping;
	ci.m_frictionSlip = tuning.m_frictionSlip;
	ci.m_bIsFrontWheel = isFrontWheel;
	ci.m_maxSuspensionTravelCm = tuning.m_maxSuspensionTravelCm;
	ci.m_maxSuspensionForce = tuning.m_maxSuspensionForce;

	m_wheelInfo.push_back(btWheelInfo(ci));

	btWheelInfo& wheel = m_wheelInfo[getNumWheels() - 1];

	updateWheelTransformsWS(wheel, false);
	updateWheelTransform(getNumWheels() - 1, false);
	return wheel;
}

const btTransform& btRaycastVehicle::getWheelTransformWS(int wheelIndex) const
{
	btAssert(wheelIndex < getNumWheels());
	const btWheelInfo& wheel = m_wheelInfo[wheelIndex];
	return wheel.m_worldTransform;
}

void btRaycastVehicle::updateWheelTransform(int wheelIndex, bool interpolatedTransform)
{
	btWheelInfo& wheel = m_wheelInfo[wheelIndex];
	updateWheelTransformsWS(wheel, interpolatedTransform);

	btVector3 up = -wheel.m_raycastInfo.m_wheelDirectionWS;
	const btVector3& right = wheel.m_raycastInfo.m_wheelAxleWS;
	btVector3 fwd = up.cross(right);
	fwd.normalize();
	//	up = right.cross(fwd);
	//	up.normalize();

	// Steering rotation
	btQuaternion steeringOrn(up, wheel.m_steering);
	btMatrix3x3 steeringMat(steeringOrn);

	// Rotation rotation
	btQuaternion rotatingOrn(right, -wheel.m_rotation);
	btMatrix3x3 rotatingMat(rotatingOrn);

	// TODO: This was different in old repository
	btMatrix3x3 basis2;
	basis2[0][m_indexRightAxis] = -right[0];
	basis2[1][m_indexRightAxis] = -right[1];
	basis2[2][m_indexRightAxis] = -right[2];

	basis2[0][m_indexUpAxis] = up[0];
	basis2[1][m_indexUpAxis] = up[1];
	basis2[2][m_indexUpAxis] = up[2];

	basis2[0][m_indexForwardAxis] = fwd[0];
	basis2[1][m_indexForwardAxis] = fwd[1];
	basis2[2][m_indexForwardAxis] = fwd[2];

	wheel.m_worldTransform.setBasis(steeringMat * rotatingMat * basis2);
	wheel.m_worldTransform.setOrigin(
		wheel.m_raycastInfo.m_hardPointWS + wheel.m_raycastInfo.m_wheelDirectionWS * wheel.m_raycastInfo.m_suspensionLength);
}

void btRaycastVehicle::resetSuspension()
{
	int i;
	for (i = 0; i < m_wheelInfo.size(); i++)
	{
		btWheelInfo& wheel = m_wheelInfo[i];
		wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
		wheel.m_suspensionRelativeVelocity = btScalar(0.0);

		wheel.m_raycastInfo.m_contactNormalWS = -wheel.m_raycastInfo.m_wheelDirectionWS;
		//wheel_info.setContactFriction(btScalar(0.0));
		wheel.m_clippedInvContactDotSuspension = btScalar(1.0);
	}
}

void btRaycastVehicle::updateWheelTransformsWS(btWheelInfo& wheel, bool interpolatedTransform)
{
	wheel.m_raycastInfo.m_isInContact = false;

	btTransform chassisTrans = getChassisWorldTransform();
	if (interpolatedTransform && (getRigidBody()->getMotionState()))
	{
		getRigidBody()->getMotionState()->getWorldTransform(chassisTrans);
	}

	wheel.m_raycastInfo.m_hardPointWS = chassisTrans * wheel.m_chassisConnectionPointCS;
	wheel.m_raycastInfo.m_wheelDirectionWS = chassisTrans.getBasis() * wheel.m_wheelDirectionCS;
	wheel.m_raycastInfo.m_wheelAxleWS = chassisTrans.getBasis() * wheel.m_wheelAxleCS;
}

btScalar btRaycastVehicle::rayCast(int w)
{
	btWheelInfo& wheel = m_wheelInfo[w];

	updateWheelTransformsWS(wheel, false);

	btScalar depth = -1;

	// Suspension rest length goes to center of wheel, so add radius to get to the ground
	btScalar raylen = wheel.getSuspensionRestLength() + wheel.m_wheelsRadius;

	btVector3 rayvector = wheel.m_raycastInfo.m_wheelDirectionWS * (raylen);
	const btVector3& source = wheel.m_raycastInfo.m_hardPointWS;  // Attachment point
	wheel.m_raycastInfo.m_contactPointWS = source + rayvector;
	const btVector3& target = wheel.m_raycastInfo.m_contactPointWS;

	btScalar param = btScalar(0.);

	btVehicleRaycaster::btVehicleRaycasterResult rayResults;

	btAssert(m_vehicleRaycaster);

	void* object = m_vehicleRaycaster->castRay(&wheel, source, target, rayResults);

	wheel.m_raycastInfo.m_groundObject = NULL;

	if (object)
	{
		param = rayResults.m_distFraction;
		depth = raylen * rayResults.m_distFraction;
		wheel.m_raycastInfo.m_contactNormalWS = rayResults.m_hitNormalInWorld;
		wheel.m_raycastInfo.m_isInContact = true;

		//wheel.m_raycastInfo.m_groundObject = &getFixedBody();///@todo for driving on dynamic/movable objects!;
		wheel.m_raycastInfo.m_groundObject = object;

		btScalar hitDistance = param * raylen;
		wheel.m_raycastInfo.m_suspensionLength = hitDistance - wheel.m_wheelsRadius;
		//clamp on max suspension travel

		btScalar minSuspensionLength = wheel.getSuspensionRestLength() - wheel.m_maxSuspensionTravelCm * btScalar(0.01);
		btScalar maxSuspensionLength = wheel.getSuspensionRestLength() + wheel.m_maxSuspensionTravelCm * btScalar(0.01);
		if (wheel.m_raycastInfo.m_suspensionLength < minSuspensionLength)
		{
			wheel.m_raycastInfo.m_suspensionLength = minSuspensionLength;
		}
		if (wheel.m_raycastInfo.m_suspensionLength > maxSuspensionLength)
		{
			wheel.m_raycastInfo.m_suspensionLength = maxSuspensionLength;
		}

		wheel.m_raycastInfo.m_contactPointWS = rayResults.m_hitPointInWorld;

		btScalar denominator = wheel.m_raycastInfo.m_contactNormalWS.dot(wheel.m_raycastInfo.m_wheelDirectionWS);

		btVector3 chassis_velocity_at_contactPoint;
		btVector3 relpos = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();

		chassis_velocity_at_contactPoint = getRigidBody()->getVelocityInLocalPoint(relpos);

		btScalar projVel = wheel.m_raycastInfo.m_contactNormalWS.dot(chassis_velocity_at_contactPoint);

		if (denominator >= btScalar(-0.1))
		{
			wheel.m_suspensionRelativeVelocity = btScalar(0.0);
			wheel.m_clippedInvContactDotSuspension = btScalar(1.0) / btScalar(0.1);
		}
		else
		{
			btScalar inv = btScalar(-1.) / denominator;
			wheel.m_suspensionRelativeVelocity = projVel * inv;
			wheel.m_clippedInvContactDotSuspension = inv;
		}
	}
	else
	{
		//put wheel info as in rest position
		wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
		wheel.m_suspensionRelativeVelocity = btScalar(0.0);
		wheel.m_raycastInfo.m_contactNormalWS = -wheel.m_raycastInfo.m_wheelDirectionWS;
		wheel.m_clippedInvContactDotSuspension = btScalar(1.0);
	}

	return depth;
}

const btTransform btRaycastVehicle::getChassisWorldTransform() const
{
	/*if (getRigidBody()->getMotionState())
	{
		btTransform chassisWorldTrans;
		getRigidBody()->getMotionState()->getWorldTransform(chassisWorldTrans);
		return chassisWorldTrans;
	}
	*/

	return getRigidBody()->getCenterOfMassTransform();
}

void btRaycastVehicle::updateVehicle(btScalar step)
{
	for (int i = 0; i < getNumWheels(); i++)
	{
		updateWheelTransform(i, false);
	}

	m_currentVehicleSpeedKmHour = btScalar(3.6) * getRigidBody()->getLinearVelocity().length();

	const btTransform& chassisTrans = getChassisWorldTransform();

	btVector3 forwardW(
		chassisTrans.getBasis()[0][m_indexForwardAxis],
		chassisTrans.getBasis()[1][m_indexForwardAxis],
		chassisTrans.getBasis()[2][m_indexForwardAxis]);

	if (forwardW.dot(getRigidBody()->getLinearVelocity()) < btScalar(0.))
	{
		m_currentVehicleSpeedKmHour *= btScalar(-1.);
	}

	//
	// simulate suspension
	//

	int i = 0;
	for (i = 0; i < m_wheelInfo.size(); i++)
	{
		btScalar depth;
		depth = rayCast(i);
	}

	updateSuspension(step);

	// Apply the suspension (why not apply it in updateSuspension like updateFriction does?)
	for (i = 0; i < m_wheelInfo.size(); i++)
	{
		//apply suspension force
		btWheelInfo& wheel = m_wheelInfo[i];

		btScalar suspensionForce = wheel.m_wheelsSuspensionForce;

		// Clamp it
		if (suspensionForce > wheel.m_maxSuspensionForce)
		{
			suspensionForce = wheel.m_maxSuspensionForce;
		}

		btRigidBody* pGround = (btRigidBody*)wheel.m_raycastInfo.m_groundObject;

		btVector3 impulse = wheel.m_raycastInfo.m_contactNormalWS * suspensionForce * step;
		btVector3 relpos = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();

		m_chassisBody->applyImpulse(impulse, relpos);

		if (pGround && !pGround->isStaticOrKinematicObject())
		{
			btVector3 relPosGround = wheel.m_raycastInfo.m_contactPointWS - pGround->getCenterOfMassPosition();
			btVector3 impulseGround = impulse * m_chassisBody->getInvMass();
			impulseGround *= 1 / pGround->getInvMass();

			pGround->applyImpulse(-impulseGround, relPosGround);
		}
	}

	updateFriction(step);

	for (i = 0; i < m_wheelInfo.size(); i++)
	{
		btWheelInfo& wheel = m_wheelInfo[i];
		btVector3 relpos = wheel.m_raycastInfo.m_hardPointWS - getRigidBody()->getCenterOfMassPosition();
		btVector3 vel = getRigidBody()->getVelocityInLocalPoint(relpos);

		if (!vel.fuzzyZero() && wheel.m_raycastInfo.m_isInContact)
		{
			btVector3 relvel = vel;

			// Spin the wheels relative to the ground velocity
			btRigidBody* pOther = btRigidBody::upcast((btCollisionObject*)wheel.m_raycastInfo.m_groundObject);
			if (pOther)
			{
				btVector3 groundVel = pOther->getVelocityInLocalPoint(wheel.m_raycastInfo.m_contactPointWS - pOther->getWorldTransform().getOrigin());
				relvel -= groundVel;
			}

			// Credit: https://code.google.com/p/bullet/issues/detail?id=714
			btVector3 up = -wheel.m_raycastInfo.m_wheelDirectionWS;
			const btVector3& right = wheel.m_raycastInfo.m_wheelAxleWS;
			btVector3 fwd = up.cross(right);

			// Rotate by steering angle
			btQuaternion steeringOrn(up, wheel.m_steering);
			fwd = btTransform(steeringOrn) * fwd;
			fwd.normalize();

			btScalar proj = fwd.dot(wheel.m_raycastInfo.m_contactNormalWS);
			fwd -= wheel.m_raycastInfo.m_contactNormalWS * proj;

			btScalar proj2 = fwd.dot(relvel);
			btScalar delta = (proj2 * step) / wheel.m_wheelsRadius;

			if (!btFuzzyZero(delta))
			{
				wheel.m_deltaRotation = delta;
				wheel.m_rotation += wheel.m_deltaRotation;
			}
		}
		else
		{
			wheel.m_rotation += wheel.m_deltaRotation;
		}

		// Normalize it so we don't get floating point issues
		wheel.m_rotation = btNormalizeAngle(wheel.m_rotation);

		//wheel.m_deltaRotation *= wheel.m_rotationDamping;
		wheel.m_deltaRotation *= (btScalar(1.f) - (wheel.m_rotationDamping * step));
	}
}

void btRaycastVehicle::setSteeringValue(btScalar steering, int wheel)
{
	btAssert(wheel >= 0 && wheel < getNumWheels());

	btWheelInfo& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_steering = steering;
}

btScalar btRaycastVehicle::getSteeringValue(int wheel) const
{
	return getWheelInfo(wheel).m_steering;
}

void btRaycastVehicle::applyEngineForce(btScalar force, int wheel)
{
	btAssert(wheel >= 0 && wheel < getNumWheels());
	btWheelInfo& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_engineForce = force;
}

const btWheelInfo& btRaycastVehicle::getWheelInfo(int index) const
{
	btAssert((index >= 0) && (index < getNumWheels()));

	return m_wheelInfo[index];
}

btWheelInfo& btRaycastVehicle::getWheelInfo(int index)
{
	btAssert((index >= 0) && (index < getNumWheels()));

	return m_wheelInfo[index];
}

void btRaycastVehicle::setBrake(btScalar brake, int wheelIndex)
{
	btAssert((wheelIndex >= 0) && (wheelIndex < getNumWheels()));
	getWheelInfo(wheelIndex).m_brake = brake;
}

void btRaycastVehicle::updateSuspension(btScalar deltaTime)
{
	(void)deltaTime;

	btScalar chassisMass = btScalar(1.) / m_chassisBody->getInvMass();

	for (int w_it = 0; w_it < getNumWheels(); w_it++)
	{
		btWheelInfo& wheel_info = m_wheelInfo[w_it];

		if (wheel_info.m_raycastInfo.m_isInContact)
		{
			btScalar force;
			//	Spring
			{
				btScalar susp_length = wheel_info.getSuspensionRestLength();
				btScalar current_length = wheel_info.m_raycastInfo.m_suspensionLength;

				btScalar length_diff = (susp_length - current_length);

				force = wheel_info.m_suspensionStiffness * length_diff * wheel_info.m_clippedInvContactDotSuspension;
			}

			// Damper
			{
				btScalar projected_rel_vel = wheel_info.m_suspensionRelativeVelocity;
				{
					btScalar susp_damping;
					if (projected_rel_vel < btScalar(0.0))
					{
						susp_damping = wheel_info.m_wheelsDampingCompression;
					}
					else
					{
						susp_damping = wheel_info.m_wheelsDampingRelaxation;
					}
					force -= susp_damping * projected_rel_vel;
				}
			}

			// RESULT
			wheel_info.m_wheelsSuspensionForce = force * chassisMass;
			if (wheel_info.m_wheelsSuspensionForce < btScalar(0.))
			{
				wheel_info.m_wheelsSuspensionForce = btScalar(0.);
			}
		}
		else
		{
			wheel_info.m_wheelsSuspensionForce = btScalar(0.0);
		}
	}
}

struct btWheelContactPoint
{
	btRigidBody* m_body0;
	btRigidBody* m_body1;
	btVector3 m_frictionPositionWorld;
	btVector3 m_frictionDirectionWorld;
	btScalar m_jacDiagABInv;
	btScalar m_maxImpulse;

	btWheelContactPoint(btRigidBody* body0, btRigidBody* body1, const btVector3& frictionPosWorld, const btVector3& frictionDirectionWorld, btScalar maxImpulse)
		: m_body0(body0),
		  m_body1(body1),
		  m_frictionPositionWorld(frictionPosWorld),
		  m_frictionDirectionWorld(frictionDirectionWorld),
		  m_maxImpulse(maxImpulse)
	{
		btScalar denom0 = body0->computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
		btScalar denom1 = body1->computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
		btScalar relaxation = 1.f;
		m_jacDiagABInv = relaxation / (denom0 + denom1);
	}
};

btScalar calcRollingFriction(btWheelContactPoint& contactPoint, int numWheelsOnGround)
{
	btScalar j1 = 0.f;

	const btVector3& contactPosWorld = contactPoint.m_frictionPositionWorld;

	btVector3 rel_pos1 = contactPosWorld - contactPoint.m_body0->getCenterOfMassPosition();
	btVector3 rel_pos2 = contactPosWorld - contactPoint.m_body1->getCenterOfMassPosition();

	btScalar maxImpulse = contactPoint.m_maxImpulse;

	btVector3 vel1 = contactPoint.m_body0->getVelocityInLocalPoint(rel_pos1);
	btVector3 vel2 = contactPoint.m_body1->getVelocityInLocalPoint(rel_pos2);
	btVector3 vel = vel1 - vel2;

	btScalar vrel = contactPoint.m_frictionDirectionWorld.dot(vel);

	// calculate j that moves us to zero relative velocity
	j1 = -vrel * contactPoint.m_jacDiagABInv / btScalar(numWheelsOnGround);
	btSetMin(j1, maxImpulse);
	btSetMax(j1, -maxImpulse);

	return j1;
}

btScalar sideFrictionStiffness2 = btScalar(1.0);
void btRaycastVehicle::updateFriction(btScalar timeStep)
{
	//calculate the impulse, so that the wheels don't move sidewards
	int numWheel = getNumWheels();
	if (!numWheel)
		return;

	m_forwardWS.resize(numWheel);
	m_axle.resize(numWheel);
	m_forwardImpulse.resize(numWheel);
	m_sideImpulse.resize(numWheel);

	for (int i = 0; i < getNumWheels(); i++)
	{
		// Clear the impulses
		m_sideImpulse[i] = btScalar(0.);
		m_forwardImpulse[i] = btScalar(0.);

		btWheelInfo& wheelInfo = m_wheelInfo[i];

		btRigidBody* groundObject = (btRigidBody*)wheelInfo.m_raycastInfo.m_groundObject;

		if (groundObject)
		{
			const btTransform& wheelTrans = getWheelTransformWS(i);

			//m_axle[i] = wheelInfo.m_raycastInfo.m_wheelAxleWS;

			btMatrix3x3 wheelBasis0 = wheelTrans.getBasis();
			m_axle[i] = -btVector3(
				wheelBasis0[0][m_indexRightAxis],
				wheelBasis0[1][m_indexRightAxis],
				wheelBasis0[2][m_indexRightAxis]);

			const btVector3& surfNormalWS = wheelInfo.m_raycastInfo.m_contactNormalWS;
			btScalar proj = m_axle[i].dot(surfNormalWS);
			m_axle[i] -= surfNormalWS * proj;
			m_axle[i] = m_axle[i].normalize();

			m_forwardWS[i] = surfNormalWS.cross(m_axle[i]);
			m_forwardWS[i].normalize();

			resolveSingleBilateral(*m_chassisBody, wheelInfo.m_raycastInfo.m_contactPointWS,
								   *groundObject, wheelInfo.m_raycastInfo.m_contactPointWS,
								   btScalar(0.), m_axle[i], m_sideImpulse[i], timeStep);

			m_sideImpulse[i] *= sideFrictionStiffness2;
		}
	}

	btScalar sideFactor = btScalar(1.);
	btScalar fwdFactor = 0.5;

	bool sliding = false;
	for (int wheel = 0; wheel < getNumWheels(); wheel++)
	{
		btWheelInfo& wheelInfo = m_wheelInfo[wheel];
		class btRigidBody* groundObject = (class btRigidBody*)wheelInfo.m_raycastInfo.m_groundObject;

		//switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)

		m_forwardImpulse[wheel] = btScalar(0.);
		m_wheelInfo[wheel].m_skidInfo = btScalar(1.);

		if (groundObject)
		{
			btScalar maximp = wheelInfo.m_wheelsSuspensionForce * timeStep * wheelInfo.m_frictionSlip;  // Normal impulse * coeff of friction
			btScalar maximpSide = maximp;

			btScalar maximpSquared = maximp * maximpSide;

			// Calculate the forward impulse
			if (!btFuzzyZero(wheelInfo.m_engineForce))
			{
				m_forwardImpulse[wheel] = wheelInfo.m_engineForce * timeStep;
			}
			else
			{
				static const btScalar defaultRollingFrictionImpulse = 0.f;
				btScalar maxImpulse = wheelInfo.m_brake ? wheelInfo.m_brake : defaultRollingFrictionImpulse;
				btWheelContactPoint contactPt(m_chassisBody, groundObject, wheelInfo.m_raycastInfo.m_contactPointWS, m_forwardWS[wheel], maxImpulse);
				// btAssert(numWheelsOnGround > 0);
				m_forwardImpulse[wheel] = calcRollingFriction(contactPt, getNumWheels()) / btScalar(getNumWheels());
			}

			btScalar x = (m_forwardImpulse[wheel]) * fwdFactor;
			btScalar y = (m_sideImpulse[wheel]) * sideFactor;

			btScalar impulseSquared = (x * x + y * y);

			if (impulseSquared > maximpSquared)
			{
				sliding = true;

				btScalar factor = maximp / btSqrt(impulseSquared);

				m_wheelInfo[wheel].m_skidInfo *= factor;
			}
		}
	}

	if (sliding)
	{
		for (int wheel = 0; wheel < getNumWheels(); wheel++)
		{
			if (m_sideImpulse[wheel] != btScalar(0.))
			{
				if (m_wheelInfo[wheel].m_skidInfo < btScalar(1.))
				{
					m_forwardImpulse[wheel] *= m_wheelInfo[wheel].m_skidInfo;
					m_sideImpulse[wheel] *= m_wheelInfo[wheel].m_skidInfo;
				}
			}
		}
	}

	// apply the impulses
	for (int wheel = 0; wheel < getNumWheels(); wheel++)
	{
		btWheelInfo& wheelInfo = m_wheelInfo[wheel];

		btVector3 rel_pos = wheelInfo.m_raycastInfo.m_contactPointWS -
							m_chassisBody->getCenterOfMassPosition();

		if (m_forwardImpulse[wheel] != btScalar(0.))
		{
			btRigidBody* pGround = (btRigidBody*)m_wheelInfo[wheel].m_raycastInfo.m_groundObject;

			btVector3 impulse = m_forwardWS[wheel] * m_forwardImpulse[wheel];
			m_chassisBody->applyImpulse(impulse, rel_pos);

			// Apply an equal force against the ground
			if (pGround && !pGround->isStaticOrKinematicObject())
			{
				btVector3 relPosGround = wheelInfo.m_raycastInfo.m_contactPointWS - pGround->getCenterOfMassPosition();
				btVector3 impulseGround = impulse * m_chassisBody->getInvMass();
				impulseGround *= 1 / pGround->getInvMass();

				pGround->applyImpulse(-impulseGround, relPosGround);
			}
		}

		if (m_sideImpulse[wheel] != btScalar(0.))
		{
			btVector3 sideImp = wheelInfo.m_raycastInfo.m_wheelAxleWS * m_sideImpulse[wheel];

#if defined ROLLING_INFLUENCE_FIX  // fix. It only worked if car's up was along Y - VT.
			btVector3 vChassisWorldUp = getRigidBody()->getCenterOfMassTransform().getBasis().getColumn(m_indexUpAxis);
			rel_pos -= vChassisWorldUp * (vChassisWorldUp.dot(rel_pos) * (1.f - wheelInfo.m_rollInfluence));
#else
			rel_pos[m_indexUpAxis] *= wheelInfo.m_rollInfluence;
#endif
			m_chassisBody->applyImpulse(sideImp, rel_pos);

			btRigidBody* groundObject = (btRigidBody*)m_wheelInfo[wheel].m_raycastInfo.m_groundObject;

			if (groundObject && !groundObject->isStaticOrKinematicObject())
			{
				btVector3 rel_pos2 = wheelInfo.m_raycastInfo.m_contactPointWS -
									 groundObject->getCenterOfMassPosition();

				btVector3 sideImp2 = sideImp * m_chassisBody->getInvMass();
				sideImp2 *= 1 / groundObject->getInvMass();

				//apply friction impulse on the ground
				groundObject->applyImpulse(-sideImp2, rel_pos2);
			}
		}
	}
}

void btRaycastVehicle::debugDraw(btIDebugDraw* debugDrawer)
{
	for (int v = 0; v < this->getNumWheels(); v++)
	{
		btVector3 wheelColor(0, 1, 1);
		if (getWheelInfo(v).m_raycastInfo.m_isInContact)
		{
			wheelColor.setValue(0, 0, 1);
		}
		else
		{
			wheelColor.setValue(1, 0, 1);
		}

		btVector3 wheelPosWS = getWheelInfo(v).m_worldTransform.getOrigin();

		btVector3 axle = btVector3(
			getWheelInfo(v).m_worldTransform.getBasis()[0][getRightAxis()],
			getWheelInfo(v).m_worldTransform.getBasis()[1][getRightAxis()],
			getWheelInfo(v).m_worldTransform.getBasis()[2][getRightAxis()]);

		//debug wheels (cylinders)
		debugDrawer->drawLine(wheelPosWS, wheelPosWS + axle, wheelColor);
		debugDrawer->drawLine(wheelPosWS, getWheelInfo(v).m_raycastInfo.m_contactPointWS, wheelColor);
	}
}

void* btDefaultVehicleRaycaster::castRay(btWheelInfo* wheel, const btVector3& from, const btVector3& to, btVehicleRaycasterResult& result)
{
	//	RayResultCallback& resultCallback;

	btCollisionWorld::ClosestRayResultCallback rayCallback(from, to);

	m_dynamicsWorld->rayTest(from, to, rayCallback);

	if (rayCallback.hasHit())
	{
		const btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
		if (body && body->hasContactResponse())
		{
			result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
			result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
			result.m_hitNormalInWorld.normalize();
			result.m_distFraction = rayCallback.m_closestHitFraction;
			return (void*)body;
		}
	}

	return 0;
}

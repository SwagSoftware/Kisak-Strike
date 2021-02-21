#include "StdAfx.h"

#include "Physics_Object.h"
#include "Physics_Constraint.h"
#include "Physics_Environment.h"

#include "convert.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

/**********************
* Misc. Functions
**********************/
// I hope these work!
inline void TransformWorldToLocal(const btTransform &trans, const btRigidBody &obj, btTransform &out) {
	out = trans * obj.getWorldTransform().inverse();
}

inline void TransformLocalToWorld(const btTransform &trans, const btRigidBody &obj, btTransform &out) {
	out = trans * obj.getWorldTransform();
}

inline void GraphicTransformWorldToLocal(const btTransform &trans, const btRigidBody &obj, btTransform &out) {
	out = (trans * ((btMassCenterMotionState *)obj.getMotionState())->m_centerOfMassOffset) * obj.getWorldTransform();
}

inline void GraphicTransformLocalToWorld(const btTransform &trans, const btRigidBody &obj, btTransform &out) {
	out = (trans  * ((btMassCenterMotionState *)obj.getMotionState())->m_centerOfMassOffset) * obj.getWorldTransform();
}

// Convert an axis to a matrix (where angle around axis does not matter)
// Axis will be assumed as the forward vector (and assumed normalized)
void bullAxisToMatrix(const btVector3 &axis, btMatrix3x3 &matrix) {
	btVector3 wup(0, 1, 0);

	// Handle cases where the axis is really close to up/down vector
	// Dot = cos(theta) (for dummies), looking for 0(1) to 180(-1) degrees difference
	btScalar dot = wup.dot(axis);
	if ((dot > 1.0f - SIMD_EPSILON) || (dot < -1.0f + SIMD_EPSILON)) {
		// Axis is really close to/is the up/down vector! We'll have to use a side vector as a base instead.
		btVector3 wside(0, 0, 1);

		// Normalize the cross products, as they may not be unit length.
		btVector3 up = wside.cross(axis);
		up.normalize();

		btVector3 side = axis.cross(up);
		side.normalize();

		// Fwd Up Right
		matrix.setValue(axis.x(), up.x(), side.x(),
						axis.y(), up.y(), side.y(),
						axis.z(), up.z(), side.z());
	} else {
		// Cross products may not be unit length!
		btVector3 side = wup.cross(axis);
		side.normalize();

		btVector3 up = side.cross(axis);
		up.normalize();

		matrix.setValue(axis.x(), up.x(), side.x(),
						axis.y(), up.y(), side.y(),
						axis.z(), up.z(), side.z());
	}
}

const char *GetConstraintName(EConstraintType type) {
	switch (type) {
		case CONSTRAINT_UNKNOWN:
			return "unknown";
		case CONSTRAINT_RAGDOLL:
			return "ragdoll";
		case CONSTRAINT_HINGE:
			return "hinge";
		case CONSTRAINT_FIXED:
			return "fixed";
		case CONSTRAINT_SLIDING:
			return "sliding";
		case CONSTRAINT_BALLSOCKET:
			return "ballsocket";
		case CONSTRAINT_PULLEY:
			return "pulley";
		case CONSTRAINT_LENGTH:
			return "length";
		case CONSTRAINT_SPRING:
			return "spring";
		default:
			return "invalid";
	}
}

/***********************
* BULLET CONSTRAINTS
***********************/
// TODO: Somebody fill this out!
class btPulleyConstraint: public btTypedConstraint {
	protected:
		// The 2 points the pulley should run through in world space (such as being attached to a roof)
		btVector3	m_attachPointWS1; // Directly attached to object A
		btVector3	m_attachPointWS2; // Directly attached to object B

		// The attachment points on the 2 objects in local space
		btVector3	m_attachPointA;
		btVector3	m_attachPointB;

		btScalar	m_totalLength; // Length including gearing
		btScalar	m_gearRatio; // Affects attached object ALWAYS

	public:
		btPulleyConstraint(btRigidBody &rbA, btRigidBody &rbB, const btVector3 &pivotInA, const btVector3 &pivotInB, const btVector3 &attachWS1, const btVector3 &attachWS2, 
							btScalar totalLength, btScalar gearRatio):
							btTypedConstraint(CONSTRAINT_TYPE_USER, rbA, rbB) {
			m_attachPointA = pivotInA;
			m_attachPointB = pivotInB;

			m_attachPointWS1 = attachWS1;
			m_attachPointWS2 = attachWS2;
		}

		void getInfo1(btConstraintInfo1 *info) {
			// 2 constraint rows, one for each object
			info->m_numConstraintRows = 2;
			info->nub = 4;
		}

		void getInfo2(btConstraintInfo2 *info) {

		}

		void debugDraw(btIDebugDraw *pDrawer) {
			btVector3 worldA, worldB;
			worldA = m_rbA.getWorldTransform() * m_attachPointA;
			worldB = m_rbB.getWorldTransform() * m_attachPointB;

			// Draw the transforms on the objects
			pDrawer->drawTransform(btTransform(btMatrix3x3::getIdentity(), worldA), 1);
			pDrawer->drawTransform(btTransform(btMatrix3x3::getIdentity(), worldB), 1);

			// World transforms
			pDrawer->drawTransform(btTransform(btMatrix3x3::getIdentity(), m_attachPointWS1), 1);
			pDrawer->drawTransform(btTransform(btMatrix3x3::getIdentity(), m_attachPointWS2), 1);
		}

		// These functions do absolutely nothing!

		btScalar getParam(int num, int axis = -1) const {
			return 0xB16B00B5;
		}

		void setParam(int num, btScalar value, int axis = -1) {

		}
};

class btLengthConstraint: public btPoint2PointConstraint {
	protected:
		btScalar	m_mindist;
		btScalar	m_maxdist;
	public:
		btLengthConstraint(btRigidBody &rbA, btRigidBody &rbB, const btVector3 &pivotInA, const btVector3 &pivotInB, btScalar minDist, btScalar maxDist):
		btPoint2PointConstraint(rbA, rbB, pivotInA, pivotInB) {
			m_mindist = minDist;
			m_maxdist = maxDist;
		}

		void getInfo1(btConstraintInfo1 *info) {
			// Positions relative to objects
			btVector3 relA = m_rbA.getCenterOfMassTransform().getBasis() * getPivotInA();
			btVector3 relB = m_rbB.getCenterOfMassTransform().getBasis() * getPivotInB();

			// Exact world positions
			btVector3 posA = m_rbA.getCenterOfMassTransform().getOrigin() + relA;
			btVector3 posB = m_rbB.getCenterOfMassTransform().getOrigin() + relB;

			// Delta
			btVector3 del = posB - posA;
			btScalar currDist = del.length();

			info->m_numConstraintRows = 0;
			info->nub = 6; // FIXME: What does this even do?

			// Only need to solve the constraint if there is any error!
			if (currDist < m_mindist || currDist > m_maxdist) {
				info->m_numConstraintRows++;
				info->nub--;
			}
		}

		void getInfo2(btConstraintInfo2 *info) {
			// Positions relative to objects
			btVector3 relA = m_rbA.getCenterOfMassTransform().getBasis() * getPivotInA();
			btVector3 relB = m_rbB.getCenterOfMassTransform().getBasis() * getPivotInB();

			// Exact world positions
			btVector3 posA = m_rbA.getCenterOfMassTransform().getOrigin() + relA;
			btVector3 posB = m_rbB.getCenterOfMassTransform().getOrigin() + relB;

			// Delta
			btVector3 del = posB - posA;
			btScalar currDist = del.length();

			btVector3 ortho = del / currDist; // Axis to solve along (normalized delta)
			// Linear axis for ref object(?)
			info->m_J1linearAxis[0] = ortho[0];
			info->m_J1linearAxis[1] = ortho[1];
			info->m_J1linearAxis[2] = ortho[2];
			
			// Linear axis for att object(?)
			if (info->m_J2linearAxis) {
				info->m_J2linearAxis[0] = -ortho[0];
				info->m_J2linearAxis[1] = -ortho[1];
				info->m_J2linearAxis[2] = -ortho[2];
			}

			// Angular axis (relative pos cross normal)
			btVector3 p, q;
			p = relA.cross(ortho);
			q = relB.cross(ortho);
			info->m_J1angularAxis[0] = p[0];
			info->m_J1angularAxis[1] = p[1];
			info->m_J1angularAxis[2] = p[2];

			if (info->m_J2angularAxis) {
				info->m_J2angularAxis[0] = -q[0];
				info->m_J2angularAxis[1] = -q[1];
				info->m_J2angularAxis[2] = -q[2];
			}

			btScalar rhs = 0;

			info->m_lowerLimit[0] = 0;
			info->m_upperLimit[0] = 0;

			// Keep the distance between min and max dist
			if (currDist < m_mindist) {
				rhs = (currDist - m_mindist) * info->fps * info->erp;
				info->m_lowerLimit[0] = -SIMD_INFINITY;
			} else if (currDist > m_maxdist) {
				rhs = (currDist - m_maxdist) * info->fps * info->erp;
				info->m_upperLimit[0] = SIMD_INFINITY;
			}

			info->m_constraintError[0] = rhs; // Constraint error (target rel velocity)
			info->cfm[0] = btScalar(0.f);
		}
};

class btSpringConstraint : public btPoint2PointConstraint {
	protected:
		btScalar	m_length; // Natural length (delta x = 0)
		btScalar	m_constant; // Spring constant (F=kx)
		btScalar	m_maxForce; // max force (0 for disable)
		btScalar	m_damping;

		bool		m_onlyStretch; // Only apply forces if the spring is stretched

		void wakeObjects() {
			m_rbA.activate();
			m_rbB.activate();
		}
	public:
		btSpringConstraint(btRigidBody &rbA, btRigidBody &rbB, const btVector3 &pivotInA, const btVector3 &pivotInB, btScalar length, btScalar constant, bool onlyStretch, btScalar damping=1, btScalar maxForce=0) :
		btPoint2PointConstraint(rbA, rbB, pivotInA, pivotInB) {
			m_length = length;
			m_constant = constant;
			m_onlyStretch = onlyStretch;
			m_damping = damping;
			m_maxForce = maxForce;
		}

		void setLength(btScalar len) {
			m_length = len;
			wakeObjects();
		}

		void setConstant(btScalar constant) {
			m_constant = constant;
			wakeObjects();
		}

		void setMaxForce(btScalar maxForce) {
			m_maxForce = maxForce;
		}

		void setDamping(btScalar damping) {
			m_damping = damping;
			wakeObjects();
		}

		void getInfo1(btConstraintInfo1 *info) {
			// Positions relative to objects
			btVector3 relA = m_rbA.getCenterOfMassTransform().getBasis() * getPivotInA();
			btVector3 relB = m_rbB.getCenterOfMassTransform().getBasis() * getPivotInB();

			// Exact world positions
			btVector3 posA = m_rbA.getCenterOfMassTransform().getOrigin() + relA;
			btVector3 posB = m_rbB.getCenterOfMassTransform().getOrigin() + relB;

			// Delta
			btVector3 del = posB - posA;
			btScalar currDist = del.length();

			info->m_numConstraintRows = 0;
			info->nub = 6; // FIXME: What does this even do?

			// Only solve the constraint if needed
			if ((currDist > m_length) || (!m_onlyStretch && currDist < m_length)) {
				info->m_numConstraintRows++;
				info->nub--;
			}
		}

		// FYI: This isn't called if getInfo1 returns 0 rows or less
		void getInfo2(btConstraintInfo2 *info) {
			// Positions relative to objects
			btVector3 relA = m_rbA.getCenterOfMassTransform().getBasis() * getPivotInA();
			btVector3 relB = m_rbB.getCenterOfMassTransform().getBasis() * getPivotInB();

			// Exact world positions
			btVector3 posA = m_rbA.getCenterOfMassTransform().getOrigin() + relA;
			btVector3 posB = m_rbB.getCenterOfMassTransform().getOrigin() + relB;

			btVector3 velA = m_rbA.getLinearVelocity();
			btVector3 velB = m_rbB.getLinearVelocity();
			btVector3 relVel = velB - velA;

			// Delta position
			btVector3 del = posB - posA;
			btScalar currDist = del.length();

			btVector3 ortho = del / currDist; // Axis to solve along (normalized delta)
			// Linear axis for ref object(?)
			info->m_J1linearAxis[0] = ortho[0];
			info->m_J1linearAxis[1] = ortho[1];
			info->m_J1linearAxis[2] = ortho[2];

			// Linear axis for att object(?)
			if (info->m_J2linearAxis) {
				info->m_J2linearAxis[0] = -ortho[0];
				info->m_J2linearAxis[1] = -ortho[1];
				info->m_J2linearAxis[2] = -ortho[2];
			}

			// Angular axis (relative pos cross normal)
			btVector3 p, q;
			p = relA.cross(ortho);
			q = relB.cross(ortho);
			info->m_J1angularAxis[0] = p[0];
			info->m_J1angularAxis[1] = p[1];
			info->m_J1angularAxis[2] = p[2];

			if (info->m_J2angularAxis) {
				info->m_J2angularAxis[0] = -q[0];
				info->m_J2angularAxis[1] = -q[1];
				info->m_J2angularAxis[2] = -q[2];
			}

			btScalar rhs = 0;

			info->m_lowerLimit[0] = 0;
			info->m_upperLimit[0] = 0;

			// Always solve for stretching, push objects away if onlyStretch is false
			if (currDist > m_length || (!m_onlyStretch && currDist < m_length)) {
				btScalar deltaX = currDist - m_length;

				// F=kx (Hooke's law)
				btScalar force = m_constant * deltaX;
				btScalar velFactor = info->fps * m_damping / btScalar(info->m_numIterations);

				rhs = velFactor * force;

				if (m_maxForce) {
					if (deltaX < 0) {
						info->m_lowerLimit[0] = -m_maxForce * info->fps;
					} else {
						info->m_upperLimit[0] = m_maxForce * info->fps;
					}
				} else {
					btScalar maxForce = btFabs(force) / info->fps; // Change it into an impulse.

					info->m_lowerLimit[0] = -maxForce;
					info->m_upperLimit[0] =  maxForce;
				}
			}

			// Upper/Lower limits are impulses

			info->m_constraintError[0] = rhs; // Constraint error (target rel velocity)
			info->cfm[0] = btScalar(0.f);
		}
};

// Purpose: Bind IPhysicsUserConstraint to a bullet constraint
class btUserConstraint : public btTypedConstraint {
	public:
		btUserConstraint(btRigidBody &rbA, btRigidBody &rbB, IPhysicsUserConstraint *pConstraint): btTypedConstraint(CONSTRAINT_TYPE_USER, rbA, rbB) {
			m_pUserConstraint = pConstraint;
		}

		void getInfo1(btConstraintInfo1 *pInfo) {
			physconstraintinfo_t info;
			info.numConstraintRows = 0;
			info.nub = 0;

			CPhysicsObject *pObjA, *pObjB;
			pObjA = (CPhysicsObject *)m_rbA.getUserPointer();
			pObjB = (CPhysicsObject *)m_rbB.getUserPointer();
			m_pUserConstraint->GetConstraintInfo(pObjA, pObjB, info);

			pInfo->m_numConstraintRows = info.numConstraintRows;
			pInfo->nub = info.nub;
		}

		void getInfo2(btConstraintInfo2 *pInfo) {
			physconstraintsolveinfo_t *solveinfo = new physconstraintsolveinfo_t[pInfo->m_numConstraintRows];
			for (int i = 0; i < pInfo->m_numConstraintRows; i++) {
				solveinfo[i].Defaults();
			}

			CPhysicsObject *pObjA, *pObjB;
			pObjA = (CPhysicsObject *)m_rbA.getUserPointer();
			pObjB = (CPhysicsObject *)m_rbB.getUserPointer();
			m_pUserConstraint->GetConstraintSolveInfo(pObjA, pObjB, solveinfo, pInfo->m_numConstraintRows, pInfo->fps, pInfo->erp);

			for (int i = 0; i < pInfo->m_numConstraintRows; i++) {
				ConvertDirectionToBull(solveinfo[i].J1linearAxis, *(btVector3 *)&pInfo->m_J1linearAxis[i * pInfo->rowskip]);
				ConvertDirectionToBull(solveinfo[i].J1angularAxis, *(btVector3 *)&pInfo->m_J1angularAxis[i * pInfo->rowskip]);

				if (pInfo->m_J2linearAxis)
					ConvertDirectionToBull(solveinfo[i].J2linearAxis, *(btVector3 *)&pInfo->m_J2linearAxis[i * pInfo->rowskip]);

				if (pInfo->m_J2angularAxis)
					ConvertDirectionToBull(solveinfo[i].J2angularAxis, *(btVector3 *)&pInfo->m_J2angularAxis[i * pInfo->rowskip]);

				pInfo->cfm[i * pInfo->rowskip] = solveinfo[i].cfm;
				pInfo->m_constraintError[i * pInfo->rowskip] = solveinfo[i].constraintError;

				pInfo->m_lowerLimit[i * pInfo->rowskip] = solveinfo[i].lowerLimit;
				pInfo->m_upperLimit[i * pInfo->rowskip] = solveinfo[i].upperLimit;
			}

			delete[] solveinfo;
		}

		IPhysicsUserConstraint *getUserConstraint() {
			return m_pUserConstraint;
		}

		btScalar getParam(int num, int axis = -1) const {
			return 0xB16B00B5;
		}

		void setParam(int num, btScalar value, int axis = -1) {

		}

	private:
		IPhysicsUserConstraint *m_pUserConstraint;
};

/*********************************
* CLASS CPhysicsConstraint
*********************************/

CPhysicsConstraint::CPhysicsConstraint(CPhysicsEnvironment *pEnv, IPhysicsConstraintGroup *pGroup, CPhysicsObject *pReferenceObject, CPhysicsObject *pAttachedObject, btTypedConstraint *pConstraint, EConstraintType type) {
	m_pReferenceObject = pReferenceObject;
	m_pAttachedObject = pAttachedObject;
	m_pConstraint = pConstraint;
	m_pGroup = (CPhysicsConstraintGroup *)pGroup; // May be NULL.
	m_pEnv = pEnv;
	m_type = type;
	m_bRemovedFromEnv = false;
	m_bNotifyBroken = true;

	if (m_type == CONSTRAINT_RAGDOLL || m_type == CONSTRAINT_BALLSOCKET || m_type == CONSTRAINT_FIXED) {
	    //lwss: these branches were identical. I have a feeling parts of this lib were gimped before release.
	    // but, it makes sense here to add the 2nd arg to disable collisions on linked bodies
	    // and doing so improves the ragdoll performance by a good bit.
		m_pEnv->GetBulletEnvironment()->addConstraint(m_pConstraint, true);
	} else {
		m_pEnv->GetBulletEnvironment()->addConstraint(m_pConstraint);
	}

	m_pConstraint->setUserConstraintPtr(this);

	if (pReferenceObject) {
		pReferenceObject->AttachedToConstraint(this);
		pReferenceObject->AttachEventListener(this);
	}

	if (pAttachedObject) {
		pAttachedObject->AttachedToConstraint(this);
		pAttachedObject->AttachEventListener(this);
	}

	if (m_pGroup) {
		m_pGroup->AddConstraint(this);
	}
}

CPhysicsConstraint::~CPhysicsConstraint() {
	if (!m_bRemovedFromEnv) {
		m_pEnv->GetBulletEnvironment()->removeConstraint(m_pConstraint);
		if (m_type == CONSTRAINT_USER)
			((btUserConstraint *)m_pConstraint)->getUserConstraint()->ConstraintDestroyed(this);

		m_bRemovedFromEnv = true;
	}

	if (m_pReferenceObject) {
		m_pReferenceObject->DetachedFromConstraint(this);
		m_pReferenceObject->DetachEventListener(this);
	}

	if (m_pAttachedObject) {
		m_pAttachedObject->DetachedFromConstraint(this);
		m_pAttachedObject->DetachEventListener(this);
	}

	if (m_pGroup) {
		m_pGroup->RemoveConstraint(this);
	}

	delete m_pConstraint;
}

void CPhysicsConstraint::Activate() {
	m_pConstraint->setEnabled(true);
}

void CPhysicsConstraint::Deactivate() {
	m_pConstraint->setEnabled(false);
}

void CPhysicsConstraint::SetLinearMotor(float speed, float maxLinearImpulse) {
	switch (m_type) {
		case CONSTRAINT_SLIDING: {
			btSliderConstraint *pSlider = (btSliderConstraint *)m_pConstraint;

			pSlider->setTargetAngMotorVelocity(HL2BULL(speed));
			pSlider->setMaxAngMotorForce(HL2BULL(maxLinearImpulse));
			pSlider->setPoweredLinMotor(true);
			break;
		}
		default: {
			NOT_IMPLEMENTED
			return;
		}
	}
}

void CPhysicsConstraint::SetAngularMotor(float rotSpeed, float maxAngularImpulse) {
	switch (m_type) {
		case CONSTRAINT_HINGE: {
			btHingeConstraint *pHinge = (btHingeConstraint *)m_pConstraint;

			// FIXME: Probably not the right conversions!
			pHinge->enableAngularMotor(true, ConvertAngleToBull(rotSpeed), HL2BULL(maxAngularImpulse));
		}
		default: {
			NOT_IMPLEMENTED
			return;
		}
	}
}

// Unused.
void CPhysicsConstraint::UpdateRagdollTransforms(const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached) {
	if (m_type != CONSTRAINT_RAGDOLL) return;
	NOT_IMPLEMENTED
}

// Appears to be only used in debugging. Returns true if we fill the matrices I guess
bool CPhysicsConstraint::GetConstraintTransform(matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached) const {
	if (!pConstraintToReference && !pConstraintToAttached) return false;

	NOT_IMPLEMENTED
	return false;
}

// Appears to be used in debugging only. Return value ignored in single use case.
bool CPhysicsConstraint::GetConstraintParams(constraint_breakableparams_t *pParams) const {
	if (!pParams) return false;

	NOT_IMPLEMENTED
	return false;
}

void CPhysicsConstraint::OutputDebugInfo() {
	Msg("-------------------\n");
	Msg("%s constraint\n", GetConstraintName(m_type));
}

// UNEXPOSED
// This function is called before the rigid body is deleted.
void CPhysicsConstraint::ObjectDestroyed(CPhysicsObject *pObject) {
	if (pObject != m_pAttachedObject && pObject != m_pReferenceObject) {
		AssertMsg(0, "ObjectDestroyed called with object that isn't part of this constraint!");
		return;
	}

	if (pObject == m_pAttachedObject)
		m_pAttachedObject = NULL;

	if (pObject == m_pReferenceObject)
		m_pReferenceObject = NULL;

	// Constraint is no longer valid due to one of its objects being removed, so stop simulating it.
	bool notify = false; // Don't run the callback more than once!
	if (!m_bRemovedFromEnv) {
		m_pEnv->GetBulletEnvironment()->removeConstraint(m_pConstraint);
		m_bRemovedFromEnv = true;
		notify = true;

		if (m_type == CONSTRAINT_USER)
			((btUserConstraint *)m_pConstraint)->getUserConstraint()->ConstraintDestroyed(this);
	}

	// Tell the game that this constraint was broken.
	if (notify && m_bNotifyBroken)
		m_pEnv->HandleConstraintBroken(this);
}

// UNEXPOSED
EConstraintType CPhysicsConstraint::GetType() {
	return m_type;
}

// UNEXPOSED
btTypedConstraint *CPhysicsConstraint::GetConstraint() {
	return m_pConstraint;
}

/*********************************
* CLASS CPhysicsConstraintGroup
*********************************/

CPhysicsConstraintGroup::CPhysicsConstraintGroup(CPhysicsEnvironment *pEnv, const constraint_groupparams_t &params) {
	m_errorParams = params;
	m_pEnvironment = pEnv;
}

CPhysicsConstraintGroup::~CPhysicsConstraintGroup() {

}

void CPhysicsConstraintGroup::Activate() {
	for (int i = 0; i < m_constraints.Count(); i++) {
		m_constraints[i]->Activate();
	}
}

bool CPhysicsConstraintGroup::IsInErrorState() {
	// Called every frame
	// NOT_IMPLEMENTED
	return false;
}

void CPhysicsConstraintGroup::ClearErrorState() {
	// Called every frame
	// NOT_IMPLEMENTED
}

void CPhysicsConstraintGroup::GetErrorParams(constraint_groupparams_t *pParams) {
	if (pParams)
		*pParams = m_errorParams;
}

void CPhysicsConstraintGroup::SetErrorParams(const constraint_groupparams_t &params) {
	m_errorParams = params;
}

void CPhysicsConstraintGroup::SolvePenetration(IPhysicsObject *pObj0, IPhysicsObject *pObj1) {
	NOT_IMPLEMENTED
}

// UNEXPOSED
void CPhysicsConstraintGroup::AddConstraint(CPhysicsConstraint *pConstraint) {
	m_constraints.AddToTail(pConstraint);
}

// UNEXPOSED
void CPhysicsConstraintGroup::RemoveConstraint(CPhysicsConstraint *pConstraint) {
	m_constraints.FindAndRemove(pConstraint);
}

/************************
* CLASS CPhysicsSpring
************************/

CPhysicsSpring::CPhysicsSpring(CPhysicsEnvironment *pEnv, CPhysicsObject *pObject1, CPhysicsObject *pObject2, btTypedConstraint *pConstraint, EConstraintType type):
CPhysicsConstraint(pEnv, NULL, pObject1, pObject2, pConstraint, type) {
	m_bNotifyBroken = false; // Don't tell the game as this isn't a normal constraint
}

CPhysicsSpring::~CPhysicsSpring() {

}

void CPhysicsSpring::GetEndpoints(Vector *worldPositionStart, Vector *worldPositionEnd) {
	if (!worldPositionStart && !worldPositionEnd) return;
	NOT_IMPLEMENTED
}

void CPhysicsSpring::SetSpringConstant(float flSpringContant) {
	((btSpringConstraint *)m_pConstraint)->setConstant(flSpringContant);
}

void CPhysicsSpring::SetSpringDamping(float flSpringDamping) {
	((btSpringConstraint *)m_pConstraint)->setDamping(flSpringDamping);
}

void CPhysicsSpring::SetSpringLength(float flSpringLength) {
	((btSpringConstraint *)m_pConstraint)->setLength(ConvertDistanceToBull(flSpringLength));
}

IPhysicsObject *CPhysicsSpring::GetStartObject() {
	return m_pReferenceObject;
}

IPhysicsObject *CPhysicsSpring::GetEndObject() {
	return m_pAttachedObject;
}

/************************
* CREATION FUNCTIONS
************************/

CPhysicsSpring *CreateSpringConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, springparams_t *spring) {
	if (!spring) return NULL;

	btRigidBody *rbA = ((CPhysicsObject *)pReferenceObject)->GetObject();
	btRigidBody *rbB = ((CPhysicsObject *)pAttachedObject)->GetObject();

	btVector3 bullRefPos, bullAttPos;
	ConvertPosToBull(spring->startPosition, bullRefPos);
	ConvertPosToBull(spring->endPosition, bullAttPos);

	if (!spring->useLocalPositions) {
		bullRefPos = rbA->getWorldTransform().inverse() * bullRefPos;
		bullAttPos = rbB->getWorldTransform().inverse() * bullAttPos;
	}

	btScalar length = ConvertDistanceToBull(spring->naturalLength);

	// FIXME: Does spring constant come in from half life as N/m or N/in?
	// Also how is the damping value applied?
	btSpringConstraint *pConstraint = new btSpringConstraint(*((CPhysicsObject *)pReferenceObject)->GetObject(), *((CPhysicsObject *)pAttachedObject)->GetObject(), bullRefPos, bullAttPos,
																length, spring->constant, spring->onlyStretch, spring->damping);

	return new CPhysicsSpring(pEnv, (CPhysicsObject *)pReferenceObject, (CPhysicsObject *)pAttachedObject, pConstraint, CONSTRAINT_SPRING);
}

static void SetupAxis(int axis, btGeneric6DofConstraint *pConstraint, const constraint_axislimit_t &axisData, bool clockwise) {
	// HL to bullet axis map
	static const int axisMap[] = {0, 2, 1};

	btRotationalLimitMotor *motor = pConstraint->getRotationalLimitMotor(axisMap[axis]);

	if (axisData.torque != 0) {
		motor->m_enableMotor = true;
		motor->m_maxMotorForce = ConvertAngleToBull(HL2BULL(axisData.torque));
	}

	if (axis == 1) {
		motor->m_hiLimit = -ConvertAngleToBull(axisData.minRotation);
		motor->m_loLimit = -ConvertAngleToBull(axisData.maxRotation);

		if (axisData.torque != 0) {
			motor->m_targetVelocity = -ConvertAngleToBull(axisData.angularVelocity);
		}
	} else {
		motor->m_hiLimit = ConvertAngleToBull(axisData.maxRotation);
		motor->m_loLimit = ConvertAngleToBull(axisData.minRotation);

		if (axisData.torque != 0) {
			motor->m_targetVelocity = ConvertAngleToBull(axisData.angularVelocity);
		}
	}

	Assert(motor->m_loLimit <= motor->m_hiLimit);
}

CPhysicsConstraint *CreateRagdollConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;
	btRigidBody *objRef = pObjRef->GetObject();
	btRigidBody *objAtt = pObjAtt->GetObject();

	btTransform constraintToReference, constraintToAttached;
	ConvertMatrixToBull(ragdoll.constraintToReference, constraintToReference); // constraintToReference is ALWAYS the identity matrix.
	ConvertMatrixToBull(ragdoll.constraintToAttached, constraintToAttached);

	// Constraint needs to be positioned at the point where the objects need to rotate around
	btTransform bullAFrame = ((btMassCenterMotionState *)objRef->getMotionState())->m_centerOfMassOffset.inverse() * constraintToReference;
	btTransform bullBFrame = ((btMassCenterMotionState *)objAtt->getMotionState())->m_centerOfMassOffset.inverse() * constraintToAttached;

    //lwss: Apply ragdoll gravity from csgo
    pObjAtt->SetLocalGravity( pEnv->GetDesiredRagdollGravity() );
    pAttachedObject->SetCollisionHints( COLLISION_HINT_RAGDOLL );
    pObjRef->SetLocalGravity( pEnv->GetDesiredRagdollGravity() );
    pReferenceObject->SetCollisionHints( COLLISION_HINT_RAGDOLL );
    //lwss end

	// TODO: btGeneric6DofConstraint has a bug where if y-axis is the only unlocked axis and the difference approaches pi/2, the other two axes
	// become "undefined" and causes the constraint to explode.
	btGeneric6DofConstraint *pConstraint = new btGeneric6DofConstraint(*objRef, *objAtt, bullAFrame, bullBFrame, true);
	pConstraint->setEnabled(ragdoll.isActive);

	// Set axis limits
	for (int i = 0; i < 3; i++) {
		SetupAxis(i, pConstraint, ragdoll.axes[i], ragdoll.useClockwiseRotations);
	}

	if (ragdoll.onlyAngularLimits) {
		pConstraint->setAngularOnly(true);
	}

	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pConstraint, CONSTRAINT_RAGDOLL);
}

CPhysicsConstraint *CreateHingeConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;
	btRigidBody *objRef = pObjRef->GetObject();
	btRigidBody *objAtt = pObjAtt->GetObject();

	btVector3 bullWorldPosition, bullWorldAxis;
	ConvertPosToBull(hinge.worldPosition, bullWorldPosition);
	ConvertDirectionToBull(hinge.worldAxisDirection, bullWorldAxis);

	btMatrix3x3 worldMatrix;
	bullAxisToMatrix(bullWorldAxis, worldMatrix);

	// Setup the constraint to be on the expected axis (flip fwd and right)
	btVector3 fwd, up, right;
	fwd		= worldMatrix.getColumn(0);
	up		= worldMatrix.getColumn(1);
	right	= worldMatrix.getColumn(2);

	worldMatrix.setValue(right.x(), up.x(), fwd.x(),
						 right.y(), up.y(), fwd.y(),
						 right.z(), up.z(), fwd.z());

	// Constraint world transform
	btTransform worldTrans(worldMatrix, bullWorldPosition);

	// Setup local transforms inside of the objects
	btTransform refTransform = objRef->getWorldTransform().inverse() * worldTrans;
	btTransform attTransform = objAtt->getWorldTransform().inverse() * worldTrans;

    //lwss hack: force the hinges to start deactivated
    // this *Could* become an issue if you have hinges that are sideways waiting to be affected by gravity at the start of each session.
    // it is here as an optimization and also to help investigate a current bug with the swingin' sign on de_train
    pObjAtt->GetObject()->setActivationState( WANTS_DEACTIVATION, true  );

    //lwss: swinging objects need a lower tolerance to return back to their starting point.
    // good examples include the sign on cs_assault and swingset on de_overpass.
    pObjAtt->GetObject()->setSleepingThresholds( btScalar( 0.05 ), btScalar( 0.05 ) );
    pObjRef->GetObject()->setSleepingThresholds( btScalar( 0.05 ), btScalar( 0.05 ) );

    btHingeConstraint *pHinge = new btHingeConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(), refTransform, attTransform);

    if (hinge.hingeAxis.minRotation != hinge.hingeAxis.maxRotation)
        pHinge->setLimit(ConvertAngleToBull(hinge.hingeAxis.minRotation), ConvertAngleToBull(hinge.hingeAxis.maxRotation));

	// FIXME: Are we converting the torque correctly? Bullet takes a "max motor impulse"
	if (hinge.hingeAxis.torque)
    {
	    //lwss: there is a *1000 on the hinge friction in game/server/physconstraint.cpp(CreateConstraint)
	    // that gets applied right before this function
	    // overpass swings: 90,000; 110,000.
	    // de_train sign(BUGGED): 1,000
	    // cs_militia ct-sign: 1000 - weather vane(BUGGED): 400(yes it is 0.4*1000)
	    float torque = hinge.hingeAxis.torque / 1000;

	    //lwss: If there's torque, set a bit of damping.
	    // These values normally go from 0->1, this value works pretty good
	    pHinge->getRigidBodyB().setDamping( 0.2f, 0.2f );
        pHinge->enableAngularMotor(true, ConvertAngleToBull(hinge.hingeAxis.angularVelocity), ConvertAngleToBull( HL2BULL(torque) ));
    }

	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pHinge, CONSTRAINT_HINGE);
}

CPhysicsConstraint *CreateFixedConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;
	btFixedConstraint *pWeld = new btFixedConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(),
																pObjRef->GetObject()->getWorldTransform().inverse() * pObjAtt->GetObject()->getWorldTransform(),
																btTransform::getIdentity());

	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pWeld, CONSTRAINT_FIXED);
}

CPhysicsConstraint *CreateSlidingConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;
	btRigidBody *objRef = pObjRef->GetObject();
	btRigidBody *objAtt = pObjAtt->GetObject();

	// Axis to slide on in reference object space
	btVector3 slideAxisRef;
	ConvertDirectionToBull(sliding.slideAxisRef, slideAxisRef);

	// Reference -> attached object transform
	btTransform refToAttXform = objAtt->getWorldTransform().inverse() * objRef->getWorldTransform();

	// Attached -> reference object transform
	btTransform attToRefXform = objRef->getWorldTransform().inverse() * objAtt->getWorldTransform();

	// Build reference matrix (in reference object space)
	btMatrix3x3 refMatrix;
	bullAxisToMatrix(slideAxisRef, refMatrix);

	btQuaternion refQuat;
	refMatrix.getRotation(refQuat);

	// Important to be the same rotation around axis (prevent attached object from flipping out)
	btQuaternion attQuat = refToAttXform.getRotation() * refQuat;

	// Final frames
	btTransform refFrame = btTransform::getIdentity();
	refFrame.setBasis(refMatrix);

	// Attached frame will be where reference frame is (keep attached object in its initial spot instead of snapping to the center)
	btTransform attFrame = attToRefXform.inverse();
	attFrame.setRotation(attQuat);

	btSliderConstraint *pSlider = new btSliderConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(), refFrame, attFrame, true);

	// If linear min == lin max then there is no limit!
	if (sliding.limitMin != sliding.limitMax) {
		pSlider->setLowerLinLimit(ConvertDistanceToBull(sliding.limitMin));
		pSlider->setUpperLinLimit(ConvertDistanceToBull(sliding.limitMax));
	}

	pSlider->setLowerAngLimit(0);
	pSlider->setUpperAngLimit(0);

	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pSlider, CONSTRAINT_SLIDING);
}

CPhysicsConstraint *CreateBallsocketConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;

	btVector3 obj1Pos, obj2Pos;
	ConvertPosToBull(ballsocket.constraintPosition[0], obj1Pos);
	ConvertPosToBull(ballsocket.constraintPosition[1], obj2Pos);

	obj1Pos -= ((btMassCenterMotionState *)pObjRef->GetObject()->getMotionState())->m_centerOfMassOffset.getOrigin();
	obj2Pos -= ((btMassCenterMotionState *)pObjAtt->GetObject()->getMotionState())->m_centerOfMassOffset.getOrigin();

	btPoint2PointConstraint *pBallsock = new btPoint2PointConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(), obj1Pos, obj2Pos);
	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pBallsock, CONSTRAINT_BALLSOCKET);
}

// NOT COMPLETE
CPhysicsConstraint *CreatePulleyConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley) {
	// TODO: Bullet has no default pulley constraint. Make one.
	NOT_IMPLEMENTED
	return NULL;
}

CPhysicsConstraint *CreateLengthConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length) {
	btVector3 obj1Pos, obj2Pos;
	ConvertPosToBull(length.objectPosition[0], obj1Pos);
	ConvertPosToBull(length.objectPosition[1], obj2Pos);

	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;

	obj1Pos -= ((btMassCenterMotionState *)pObjRef->GetObject()->getMotionState())->m_centerOfMassOffset.getOrigin();
	obj2Pos -= ((btMassCenterMotionState *)pObjAtt->GetObject()->getMotionState())->m_centerOfMassOffset.getOrigin();

	btPoint2PointConstraint *pLength = new btLengthConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(), obj1Pos, obj2Pos, ConvertDistanceToBull(length.minLength), ConvertDistanceToBull(length.totalLength));
	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pLength, CONSTRAINT_LENGTH);
}

CPhysicsConstraint *CreateGearConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_gearparams_t &gear) {
	CPhysicsObject *pObjRef = (CPhysicsObject *)pReferenceObject;
	CPhysicsObject *pObjAtt = (CPhysicsObject *)pAttachedObject;

	btVector3 axes[2];
	for (int i = 0; i < 2; i++) {
		ConvertDirectionToBull(gear.objectLocalAxes[i], axes[i]);
	}

	btGearConstraint *pConstraint = new btGearConstraint(*pObjRef->GetObject(), *pObjAtt->GetObject(), axes[0], axes[1], gear.ratio);
	return new CPhysicsConstraint(pEnv, pGroup, pObjRef, pObjAtt, pConstraint, CONSTRAINT_GEAR);
}

CPhysicsConstraintGroup *CreateConstraintGroup(CPhysicsEnvironment *pEnv, const constraint_groupparams_t &params) {
	if (!pEnv) return NULL;

	return new CPhysicsConstraintGroup(pEnv, params);
}

CPhysicsConstraint *CreateUserConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, IPhysicsUserConstraint *pUserConstraint) {
	Assert(pUserConstraint);

	btTypedConstraint *constraint = new btUserConstraint(*((CPhysicsObject *)pReferenceObject)->GetObject(), *((CPhysicsObject *)pAttachedObject)->GetObject(), pUserConstraint);

	CPhysicsConstraint *pConstraint = new CPhysicsConstraint(pEnv, pGroup, (CPhysicsObject *)pReferenceObject, (CPhysicsObject *)pAttachedObject, constraint, CONSTRAINT_USER);
	pConstraint->SetNotifyBroken(false); // Until an entity is hooked up or whatever
	return pConstraint;
}
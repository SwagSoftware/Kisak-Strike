#ifndef PHYSICS_CONSTRAINT_H
#define PHYSICS_CONSTRAINT_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

#include <vphysics/constraints.h>
#include "vphysics_bullet/constraintsV32.h"

#include "Physics_Object.h"

class CPhysicsEnvironment;
class CPhysicsConstraintGroup;
class btSpringConstraint;

enum EConstraintType {
	CONSTRAINT_UNKNOWN,
	CONSTRAINT_RAGDOLL,
	CONSTRAINT_HINGE,
	CONSTRAINT_FIXED,
	CONSTRAINT_SLIDING,
	CONSTRAINT_BALLSOCKET,
	CONSTRAINT_PULLEY,
	CONSTRAINT_LENGTH,
	CONSTRAINT_SPRING,
	CONSTRAINT_GEAR,
	CONSTRAINT_USER
};

class CPhysicsConstraint : public IPhysicsConstraint, public IObjectEventListener {
	public:
		CPhysicsConstraint(CPhysicsEnvironment *pEnv, IPhysicsConstraintGroup *pGroup, CPhysicsObject *pObject1, CPhysicsObject *pObject2, btTypedConstraint *pConstraint, EConstraintType type);
		~CPhysicsConstraint();

		void					Activate();
		void					Deactivate();

		void					SetGameData(void *gameData) { m_pGameData = gameData; };
		void *					GetGameData() const { return m_pGameData; };

		IPhysicsObject *		GetReferenceObject() const { return m_pReferenceObject; };
		IPhysicsObject *		GetAttachedObject() const { return m_pAttachedObject; };

		void					SetLinearMotor(float speed, float maxLinearImpulse);
		void					SetAngularMotor(float rotSpeed, float maxAngularImpulse);
		
		void					UpdateRagdollTransforms(const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached);
		bool					GetConstraintTransform(matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached) const;
		bool					GetConstraintParams(constraint_breakableparams_t *pParams) const;
		
		void					OutputDebugInfo();

		// UNEXPOSED FUNCTIONS
	public:
		// Call this if you're an object currently being destroyed.
		virtual void			ObjectDestroyed(CPhysicsObject *pObject);

		EConstraintType			GetType();
		btTypedConstraint *		GetConstraint();

		void					SetNotifyBroken(bool notify) {m_bNotifyBroken = notify;}

	protected:
		CPhysicsObject *		m_pReferenceObject;
		CPhysicsObject *		m_pAttachedObject;
		CPhysicsConstraintGroup *m_pGroup;
		void *					m_pGameData;
		CPhysicsEnvironment *	m_pEnv;
		EConstraintType			m_type;
		bool					m_bRemovedFromEnv;
		bool					m_bNotifyBroken; // Should we notify the game that the constraint was broken?

		btTypedConstraint *		m_pConstraint;
};

class CPhysicsSpring : public IPhysicsSpring, public CPhysicsConstraint {
	public:
		CPhysicsSpring(CPhysicsEnvironment *pEnv, CPhysicsObject *pObject1, CPhysicsObject *pObject2, btTypedConstraint *pConstraint, EConstraintType type);
		~CPhysicsSpring();

		void			GetEndpoints(Vector *worldPositionStart, Vector *worldPositionEnd);
		void			SetSpringConstant(float flSpringContant);
		void			SetSpringDamping(float flSpringDamping);
		void			SetSpringLength(float flSpringLength);

		// Get the starting object
		IPhysicsObject *GetStartObject();

		// Get the end object
		IPhysicsObject *GetEndObject();
};

// FIXME: I dont think we can implement this in Bullet anyways?
// We'll have to emulate this on bullet.
class CPhysicsConstraintGroup : public IPhysicsConstraintGroup
{
	public:
		CPhysicsConstraintGroup(CPhysicsEnvironment *pEnv, const constraint_groupparams_t &params);
		~CPhysicsConstraintGroup(void);

		void	Activate();
		bool	IsInErrorState();
		void	ClearErrorState();
		void	GetErrorParams(constraint_groupparams_t *pParams);
		void	SetErrorParams(const constraint_groupparams_t &params);
		void	SolvePenetration(IPhysicsObject *pObj0, IPhysicsObject *pObj1);

	public:
		// Unexposed functions
		void	AddConstraint(CPhysicsConstraint *pConstraint);
		void	RemoveConstraint(CPhysicsConstraint *pConstraint);

	private:
		CUtlVector<CPhysicsConstraint *>	m_constraints;
		constraint_groupparams_t			m_errorParams;
		CPhysicsEnvironment *				m_pEnvironment;
};

// CONSTRAINT CREATION FUNCTIONS
CPhysicsSpring *CreateSpringConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, springparams_t *spring);
CPhysicsConstraint *CreateRagdollConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll);
CPhysicsConstraint *CreateHingeConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge);
CPhysicsConstraint *CreateFixedConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed);
CPhysicsConstraint *CreateSlidingConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding);
CPhysicsConstraint *CreateBallsocketConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket);
CPhysicsConstraint *CreatePulleyConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley);
CPhysicsConstraint *CreateLengthConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length);
CPhysicsConstraint *CreateGearConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_gearparams_t &gear);
CPhysicsConstraint *CreateUserConstraint(CPhysicsEnvironment *pEnv, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, IPhysicsUserConstraint *pUserConstraint);

CPhysicsConstraintGroup *CreateConstraintGroup(CPhysicsEnvironment *pEnv, const constraint_groupparams_t &params);

#endif // PHYSICS_CONSTRAINT_H

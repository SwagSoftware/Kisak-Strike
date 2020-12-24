#ifndef PHYSICS_SHADOWCONTROLLER_H
#define PHYSICS_SHADOWCONTROLLER_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

#include "IController.h"

#include "Physics_Object.h"

struct shadowcontrol_params_t {
	shadowcontrol_params_t() {lastPosition.setZero();}

	btVector3		targetPosition;
	btQuaternion	targetRotation;
	btScalar		maxSpeed;
	btScalar		maxDampSpeed;
	btScalar		maxAngular;
	btScalar		maxDampAngular;
	btVector3		lastPosition;
	float			dampFactor;
	float			teleportDistance;
};

class CShadowController : public IController, public IPhysicsShadowController, public IObjectEventListener
{
	public:
		CShadowController(CPhysicsObject *pObject, bool allowTranslation, bool allowRotation);
		~CShadowController();

		void					Update(const Vector &position, const QAngle &angles, float timeOffset);
		void					MaxSpeed(float maxSpeed, float maxAngularSpeed);
		void					StepUp(float height);
		void					SetTeleportDistance(float teleportDistance);

		bool					AllowsTranslation();
		bool					AllowsRotation();

		void					SetPhysicallyControlled(bool isPhysicallyControlled);
		bool					IsPhysicallyControlled();

		void					GetLastImpulse(Vector *pOut);
		void					UseShadowMaterial(bool bUseShadowMaterial);
		void					ObjectMaterialChanged(int materialIndex);

		float					GetTargetPosition(Vector *pPositionOut, QAngle *pAnglesOut);
		float					GetTeleportDistance();
		void					GetMaxSpeed(float *pMaxSpeedOut, float *pMaxAngularSpeedOut);

		// UNEXPOSED FUNCTIONS
		void					Tick(float deltaTime);
		void					SetAllowsTranslation(bool enable);
		void					SetAllowsRotation(bool enable);

		void					ObjectDestroyed(CPhysicsObject *pObject);

		int						GetTicksSinceUpdate();
	private:
		void					AttachObject();
		void					DetachObject();

		// NOTE: If you add more than 7 flags, change the m_flags variable type to a short.
		enum EShadowFlags {
			FLAG_ALLOWPHYSICSMOVEMENT	= 1<<0,
			FLAG_ALLOWPHYSICSROTATION	= 1<<1,
			FLAG_PHYSICALLYCONTROLLED	= 1<<2,
			FLAG_USESHADOWMATERIAL		= 1<<3,
		};

		CPhysicsObject *		m_pObject;
		float					m_secondsToArrival;
		btVector3				m_currentSpeed;
		float					m_savedMass;
		float					m_timeOffset;
		int						m_savedMaterialIndex;
		int						m_ticksSinceUpdate;
		bool					m_enable;
		char					m_flags;
		shadowcontrol_params_t	m_shadow;
};

float ComputeShadowControllerHL(CPhysicsObject *pObject, const hlshadowcontrol_params_t &params, float secondsToArrival, float dt);

CShadowController *CreateShadowController(IPhysicsObject *pObject, bool allowPhysicsMovement, bool allowPhysicsRotation);

#endif // PHYSICS_SHADOWCONTROLLER_H

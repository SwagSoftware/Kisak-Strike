#ifndef PHYSICS_DRAGCONTROLLER_H
#define PHYSICS_DRAGCONTROLLER_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

class CPhysicsObject;

class CPhysicsDragController {
	public:
									CPhysicsDragController();
		void						SetAirDensity(float density);
		float						GetAirDensity();

		void						AddPhysicsObject(CPhysicsObject *pObject);
		void						RemovePhysicsObject(CPhysicsObject *pObject);
		void						Tick(btScalar dt);
		bool						IsControlling(const CPhysicsObject *pObject) const;
	private:
		float						m_airDensity;

		CUtlVector<CPhysicsObject *>m_ents;
};

#endif // PHYSICS_DRAGCONTROLLER_H
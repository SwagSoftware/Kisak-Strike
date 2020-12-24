#ifndef PHYSICS_VEHICLEAIRBOAT_H
#define PHYSICS_VEHICLEAIRBOAT_H

class CPhysicsObject;
class IPhysicsGameTrace;

class CAirboatVehicle : public btActionInterface {
	public:
		CAirboatVehicle(CPhysicsObject *pBody, IPhysicsGameTrace *pGameTrace);
		~CAirboatVehicle();

		virtual void updateAction(btCollisionWorld *pWorld, btScalar dt);
		virtual void debugDraw(btIDebugDraw *pDebugDrawer);

	private:
		CPhysicsObject *m_pBody;
		IPhysicsGameTrace *m_pGameTrace;
};

#endif // PHYSICS_VEHICLEAIRBOAT_H
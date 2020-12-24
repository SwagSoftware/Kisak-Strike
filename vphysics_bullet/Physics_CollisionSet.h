#ifndef PHYSICS_COLLISIONSET_H
#define PHYSICS_COLLISIONSET_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

class CPhysicsCollisionSet : public IPhysicsCollisionSet {
	public:
						CPhysicsCollisionSet(int iMaxEntries);
						~CPhysicsCollisionSet();

		void			EnableCollisions(int index0, int index1);
		void			DisableCollisions(int index0, int index1);

		bool			ShouldCollide(int index0, int index1);

	private:
		int				m_iMaxEntries;
		int *			m_collArray;
};

CPhysicsCollisionSet *CreateCollisionSet(int maxElements);

#endif // PHYSICS_COLLISIONSET_H
#ifndef PHYSICS_H
#define PHYSICS_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

#include <utlhashtable.h>

class IPhysicsEnvironment;
class IPhysicsCollisionSet;

class CPhysics : public CTier1AppSystem<IPhysics32> {
	typedef CTier1AppSystem<IPhysics32> BaseClass;
	public:
		~CPhysics();
        bool                        Connect( CreateInterfaceFn factory );
        void                        Disconnect( void );
		void *						QueryInterface(const char *pInterfaceName);
		InitReturnVal_t				Init();
		void						Shutdown();

		IPhysicsEnvironment *		CreateEnvironment();
		void						DestroyEnvironment(IPhysicsEnvironment *pEnv);
		IPhysicsEnvironment *		GetActiveEnvironmentByIndex(int index);
		int							GetActiveEnvironmentCount();

		IPhysicsObjectPairHash *	CreateObjectPairHash();
		void						DestroyObjectPairHash(IPhysicsObjectPairHash *pHash);

		IPhysicsCollisionSet *		FindOrCreateCollisionSet( uintptr_t id, int maxElementCount);
		IPhysicsCollisionSet *		FindCollisionSet( uintptr_t id);
		void						DestroyAllCollisionSets();

        IVPhysicsDebugOverlay *         GetPhysicsDebugOverlay() { return m_pPhysicsDebugOverlay; }
	private:
		CUtlVector<IPhysicsEnvironment *>	m_envList;
		CUtlVector<IPhysicsCollisionSet *>	m_collisionSets;
		CUtlHashtable<unsigned int, unsigned int> m_colSetTable;
		IVPhysicsDebugOverlay *m_pPhysicsDebugOverlay;
};

extern CPhysics g_Physics;

#endif
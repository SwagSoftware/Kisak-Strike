#include "StdAfx.h"

#include <cmodel.h>
#include <cstring>

#include "Physics_Environment.h"
#include "Physics.h"
#include "Physics_Object.h"
#include "Physics_ShadowController.h"
#include "Physics_PlayerController.h"
#include "Physics_FluidController.h"
#include "Physics_DragController.h"
#include "Physics_MotionController.h"
#include "Physics_Constraint.h"
#include "Physics_Collision.h"
#include "Physics_VehicleController.h"
#include "miscmath.h"
#include "convert.h"

#if DEBUG_DRAW
	#include "DebugDrawer.h"
#endif

#include "BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"



/*****************************
* MISC. CLASSES
*****************************/

class IDeleteQueueItem {
	public:
		virtual void Delete() = 0;
};

template <typename T>
class CDeleteProxy : public IDeleteQueueItem {
	public:
		CDeleteProxy(T *pItem) : m_pItem(pItem) {}
		virtual void Delete() override { delete m_pItem; } 
	private:
		T *m_pItem;
};

class CDeleteQueue {
	public:
		void Add(IDeleteQueueItem *pItem) {
			m_list.AddToTail(pItem);
		}

		template <typename T>
		void QueueForDelete(T *pItem) {
			Add(new CDeleteProxy<T>(pItem));
		}

		void DeleteAll() {
			for (int i = m_list.Count()-1; i >= 0; --i) {
				m_list[i]->Delete();
				delete m_list[i];
			}
			m_list.RemoveAll();
		}
	private:
		CUtlVector<IDeleteQueueItem *> m_list;
};

bool CCollisionSolver::needBroadphaseCollision(btBroadphaseProxy *proxy0, btBroadphaseProxy *proxy1) const {
	btRigidBody *body0 = btRigidBody::upcast(static_cast<btCollisionObject*>(proxy0->m_clientObject));
	btRigidBody *body1 = btRigidBody::upcast(static_cast<btCollisionObject*>(proxy1->m_clientObject));

	if(!body0 || !body1)
	{
		// Check if one of them is a soft body
		//	btCollisionObject *colObj0 = static_cast<btCollisionObject*>(proxy0->m_clientObject);
		//	btCollisionObject *colObj1 = static_cast<btCollisionObject*>(proxy1->m_clientObject);
		//	if (colObj0->getInternalType() & btCollisionObject::CO_SOFT_BODY || colObj1->getInternalType() & btCollisionObject::CO_SOFT_BODY) {
		//		return true;
		//	}
		return (body0 != nullptr && !body0->isStaticObject()) || (body1 != nullptr && !body1->isStaticObject());
	}

	CPhysicsObject *pObject0 = static_cast<CPhysicsObject*>(body0->getUserPointer());
	CPhysicsObject *pObject1 = static_cast<CPhysicsObject*>(body1->getUserPointer());

	bool collides = NeedsCollision(pObject0, pObject1) && 
		(proxy0->m_collisionFilterGroup & proxy1->m_collisionFilterMask) && (proxy1->m_collisionFilterGroup & proxy0->m_collisionFilterMask);

	if (!collides) {
		// Clean this pair from the cache
		m_pEnv->GetBulletEnvironment()->getBroadphase()->getOverlappingPairCache()->removeOverlappingPair(proxy0, proxy1, m_pEnv->GetBulletEnvironment()->getDispatcher());
	}
			
	return collides;
}

bool CCollisionSolver::NeedsCollision(CPhysicsObject *pObject0, CPhysicsObject *pObject1) const {
	if (pObject0 && pObject1) {
		// No static->static collisions
		if (pObject0->IsStatic() && pObject1->IsStatic())
		{
			return false;
		}

		// No shadow->shadow collisions
		if (pObject0->GetShadowController() && pObject1->GetShadowController())
		{
			return false;
		}
		
		if (!pObject0->IsCollisionEnabled() || !pObject1->IsCollisionEnabled()) 
		{
			
			return false;
		}

		if ((pObject0->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION) || (pObject1->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE))
		{
			
			return false;
		}
		
		// No kinematic->static collisions
		if ((pObject0->GetObject()->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT && pObject1->IsStatic())
		 || (pObject1->GetObject()->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT && pObject0->IsStatic()))
		{
			return false;
		}
		
		if ((pObject1->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION) || (pObject0->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)) 
		{
			return false;
		}

		// Most expensive call, do this check last
		if (m_pSolver && !m_pSolver->ShouldCollide(pObject0, pObject1, pObject0->GetGameData(), pObject1->GetGameData()))
		{
			return false;
		}
	}
	else 
	{
		// One of the objects has no phys object...
		if (pObject0 && !pObject0->IsCollisionEnabled())
			return false;

		if (pObject1 && !pObject1->IsCollisionEnabled())
			return false;
	}

	return true;
}

void SerializeWorld_f(const CCommand &args) {
	if (args.ArgC() != 3) {
		Msg("Usage: bt_serialize <index> <name>\n");
		return;
	}

	CPhysicsEnvironment *pEnv = (CPhysicsEnvironment *)g_Physics.GetActiveEnvironmentByIndex(atoi(args.Arg(2)));
	if (pEnv) {
		btDiscreteDynamicsWorld *pWorld = static_cast<btDiscreteDynamicsWorld*>(pEnv->GetBulletEnvironment());
		Assert(pWorld);

		btSerializer *pSerializer = new btDefaultSerializer;
		pWorld->serialize(pSerializer);

		// FIXME: We shouldn't be using this. Find the appropiate method from valve interfaces.
		const char *pName = args.Arg(2);
		FILE *pFile = fopen(pName, "wb");
		if (pFile) {
			fwrite(pSerializer->getBufferPointer(), pSerializer->getCurrentBufferSize(), 1, pFile);
			fclose(pFile);
		} else {
			Warning("Couldn't open \"%s\" for writing!\n", pName);
		}
	} else {
		Warning("Invalid environment index supplied!\n");
	}
}

static ConCommand cmd_serializeworld("bt_serialize", SerializeWorld_f, "Serialize environment by index (usually 0=server, 1=client)\n\tDumps the file out to the exe directory.");

/*******************************
* CLASS CObjectTracker
*******************************/

class CObjectTracker {
	public:
		CObjectTracker(CPhysicsEnvironment *pEnv, IPhysicsObjectEvent *pObjectEvents) {
			m_pEnv = pEnv;
			m_pObjEvents = pObjectEvents;
		}

		int GetActiveObjectCount() const {
			return m_activeObjects.Count();
		}

		void GetActiveObjects(IPhysicsObject **pOutputObjectList) const {
			if (!pOutputObjectList) return;

			const int size = m_activeObjects.Count();
			for (int i = 0; i < size; i++) {
				pOutputObjectList[i] = m_activeObjects[i];
			}
		}

		void SetObjectEventHandler(IPhysicsObjectEvent *pEvents) {
			m_pObjEvents = pEvents;
		}

		void ObjectRemoved(CPhysicsObject *pObject) {
			m_activeObjects.FindAndRemove(pObject);
		}

		void Tick() {
			btDiscreteDynamicsWorld *pBulletEnv = m_pEnv->GetBulletEnvironment();
			btCollisionObjectArray &colObjArray = pBulletEnv->getCollisionObjectArray();
			for (int i = 0; i < colObjArray.size(); i++) {
				CPhysicsObject *pObj = static_cast<CPhysicsObject*>(colObjArray[i]->getUserPointer());
				if (!pObj) continue; // Internal object that the game doesn't need to know about

				Assert(*(char *)pObj != 0xDD); // Make sure the object isn't deleted (only works in debug builds) TODO: This can be removed, result is always true
 
				// Don't add objects marked for delete
				if (pObj->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE) {
					continue;
				}

				if (colObjArray[i]->getActivationState() != pObj->GetLastActivationState()) {
					const int newState = colObjArray[i]->getActivationState();

					// Not a state we want to track.
					if (newState == WANTS_DEACTIVATION)
						continue;

					if (m_pObjEvents) {
						switch (newState) {
							// FIXME: Objects may call objectwake twice if they go from disable_deactivation -> active_tag
							case DISABLE_DEACTIVATION:
							case ACTIVE_TAG:
								m_pObjEvents->ObjectWake(pObj);
								break;
							case ISLAND_SLEEPING:
								m_pObjEvents->ObjectSleep(pObj);
								break;
							case DISABLE_SIMULATION:
								// Don't call ObjectSleep on DISABLE_SIMULATION on purpose.
								break;
							default:
								NOT_IMPLEMENTED;
								assert(false);
						}
					}

					switch (newState) {
						case DISABLE_DEACTIVATION:
						case ACTIVE_TAG:
							// Don't add the object twice!
							if (m_activeObjects.Find(pObj) == -1)
								m_activeObjects.AddToTail(pObj);

							break;
						case DISABLE_SIMULATION:
						case ISLAND_SLEEPING:
							m_activeObjects.FindAndRemove(pObj);
							break;
						default:
							NOT_IMPLEMENTED;
							assert(false);
					}

					pObj->SetLastActivationState(newState);
				}
			}
		}

	private:
		CPhysicsEnvironment *m_pEnv;
		IPhysicsObjectEvent *m_pObjEvents;

		CUtlVector<IPhysicsObject *> m_activeObjects;
};

/*******************************
* CLASS CPhysicsCollisionData
*******************************/

class CPhysicsCollisionData : public IPhysicsCollisionData {
	public:
		CPhysicsCollisionData(btManifoldPoint *manPoint) {
			ConvertDirectionToHL(manPoint->m_normalWorldOnB, m_surfaceNormal);
			ConvertPosToHL(manPoint->getPositionWorldOnA(), m_contactPoint);
			ConvertPosToHL(manPoint->m_lateralFrictionDir1, m_contactSpeed);	// FIXME: Need the correct variable from the manifold point
		}

		// normal points toward second object (object index 1)
		void GetSurfaceNormal(Vector &out) override
		{
			out = m_surfaceNormal;
		}

		// contact point of collision (in world space)
		void GetContactPoint(Vector &out) override
		{
			out = m_contactPoint;
		}

		// speed of surface 1 relative to surface 0 (in world space)
		void GetContactSpeed(Vector &out) override
		{
			out = m_contactSpeed;
		}

	private:
		Vector m_surfaceNormal;
		Vector m_contactPoint;
		Vector m_contactSpeed;
};

/*********************************
* CLASS CCollisionEventListener
*********************************/

class CCollisionEventListener : public btSolveCallback {
	public:
		CCollisionEventListener(CPhysicsEnvironment *pEnv) {
			m_pEnv = pEnv;
			m_pCallback = NULL;
		}

		// TODO: Optimize this, heavily!
		void preSolveContact(btSolverBody *body0, btSolverBody *body1, btManifoldPoint *cp) override
		{
			CPhysicsObject *pObj0 = static_cast<CPhysicsObject*>(body0->m_originalColObj->getUserPointer());
			CPhysicsObject *pObj1 = static_cast<CPhysicsObject*>(body1->m_originalColObj->getUserPointer());
			if (pObj0->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE || pObj1->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)
				return;

			const unsigned int flags0 = pObj0->GetCallbackFlags();
			const unsigned int flags1 = pObj1->GetCallbackFlags();

			// Clear it
			memset(&m_tmpEvent, 0, sizeof(m_tmpEvent));
			m_tmpEvent.collisionSpeed = 0.f; // Invalid pre-collision
			m_tmpEvent.deltaCollisionTime = 10.f; // FIXME: Find a way to track the real delta time
			m_tmpEvent.isCollision = (flags0 & flags1 & CALLBACK_GLOBAL_COLLISION); // False when either one of the objects don't have CALLBACK_GLOBAL_COLLISION
			m_tmpEvent.isShadowCollision = ((flags0 ^ flags1) & CALLBACK_SHADOW_COLLISION) != 0; // True when only one of the objects is a shadow (if both are shadow, it's handled by the game)

			m_tmpEvent.pObjects[0] = pObj0;
			m_tmpEvent.pObjects[1] = pObj1;	
			m_tmpEvent.surfaceProps[0] = pObj0->GetMaterialIndex();
			m_tmpEvent.surfaceProps[1] = pObj1->GetMaterialIndex();

			if ((pObj0->IsStatic() && !(flags1 & CALLBACK_GLOBAL_COLLIDE_STATIC)) || (pObj1->IsStatic() && !(flags0 & CALLBACK_GLOBAL_COLLIDE_STATIC))) {
				m_tmpEvent.isCollision = false;
			}

			if (!m_tmpEvent.isCollision && !m_tmpEvent.isShadowCollision) return;

			CPhysicsCollisionData data(cp);
			m_tmpEvent.pInternalData = &data;

			// Give the game its stupid velocities
			if (body0->m_originalBody) {
				m_tmpVelocities[0] = body0->m_originalBody->getLinearVelocity();
				m_tmpAngVelocities[0] = body0->m_originalBody->getAngularVelocity();

				body0->m_originalBody->setLinearVelocity(m_tmpVelocities[0] + body0->internalGetDeltaLinearVelocity());
				body0->m_originalBody->setAngularVelocity(m_tmpAngVelocities[0] + body0->internalGetDeltaAngularVelocity());
			}
			if (body1->m_originalBody) {
				m_tmpVelocities[1] = body1->m_originalBody->getLinearVelocity();
				m_tmpAngVelocities[1] = body1->m_originalBody->getAngularVelocity();

				body1->m_originalBody->setLinearVelocity(m_tmpVelocities[1] + body1->internalGetDeltaLinearVelocity());
				body1->m_originalBody->setAngularVelocity(m_tmpAngVelocities[1] + body1->internalGetDeltaAngularVelocity());
			}

			if (m_pCallback)
				m_pCallback->PreCollision(&m_tmpEvent);

			// Restore the velocities
			// UNDONE: No need, postSolveContact will do this.
			/*
			if (body0->m_originalBody) {
				body0->m_originalBody->setLinearVelocity(m_tmpVelocities[0]);
				body0->m_originalBody->setAngularVelocity(m_tmpAngVelocities[0]);
			}
			if (body1->m_originalBody) {
				body1->m_originalBody->setLinearVelocity(m_tmpVelocities[1]);
				body1->m_originalBody->setAngularVelocity(m_tmpAngVelocities[1]);
			}
			*/
		}

		// TODO: Optimize this, heavily!
		void postSolveContact(btSolverBody *body0, btSolverBody *body1, btManifoldPoint *cp) override
		{
			btRigidBody *rb0 = btRigidBody::upcast(body0->m_originalColObj);
			btRigidBody *rb1 = btRigidBody::upcast(body1->m_originalColObj);

			// FIXME: Problem with bullet code, only one solver body created for static objects!
			// There could be more than one static object created by us!
			CPhysicsObject *pObj0 = static_cast<CPhysicsObject*>(body0->m_originalColObj->getUserPointer());
			CPhysicsObject *pObj1 = static_cast<CPhysicsObject*>(body1->m_originalColObj->getUserPointer());
			if (pObj0->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE || pObj1->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)
				return;

			//unsigned int flags0 = pObj0->GetCallbackFlags();
			//unsigned int flags1 = pObj1->GetCallbackFlags();

			const btScalar combinedInvMass = rb0->getInvMass() + rb1->getInvMass();
			m_tmpEvent.collisionSpeed = BULL2HL(cp->m_appliedImpulse * combinedInvMass); // Speed of body 1 rel to body 2 on axis of constraint normal

            // FIXME: Find a way to track the real delta time
            // IVP tracks this delta time between object pairs

            // lwss: I had a method in here to track the delta times between the pairs.
            // I wondered what it was for, turns out it's all 95% for a sound hack
            // in which they don't play sounds too often on 2 colliding objects.
            // so for now, I'll just leave this as is and play all physics sounds
            m_tmpEvent.deltaCollisionTime = 10.f;

			//lwss hack - ragdoll sounds are a bit too loud
			if( pObj0->GetCollisionHints() & COLLISION_HINT_RAGDOLL || pObj1->GetCollisionHints() & COLLISION_HINT_RAGDOLL )
            {
			    // value determined from testing :))
                m_tmpEvent.collisionSpeed *= 0.4f;
            }
			//lwss end

			/*
			m_tmpEvent.isCollision = (flags0 & flags1 & CALLBACK_GLOBAL_COLLISION); // False when either one of the objects don't have CALLBACK_GLOBAL_COLLISION
			m_tmpEvent.isShadowCollision = (flags0 ^ flags1) & CALLBACK_SHADOW_COLLISION; // True when only one of the objects is a shadow

			m_tmpEvent.pObjects[0] = pObj0;
			m_tmpEvent.pObjects[1] = pObj1;	
			m_tmpEvent.surfaceProps[0] = pObj0 ? pObj0->GetMaterialIndex() : 0;
			m_tmpEvent.surfaceProps[1] = pObj1 ? pObj1->GetMaterialIndex() : 0;
			*/

			if (!m_tmpEvent.isCollision && !m_tmpEvent.isShadowCollision) return;

			CPhysicsCollisionData data(cp);
			m_tmpEvent.pInternalData = &data;

			// Give the game its stupid velocities
			if (body0->m_originalBody) {
				body0->m_originalBody->setLinearVelocity(m_tmpVelocities[0] + body0->internalGetDeltaLinearVelocity());
				body0->m_originalBody->setAngularVelocity(m_tmpAngVelocities[0] + body0->internalGetDeltaAngularVelocity());
			}
			if (body1->m_originalBody) {
				body1->m_originalBody->setLinearVelocity(m_tmpVelocities[1] + body1->internalGetDeltaLinearVelocity());
				body1->m_originalBody->setAngularVelocity(m_tmpAngVelocities[1] + body1->internalGetDeltaAngularVelocity());
			}

			if (m_pCallback)
				m_pCallback->PostCollision(&m_tmpEvent);

			// Restore the velocities
			if (body0->m_originalBody) {
				body0->m_originalBody->setLinearVelocity(m_tmpVelocities[0]);
				body0->m_originalBody->setAngularVelocity(m_tmpAngVelocities[0]);
			}
			if (body1->m_originalBody) {
				body1->m_originalBody->setLinearVelocity(m_tmpVelocities[1]);
				body1->m_originalBody->setAngularVelocity(m_tmpAngVelocities[1]);
			}
		}

		void friction(btSolverBody *body0, btSolverBody *body1, btSolverConstraint *constraint) override
		{
			/*
			btRigidBody *rb0 = btRigidBody::upcast(body0->m_originalColObj);
			btRigidBody *rb1 = btRigidBody::upcast(body1->m_originalColObj);

			// FIXME: Problem with bullet code, only one solver body created for static objects!
			// There could be more than one static object created by us!
			CPhysicsObject *pObj0 = (CPhysicsObject *)body0->m_originalColObj->getUserPointer();
			CPhysicsObject *pObj1 = (CPhysicsObject *)body1->m_originalColObj->getUserPointer();

			unsigned int flags0 = pObj0->GetCallbackFlags();
			unsigned int flags1 = pObj1->GetCallbackFlags();

			// Don't do the callback if it's disabled on either object
			if (!(flags0 & flags1 & CALLBACK_GLOBAL_FRICTION)) return;

			// If the solver uses 2 friction directions
			btSolverConstraint *constraint2 = NULL;
			if (m_pEnv->GetBulletEnvironment()->getSolverInfo().m_solverMode & SOLVER_USE_2_FRICTION_DIRECTIONS) {
				constraint2 = constraint + 1; // Always stored after the first one
			}

			// Calculate the energy consumed
			float totalImpulse = constraint->m_appliedImpulse + (constraint2 ? constraint2->m_appliedImpulse : 0);
			totalImpulse *= rb0->getInvMass(); // Get just the velocity
			totalImpulse *= totalImpulse; // Square it
			totalImpulse *= SAFE_DIVIDE(.5, rb0->getInvMass()); // Add back the mass (1/2*mv^2)

			if (m_pCallback)
				m_pCallback->Friction(pObj0)
			*/
		}

		void SetCollisionEventCallback(IPhysicsCollisionEvent *pCallback) {
			m_pCallback = pCallback;
		}

	private:
		CPhysicsEnvironment *m_pEnv;
		IPhysicsCollisionEvent *m_pCallback;

		// Temp. variables saved between Pre/PostCollision
		btVector3 m_tmpVelocities[2];
		btVector3 m_tmpAngVelocities[2];
		vcollisionevent_t m_tmpEvent{};
};

/*******************************
* Bullet Dynamics World Static References
*******************************/

// TODO: See if this dynamics world pointer is right, it's assigned twice
static bool gBulletDynamicsWorldGuard = false;
static btDynamicsWorld* gBulletDynamicsWorld = NULL;

#ifdef BT_THREADSAFE
static bool gMultithreadedWorld = true;
static SolverType gSolverType = SOLVER_TYPE_SEQUENTIAL_IMPULSE_MT;
#else
static bool gMultithreadedWorld = false;
static SolverType gSolverType = SOLVER_TYPE_SEQUENTIAL_IMPULSE;
#endif

static int gSolverMode = SOLVER_SIMD | SOLVER_USE_WARMSTARTING |
	/* SOLVER_RANDMIZE_ORDER | SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS | SOLVER_USE_2_FRICTION_DIRECTIONS |*/ 0;

/*******************************
* Bullet Dynamics World ConVars
*******************************/

// bt_solveriterations
static void cvar_solver_iterations_Change(IConVar *var, const char *pOldValue, float flOldValue);
static ConVar cvar_solver_iterations("bt_solver_iterations", "4", FCVAR_REPLICATED, "Number of collision solver iterations", true, 1, true, 32, cvar_solver_iterations_Change);
static void cvar_solver_iterations_Change(IConVar *var, const char *pOldValue, float flOldValue)
{
	if(gBulletDynamicsWorld)
	{
		gBulletDynamicsWorld->getSolverInfo().m_numIterations = cvar_solver_iterations.GetInt();
		Msg("Solver iteration count is changed from %i to %i\n", static_cast<int>(flOldValue), cvar_solver_iterations.GetInt());
	}
}

// bt_solver_residualthreshold
static void cvar_solver_residualthreshold_Change(IConVar *var, const char *pOldValue, float flOldValue);
static ConVar cvar_solver_residualthreshold("bt_solver_residualthreshold", "0.0", FCVAR_REPLICATED, "Solver leastSquaresResidualThreshold (used to run fewer solver iterations when convergence is good)", true, 0.0f, true, 0.25f, cvar_solver_residualthreshold_Change);
static void cvar_solver_residualthreshold_Change(IConVar *var, const char *pOldValue, float flOldValue)
{
	if (gBulletDynamicsWorld)
	{
		gBulletDynamicsWorld->getSolverInfo().m_leastSquaresResidualThreshold = cvar_solver_residualthreshold.GetFloat();
		Msg("Solver residual threshold is changed from %f to %f\n", flOldValue, cvar_solver_residualthreshold.GetFloat());
	}
}

// bt_substeps
static ConVar bt_max_world_substeps("bt_max_world_substeps", "5", FCVAR_REPLICATED, "Maximum amount of catchup simulation-steps BulletPhysics is allowed to do per Simulation()", true, 1, true, 12);

// Threadsafe specific console variables
#ifdef BT_THREADSAFE

// bt_threadcount
static void cvar_threadcount_Change(IConVar *var, const char *pOldValue, float flOldValue);
static ConVar cvar_threadcount("bt_threadcount", "1", FCVAR_REPLICATED, "Number of cores utilized by bullet task scheduler. By default, TBB sets this to optimal value", true, 1, true, static_cast<float>(BT_MAX_THREAD_COUNT), cvar_threadcount_Change);
static void cvar_threadcount_Change(IConVar *var, const char *pOldValue, float flOldValue)
{
	const int newNumThreads = MIN(cvar_threadcount.GetInt(), int(BT_MAX_THREAD_COUNT));
	const int oldNumThreads = btGetTaskScheduler()->getNumThreads();
	// only call when the thread count is different
	if (newNumThreads != oldNumThreads)
	{
		btGetTaskScheduler()->setNumThreads(newNumThreads);
		Msg("Changed %s task scheduler thread count from %i to %i\n", btGetTaskScheduler()->getName(), oldNumThreads, newNumThreads);
	}
}

// bt_island_batchingthreshold
static void cvar_island_batchingthreshold_Change(IConVar *var, const char *pOldValue, float flOldValue);
static ConVar cvar_island_batchingthreshold("bt_solver_islandbatchingthreshold", std::to_string(btSequentialImpulseConstraintSolverMt::s_minimumContactManifoldsForBatching).c_str(), FCVAR_REPLICATED, "If the number of manifolds that an island have reaches to that value, they will get batched", true, 1, true, 2000, cvar_island_batchingthreshold_Change);
static void cvar_island_batchingthreshold_Change(IConVar *var, const char *pOldValue, float flOldValue)
{
	btSequentialImpulseConstraintSolverMt::s_minimumContactManifoldsForBatching = cvar_island_batchingthreshold.GetInt();
	Msg("Island batching threshold is changed from %i to %i\n", static_cast<int>(flOldValue), cvar_island_batchingthreshold.GetInt());
}

// bt_solver_minbatchsize
static void cvar_solver_minbatchsize_Change(IConVar *var, const char *pOldValue, float flOldValue);
static ConVar cvar_solver_minbatchsize("bt_solver_minbatchsize", std::to_string(btSequentialImpulseConstraintSolverMt::s_minBatchSize).c_str(), FCVAR_REPLICATED, "Minimum size of batches for solver", true, 1, true, 1000, cvar_solver_minbatchsize_Change);

// bt_solver_maxbatchsize
static void cvar_solver_maxbatchsize_Change(IConVar *var, const char *pOldValue, float flOldValue);
static ConVar cvar_solver_maxbatchsize("bt_solver_maxbatchsize", std::to_string(btSequentialImpulseConstraintSolverMt::s_maxBatchSize).c_str(), FCVAR_REPLICATED, "Maximum size of batches for solver", true, 1, true, 1000, cvar_solver_maxbatchsize_Change);

static void cvar_solver_minbatchsize_Change(IConVar *var, const char *pOldValue, float flOldValue)
{
	cvar_solver_maxbatchsize.SetValue(MAX(cvar_solver_maxbatchsize.GetInt(), cvar_solver_minbatchsize.GetInt()));
	btSequentialImpulseConstraintSolverMt::s_minBatchSize = cvar_solver_minbatchsize.GetInt();
	btSequentialImpulseConstraintSolverMt::s_maxBatchSize = cvar_solver_maxbatchsize.GetInt();

	Msg("Min batch size for solver is changed from %i to %i\n", flOldValue, cvar_solver_minbatchsize.GetFloat());
}

static void cvar_solver_maxbatchsize_Change(IConVar *var, const char *pOldValue, float flOldValue)
{
	cvar_solver_minbatchsize.SetValue(MIN(cvar_solver_maxbatchsize.GetInt(), cvar_solver_minbatchsize.GetInt()));
	btSequentialImpulseConstraintSolverMt::s_minBatchSize = cvar_solver_minbatchsize.GetInt();
	btSequentialImpulseConstraintSolverMt::s_maxBatchSize = cvar_solver_maxbatchsize.GetInt();

	Msg("Max batch size for solver is changed from %i to %i\n", flOldValue, cvar_solver_maxbatchsize.GetFloat());
}

#endif


/*******************************
* CLASS CPhysicsEnvironment
*******************************/

CPhysicsEnvironment::CPhysicsEnvironment() {
	m_multithreadedWorld = false;
	m_multithreadCapable = false;
	m_deleteQuick		= false;
	m_bUseDeleteQueue	= false;
	m_inSimulation		= false;
	m_bConstraintNotify = false;
	m_pDebugOverlay		= NULL;
	m_pConstraintEvent	= NULL;
	m_pObjectEvent		= NULL;
	m_pObjectTracker	= NULL;
	m_pCollisionEvent	= NULL;
	m_pThreadManager	= NULL;

	m_pBulletBroadphase		= NULL;
	m_pBulletConfiguration	= NULL;
	m_pBulletDispatcher		= NULL;
	m_pBulletDynamicsWorld	= NULL;
	m_pBulletGhostCallback	= NULL;
	m_pBulletSolver			= NULL;

	m_timestep = 0.f;
	m_invPSIScale = 0.f;
	m_simPSICurrent = 0;
	m_simPSI = 0;
	m_simTime = 0.0f;

#ifdef BT_THREADSAFE
	// Initilize task scheduler, we will be using TBB
	// btSetTaskScheduler(btGetSequentialTaskScheduler()); // Can be used for debugging purposes

	//btSetTaskScheduler(btGetTBBTaskScheduler());
	//const int maxNumThreads = btGetTBBTaskScheduler()->getMaxNumThreads();
	//btGetTBBTaskScheduler()->setNumThreads(maxNumThreads);

    btSetTaskScheduler(btGetOpenMPTaskScheduler());
    //lwss: this is silly to use all cpu threads, how about just 2
    btGetOpenMPTaskScheduler()->setNumThreads(2);
	cvar_threadcount.SetValue(2);
#endif
	
	// Create a fresh new dynamics world
	CreateEmptyDynamicsWorld();
}

CPhysicsEnvironment::~CPhysicsEnvironment() {
#if DEBUG_DRAW
	delete m_debugdraw;
#endif
	CPhysicsEnvironment::SetQuickDelete(true);

	for (int i = m_objects.Count() - 1; i >= 0; --i) {
		delete m_objects[i];
	}

	m_objects.RemoveAll();
	CPhysicsEnvironment::CleanupDeleteList();

	delete m_pDeleteQueue;
	delete m_pPhysicsDragController;

	delete m_pBulletDynamicsWorld;
	delete m_pBulletSolver;
	delete m_pBulletBroadphase;
	delete m_pBulletDispatcher;
	delete m_pBulletConfiguration;
	delete m_pBulletGhostCallback;

	// delete m_pCollisionListener;
	delete m_pCollisionSolver;
	delete m_pObjectTracker;
}

btConstraintSolver* createSolverByType(SolverType t)
{
	btMLCPSolverInterface* mlcpSolver = NULL;
	switch (t)
	{
		case SOLVER_TYPE_SEQUENTIAL_IMPULSE:
			return new btSequentialImpulseConstraintSolver();
		case SOLVER_TYPE_SEQUENTIAL_IMPULSE_MT:
			return new btSequentialImpulseConstraintSolverMt();
		case SOLVER_TYPE_NNCG:
			return new btNNCGConstraintSolver();
		case SOLVER_TYPE_MLCP_PGS:
			mlcpSolver = new btSolveProjectedGaussSeidel();
			break;
		case SOLVER_TYPE_MLCP_DANTZIG:
			mlcpSolver = new btDantzigSolver();
			break;
		case SOLVER_TYPE_MLCP_LEMKE:
			mlcpSolver = new btLemkeSolver();
			break;
		default:
			assert(false);
			break;
	}
	if (mlcpSolver)
	{
		return new btMLCPSolver(mlcpSolver);
	}
	return NULL;
}

IVPhysicsDebugOverlay *g_pDebugOverlay = NULL;
void CPhysicsEnvironment::CreateEmptyDynamicsWorld()
{
	if(gBulletDynamicsWorldGuard)
	{
		gBulletDynamicsWorld = m_pBulletDynamicsWorld;
	}
	else
	{
		gBulletDynamicsWorldGuard = true;
	}

	m_pCollisionListener = new CCollisionEventListener(this);
	
	m_solverType = gSolverType;
#ifdef BT_THREADSAFE
	btAssert(btGetTaskScheduler() != NULL);
	if (btGetTaskScheduler() != NULL && btGetTaskScheduler()->getNumThreads() > 1)
	{
		m_multithreadCapable = true;
	}
#endif
	if (gMultithreadedWorld)
	{
#ifdef BT_THREADSAFE
		btAssert(btGetTaskScheduler() != NULL);

		m_pBulletDispatcher = NULL;
		btDefaultCollisionConstructionInfo cci;
		cci.m_defaultMaxPersistentManifoldPoolSize = 80000;
		cci.m_defaultMaxCollisionAlgorithmPoolSize = 80000;
		m_pBulletConfiguration = new btDefaultCollisionConfiguration(cci);

		// Dispatcher generates around 360 pair objects on average. Maximize thread usage by using this value
		m_pBulletDispatcher = new btCollisionDispatcherMt(m_pBulletConfiguration, 360 / cvar_threadcount.GetInt() + 1);
		m_pBulletBroadphase = new btDbvtBroadphase();

		// Enable deferred collide, increases performance with many collisions calculations going on at the same time
		static_cast<btDbvtBroadphase*>(m_pBulletBroadphase)->m_deferedcollide = true;

		btConstraintSolverPoolMt* solverPool;
		{
			SolverType poolSolverType = m_solverType;
			if (poolSolverType == SOLVER_TYPE_SEQUENTIAL_IMPULSE_MT)
			{
				// pool solvers shouldn't be parallel solvers, we don't allow that kind of
				// nested parallelism because of performance issues
				poolSolverType = SOLVER_TYPE_SEQUENTIAL_IMPULSE;
			}
			CUtlVector<btConstraintSolver*> solvers;
			const int threadCount = cvar_threadcount.GetInt();
			for (int i = 0; i < threadCount; ++i)
			{
				auto solver = createSolverByType(poolSolverType);
				solver->setSolveCallback(m_pCollisionListener);
				solvers.AddToTail(solver);
			}
			solverPool = new btConstraintSolverPoolMt(solvers.Base(), threadCount);
			m_pBulletSolver = solverPool;
			
			m_pBulletSolver->setSolveCallback(m_pCollisionListener);
		}
		btSequentialImpulseConstraintSolverMt* solverMt = NULL;
		if (m_solverType == SOLVER_TYPE_SEQUENTIAL_IMPULSE_MT)
		{
			solverMt = new btSequentialImpulseConstraintSolverMt();
		}
		btDiscreteDynamicsWorld* world = new btDiscreteDynamicsWorldMt(m_pBulletDispatcher, m_pBulletBroadphase, solverPool, solverMt, m_pBulletConfiguration);
		m_pBulletDynamicsWorld = world;
		m_pBulletDynamicsWorld->setForceUpdateAllAabbs(false);
		
		gBulletDynamicsWorld = world; // Also keep a static ref for ConVar callbacks
		m_multithreadedWorld = true;
#endif  // #if BT_THREADSAFE
	}
	else
	{
		// single threaded world
		m_multithreadedWorld = false;

		///collision configuration contains default setup for memory, collision setup
		m_pBulletConfiguration = new btDefaultCollisionConfiguration();

		// Use the default collision dispatcher. For parallel processing you can use a different dispatcher (see Extras/BulletMultiThreaded)
		m_pBulletDispatcher = new btCollisionDispatcher(m_pBulletConfiguration);

		m_pBulletBroadphase = new btDbvtBroadphase();

		SolverType solverType = m_solverType;
		if (solverType == SOLVER_TYPE_SEQUENTIAL_IMPULSE_MT)
		{
			// using the parallel solver with the single-threaded world works, but is
			// disabled here to avoid confusion
			solverType = SOLVER_TYPE_SEQUENTIAL_IMPULSE;
		}
		m_pBulletSolver = createSolverByType(solverType);
		m_pBulletSolver->setSolveCallback(m_pCollisionListener);

		m_pBulletDynamicsWorld = new btDiscreteDynamicsWorld(m_pBulletDispatcher, m_pBulletBroadphase, m_pBulletSolver, m_pBulletConfiguration);
	}
	m_pBulletDynamicsWorld->getSolverInfo().m_solverMode = gSolverMode;
	m_pBulletDynamicsWorld->getSolverInfo().m_numIterations = cvar_solver_iterations.GetInt();
	
	m_pBulletGhostCallback = new btGhostPairCallback;
	m_pCollisionSolver = new CCollisionSolver(this);
	m_pBulletDynamicsWorld->getPairCache()->setOverlapFilterCallback(m_pCollisionSolver);
	m_pBulletBroadphase->getOverlappingPairCache()->setInternalGhostPairCallback(m_pBulletGhostCallback);

	m_pDeleteQueue = new CDeleteQueue;
	m_pPhysicsDragController = new CPhysicsDragController;
	m_pObjectTracker = new CObjectTracker(this, NULL);

	m_perfparams.Defaults();
	memset(&m_stats, 0, sizeof(m_stats));

	// TODO: Threads solve any oversized batches (>32?), otherwise solving done on main thread.
	m_pBulletDynamicsWorld->getSolverInfo().m_minimumSolverBatchSize = 128; // Combine islands up to this many constraints
	m_pBulletDynamicsWorld->getDispatchInfo().m_allowedCcdPenetration = 0.0001f;
	m_pBulletDynamicsWorld->setApplySpeculativeContactRestitution(true);

	m_pBulletDynamicsWorld->setInternalTickCallback(TickCallback, (void *)this);


	// HACK: Get ourselves a debug overlay on the client
	// CreateInterfaceFn engine = Sys_GetFactory("engine");
	// CPhysicsEnvironment::SetDebugOverlay(engine);
	m_pDebugOverlay = g_Physics.GetPhysicsDebugOverlay();
	g_pDebugOverlay = m_pDebugOverlay;

#if DEBUG_DRAW
    m_debugdraw = new CDebugDrawer(m_pBulletDynamicsWorld);
    m_debugdraw->SetDebugOverlay( m_pDebugOverlay );
#endif
}

// Don't call this directly
void CPhysicsEnvironment::TickCallback(btDynamicsWorld *world, btScalar timeStep) {
	if (!world) return;

	CPhysicsEnvironment *pEnv = static_cast<CPhysicsEnvironment*>(world->getWorldUserInfo());
	if (pEnv)
		pEnv->BulletTick(timeStep);
}

//void CPhysicsEnvironment::SetDebugOverlay(CreateInterfaceFn debugOverlayFactory) {
//	if (debugOverlayFactory && !g_pDebugOverlay)
//		g_pDebugOverlay = static_cast<IVPhysicsDebugOverlay*>(debugOverlayFactory(VPHYSICS_DEBUG_OVERLAY_INTERFACE_VERSION, NULL));
//
//#if DEBUG_DRAW
//	if (g_pDebugOverlay)
//		m_debugdraw->SetDebugOverlay(g_pDebugOverlay);
//#endif
//}

IVPhysicsDebugOverlay *CPhysicsEnvironment::GetDebugOverlay() {
	return g_pDebugOverlay;
}

btIDebugDraw *CPhysicsEnvironment::GetDebugDrawer() const
{
	return reinterpret_cast<btIDebugDraw*>(m_debugdraw);
}

void CPhysicsEnvironment::SetGravity(const Vector &gravityVector) {
	btVector3 temp;
	ConvertPosToBull(gravityVector, temp);

	m_pBulletDynamicsWorld->setGravity(temp);
}

void CPhysicsEnvironment::GetGravity(Vector *pGravityVector) const {
	if (!pGravityVector) return;

	const btVector3 temp = m_pBulletDynamicsWorld->getGravity();
	ConvertPosToHL(temp, *pGravityVector);
}

void CPhysicsEnvironment::SetAirDensity(float density) {
	m_pPhysicsDragController->SetAirDensity(density);
}

float CPhysicsEnvironment::GetAirDensity() const {
	return m_pPhysicsDragController->GetAirDensity();
}

IPhysicsObject *CPhysicsEnvironment::CreatePolyObject(const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams) {
	IPhysicsObject *pObject = CreatePhysicsObject(this, pCollisionModel, materialIndex, position, angles, pParams, false);
	m_objects.AddToTail(pObject);
	return pObject;
}

IPhysicsObject *CPhysicsEnvironment::CreatePolyObjectStatic(const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams) {
	IPhysicsObject *pObject = CreatePhysicsObject(this, pCollisionModel, materialIndex, position, angles, pParams, true);
	m_objects.AddToTail(pObject);
	return pObject;
}

// Deprecated. Create a sphere model using collision interface.
IPhysicsObject *CPhysicsEnvironment::CreateSphereObject(float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic) {
	IPhysicsObject *pObject = CreatePhysicsSphere(this, radius, materialIndex, position, angles, pParams, isStatic);
	m_objects.AddToTail(pObject);
	return pObject;
}

void CPhysicsEnvironment::DestroyObject(IPhysicsObject *pObject) {
	if (!pObject) return;
	Assert(m_deadObjects.Find(pObject) == -1);	// If you hit this assert, the object is already on the list!

	m_objects.FindAndRemove(pObject);
	m_pObjectTracker->ObjectRemoved(dynamic_cast<CPhysicsObject*>(pObject));

	if (m_inSimulation || m_bUseDeleteQueue) {
		// We're still in the simulation, so deleting an object would be disastrous here. Queue it!
		dynamic_cast<CPhysicsObject*>(pObject)->AddCallbackFlags(CALLBACK_MARKED_FOR_DELETE);
		m_deadObjects.AddToTail(pObject);
	} else {
		delete pObject;
	}
}

IPhysicsFluidController *CPhysicsEnvironment::CreateFluidController(IPhysicsObject *pFluidObject, fluidparams_t *pParams) {
	CPhysicsFluidController *pFluid = ::CreateFluidController(this, static_cast<CPhysicsObject*>(pFluidObject), pParams);
	if (pFluid)
		m_fluids.AddToTail(pFluid);

	return pFluid;
}

void CPhysicsEnvironment::DestroyFluidController(IPhysicsFluidController *pController) {
	m_fluids.FindAndRemove(dynamic_cast<CPhysicsFluidController*>(pController));
	delete pController;
}

IPhysicsSpring *CPhysicsEnvironment::CreateSpring(IPhysicsObject *pObjectStart, IPhysicsObject *pObjectEnd, springparams_t *pParams) {
	return ::CreateSpringConstraint(this, pObjectStart, pObjectEnd, pParams);
}

void CPhysicsEnvironment::DestroySpring(IPhysicsSpring *pSpring) {
	if (!pSpring) return;

	CPhysicsConstraint* pConstraint = reinterpret_cast<CPhysicsConstraint*>(pSpring);

	if (m_deleteQuick) {
		IPhysicsObject *pObj0 = pConstraint->GetReferenceObject();
		if (pObj0 && !pObj0->IsStatic())
			pObj0->Wake();

		IPhysicsObject *pObj1 = pConstraint->GetAttachedObject();
		if (pObj1 && !pObj0->IsStatic())
			pObj1->Wake();
	}

	if (m_inSimulation) {
		pConstraint->Deactivate();
		m_pDeleteQueue->QueueForDelete(pSpring);
	} else {
		delete pSpring;
	}
}

IPhysicsConstraint *CPhysicsEnvironment::CreateRagdollConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll) {
	return ::CreateRagdollConstraint(this, pReferenceObject, pAttachedObject, pGroup, ragdoll);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateHingeConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge) {
	return ::CreateHingeConstraint(this, pReferenceObject, pAttachedObject, pGroup, hinge);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateFixedConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed) {
	return ::CreateFixedConstraint(this, pReferenceObject, pAttachedObject, pGroup, fixed);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateSlidingConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding) {
	return ::CreateSlidingConstraint(this, pReferenceObject, pAttachedObject, pGroup, sliding);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateBallsocketConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket) {
	return ::CreateBallsocketConstraint(this, pReferenceObject, pAttachedObject, pGroup, ballsocket);
}

IPhysicsConstraint *CPhysicsEnvironment::CreatePulleyConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley) {
	return ::CreatePulleyConstraint(this, pReferenceObject, pAttachedObject, pGroup, pulley);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateLengthConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length) {
	return ::CreateLengthConstraint(this, pReferenceObject, pAttachedObject, pGroup, length);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateGearConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_gearparams_t &gear) {
	return ::CreateGearConstraint(this, pReferenceObject, pAttachedObject, pGroup, gear);
}

IPhysicsConstraint *CPhysicsEnvironment::CreateUserConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, IPhysicsUserConstraint *pConstraint) {
	return ::CreateUserConstraint(this, pReferenceObject, pAttachedObject, pGroup, pConstraint);
}

void CPhysicsEnvironment::DestroyConstraint(IPhysicsConstraint *pConstraint) {
	if (!pConstraint) return;

	if (m_deleteQuick) {
		IPhysicsObject *pObj0 = pConstraint->GetReferenceObject();
		if (pObj0 && !pObj0->IsStatic())
			pObj0->Wake();

		IPhysicsObject *pObj1 = pConstraint->GetAttachedObject();
		if (pObj1 && !pObj1->IsStatic())
			pObj1->Wake();
	}

	if (m_inSimulation) {
		pConstraint->Deactivate();
		m_pDeleteQueue->QueueForDelete(pConstraint);
	} else {
		delete pConstraint;
	}
}

IPhysicsConstraintGroup *CPhysicsEnvironment::CreateConstraintGroup(const constraint_groupparams_t &groupParams) {
	return ::CreateConstraintGroup(this, groupParams);
}

void CPhysicsEnvironment::DestroyConstraintGroup(IPhysicsConstraintGroup *pGroup) {
	delete pGroup;
}

IPhysicsShadowController *CPhysicsEnvironment::CreateShadowController(IPhysicsObject *pObject, bool allowTranslation, bool allowRotation) {
	CShadowController *pController = ::CreateShadowController(pObject, allowTranslation, allowRotation);
	if (pController)
		m_controllers.AddToTail(pController);

	return pController;
}

void CPhysicsEnvironment::DestroyShadowController(IPhysicsShadowController *pController) {
	if (!pController) return;

	m_controllers.FindAndRemove(static_cast<CShadowController*>(pController));
	delete pController;
}

IPhysicsPlayerController *CPhysicsEnvironment::CreatePlayerController(IPhysicsObject *pObject) {
	CPlayerController *pController = ::CreatePlayerController(this, pObject);
	if (pController)
		m_controllers.AddToTail(pController);

	return pController;
}

void CPhysicsEnvironment::DestroyPlayerController(IPhysicsPlayerController *pController) {
	if (!pController) return;

	m_controllers.FindAndRemove(dynamic_cast<CPlayerController*>(pController));
	delete pController;
}

IPhysicsMotionController *CPhysicsEnvironment::CreateMotionController(IMotionEvent *pHandler) {
	CPhysicsMotionController *pController = dynamic_cast<CPhysicsMotionController*>(::CreateMotionController(this, pHandler));
	if (pController)
		m_controllers.AddToTail(pController);

	return pController;
}

void CPhysicsEnvironment::DestroyMotionController(IPhysicsMotionController *pController) {
	if (!pController) return;

	m_controllers.FindAndRemove(static_cast<CPhysicsMotionController*>(pController));
	delete pController;
}

IPhysicsVehicleController *CPhysicsEnvironment::CreateVehicleController(IPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace) {
	return ::CreateVehicleController(this, static_cast<CPhysicsObject*>(pVehicleBodyObject), params, nVehicleType, pGameTrace);
}

void CPhysicsEnvironment::DestroyVehicleController(IPhysicsVehicleController *pController) {
	delete pController;
}

void CPhysicsEnvironment::SetCollisionSolver(IPhysicsCollisionSolver *pSolver) {
	m_pCollisionSolver->SetHandler(pSolver);
}

void CPhysicsEnvironment::Simulate(float deltaTime) {
	Assert(m_pBulletDynamicsWorld);

	// Input deltaTime is how many seconds have elapsed since the previous frame
	// phys_timescale can scale this parameter however...
	// Can cause physics to slow down on the client environment when the game's window is not in focus
	// Also is affected by the tickrate as well

	if (deltaTime > 1.0 || deltaTime < 0.0) {
		deltaTime = 0;
	} else if (deltaTime > 0.1) {
		deltaTime = 0.1f;
	}

	// sim PSI: How many substeps are done in a single simulation step
	m_simPSI = bt_max_world_substeps.GetInt() != 0 ? bt_max_world_substeps.GetInt() : 1;
	m_simPSICurrent = m_simPSI; // Substeps left in this step
	m_numSubSteps = m_simPSI;
	m_curSubStep = 0;
	
	// Simulate no less than 1 ms
	if (deltaTime > 0.001) {
		// Now mark us as being in simulation. This is used for callbacks from bullet mid-simulation
		// so we don't end up doing stupid things like deleting objects still in use
		m_inSimulation = true;

		m_subStepTime = m_timestep;

		// Okay, how this fixed timestep shit works:
		// The game sends in deltaTime which is the amount of time that has passed since the last frame
		// Bullet will add the deltaTime to its internal counter
		// When this internal counter exceeds m_timestep (param 3 to the below), the simulation will run for fixedTimeStep seconds
		// If the internal counter does not exceed fixedTimeStep, bullet will just interpolate objects so the game can render them nice and happy
		//if( deltaTime > (bt_max_world_substeps.GetInt() * m_timestep) )
        //{
		//    Msg("Warning! We are losing physics-time! - deltaTime(%f) - maxTime(%f)\n", deltaTime, (bt_max_world_substeps.GetInt() * m_timestep));
        //}
		int stepsTaken = m_pBulletDynamicsWorld->stepSimulation(deltaTime, bt_max_world_substeps.GetInt(), m_timestep, m_simPSICurrent);

		//lwss: stepSimulation returns the number of steps taken in the simulation this go-around.
		// We can simply multiply this by the timestep(60hz client - 64hz server) to get the time simulated.
		// and add it to our simtime.
		m_simTime += ( stepsTaken * m_timestep );

		// No longer in simulation!
		m_inSimulation = false;
	}

#if DEBUG_DRAW
	m_debugdraw->DrawWorld();
#endif
}

bool CPhysicsEnvironment::IsInSimulation() const {
	return m_inSimulation;
}

float CPhysicsEnvironment::GetSimulationTimestep() const {
	return m_timestep;
}

void CPhysicsEnvironment::SetSimulationTimestep(float timestep) {
	m_timestep = timestep;
}

float CPhysicsEnvironment::GetSimulationTime() const {
    //lwss - add simulation time. (mainly used for forced ragdoll sleeping hack in csgo)
	return m_simTime;
}

void CPhysicsEnvironment::ResetSimulationClock() {
    m_simTime = 0.0f;
}

float CPhysicsEnvironment::GetNextFrameTime() const {
	NOT_IMPLEMENTED
	return 0;
}

void CPhysicsEnvironment::SetCollisionEventHandler(IPhysicsCollisionEvent *pCollisionEvents) {
	//lwss: This enables the CCollisionEvent::PreCollision() and CCollisionEvent::PostCollision()
	// in client/physics.cpp and server/physics.cpp
	// Used for: Physics hit sounds, dust particles, and screen shake(unused?)
	// The accuracy of the physics highly depends on the stepSimulation() maxSteps and resolution.
	//
	// There is another section that calls m_pCollisionEvent->Friction().
	// That section is somewhat similar, But deals with friction Scrapes instead of impacts.
	// It is ran on the server, which sends the particle/sound commands to the client

    // TODO
    m_pCollisionListener->SetCollisionEventCallback(pCollisionEvents);
	m_pCollisionEvent = pCollisionEvents;
}

void CPhysicsEnvironment::SetObjectEventHandler(IPhysicsObjectEvent *pObjectEvents) {
	m_pObjectEvent = pObjectEvents;

	m_pObjectTracker->SetObjectEventHandler(pObjectEvents);
}

void CPhysicsEnvironment::SetConstraintEventHandler(IPhysicsConstraintEvent *pConstraintEvents) {
	m_pConstraintEvent = pConstraintEvents;
}

void CPhysicsEnvironment::SetQuickDelete(bool bQuick) {
	m_deleteQuick = bQuick;
}

int CPhysicsEnvironment::GetActiveObjectCount() const {
	return m_pObjectTracker->GetActiveObjectCount();
}

void CPhysicsEnvironment::GetActiveObjects(IPhysicsObject **pOutputObjectList) const {
	return m_pObjectTracker->GetActiveObjects(pOutputObjectList);
}

int CPhysicsEnvironment::GetObjectCount() const {
	return m_objects.Count();
}

const IPhysicsObject **CPhysicsEnvironment::GetObjectList(int *pOutputObjectCount) const {
	if (pOutputObjectCount) {
		*pOutputObjectCount = m_objects.Count();
	}

	return const_cast<const IPhysicsObject**>(m_objects.Base());
}

bool CPhysicsEnvironment::TransferObject(IPhysicsObject *pObject, IPhysicsEnvironment *pDestinationEnvironment) {
	if (!pObject || !pDestinationEnvironment) return false;

	if (pDestinationEnvironment == this) {
		dynamic_cast<CPhysicsObject*>(pObject)->TransferToEnvironment(this);
		m_objects.AddToTail(pObject);
		if (pObject->IsFluid())
			m_fluids.AddToTail(dynamic_cast<CPhysicsObject*>(pObject)->GetFluidController());

		return true;
	} else {
		m_objects.FindAndRemove(pObject);
		if (pObject->IsFluid())
			m_fluids.FindAndRemove(dynamic_cast<CPhysicsObject*>(pObject)->GetFluidController());

		return pDestinationEnvironment->TransferObject(pObject, pDestinationEnvironment);
	}
}

void CPhysicsEnvironment::CleanupDeleteList() {
	for (int i = 0; i < m_deadObjects.Count(); i++) {
		delete m_deadObjects.Element(i);
	}

	m_deadObjects.Purge();
	m_pDeleteQueue->DeleteAll();
}

void CPhysicsEnvironment::EnableDeleteQueue(bool enable) {
	m_bUseDeleteQueue = enable;
}

bool CPhysicsEnvironment::Save(const physsaveparams_t &params) {
	NOT_IMPLEMENTED
	return false;
}

void CPhysicsEnvironment::PreRestore(const physprerestoreparams_t &params) {
	NOT_IMPLEMENTED
}

bool CPhysicsEnvironment::Restore(const physrestoreparams_t &params) {
	NOT_IMPLEMENTED
	return false;
}

void CPhysicsEnvironment::PostRestore() {
	NOT_IMPLEMENTED
}

//lwss add
void CPhysicsEnvironment::SetAlternateGravity(const Vector &gravityVector)
{
    ConvertPosToBull(gravityVector, m_vecRagdollGravity);
}

void CPhysicsEnvironment::GetAlternateGravity(Vector *pGravityVector) const
{
    ConvertPosToHL( m_vecRagdollGravity, *pGravityVector );
}

float CPhysicsEnvironment::GetDeltaFrameTime(int maxTicks) const
{
    //TODO - implement this for bullet

    // this is fully accurate, the IDA king has spoken.
    //double timeDiff = m_pPhysEnv->time_of_next_psi.get_time() - m_pPhysEnv->current_time.get_time();
    //return float( (float(maxTicks) * m_pPhysEnv->delta_PSI_time) + timeDiff );

    // HACK ALERT!!
    return 1.0f;
}

void CPhysicsEnvironment::ForceObjectsToSleep(IPhysicsObject **pList, int listCount)
{
    //lwss: Technically this does not force the objects to sleep, but works OK.
    for( int i = 0; i < listCount; i++ )
    {
        pList[i]->Sleep();
    }
}

void CPhysicsEnvironment::SetPredicted(bool bPredicted)
{
    // This check is a little different from retail
    if( m_objects.Count() > 0 || m_deadObjects.Count() > 0 )
    {
        Error( "Predicted physics are not designed to change once objects have been made.\n");
        /* Exit */
    }

    if( bPredicted )
        Warning("WARNING: Kisak physics does NOT have prediction!\n");

    //m_predictionEnabled = bPredicted;
}

bool CPhysicsEnvironment::IsPredicted()
{
    //lwss hack - I didn't redo the whole physics prediction.
    // return false so it doesn't try to use it.
    return false;
    //return m_predictionEnabled;
}

void CPhysicsEnvironment::SetPredictionCommandNum(int iCommandNum)
{
    if( !m_predictionEnabled )
        return;

    //lwss hack - didn't reimplement the entire physics prediction system.
    m_predictionCmdNum = iCommandNum;
    Warning("LWSS didn't implement SetPredictionCommandNum\n");
}

int CPhysicsEnvironment::GetPredictionCommandNum()
{
    return m_predictionCmdNum;
}

void CPhysicsEnvironment::DoneReferencingPreviousCommands(int iCommandNum)
{
    //lwss hack
    //Warning("LWSS didn't implement DoneReferencingPreviousCommands\n");
}

void CPhysicsEnvironment::RestorePredictedSimulation()
{
    //lwss hack
    Warning("LWSS didn't implement RestorePredictedSimulation\n");
}

void CPhysicsEnvironment::DestroyCollideOnDeadObjectFlush(CPhysCollide *)
{
    //lwss hack
    Warning("LWSS didn't implement DestroyCollideOnDeadObjectFlush\n");
    // m_lastObjectThisTick // +20 bytes
}
//lwss end

bool CPhysicsEnvironment::IsCollisionModelUsed(CPhysCollide *pCollide) const {
	for (int i = 0; i < m_objects.Count(); i++) {
		if (dynamic_cast<CPhysicsObject*>(m_objects[i])->GetObject()->getCollisionShape() == pCollide->GetCollisionShape())
			return true;
	}

	// Also scan the to-be-deleted objects list
	for (int i = 0; i < m_deadObjects.Count(); i++) {
		if (dynamic_cast<CPhysicsObject*>(m_deadObjects[i])->GetObject()->getCollisionShape() == pCollide->GetCollisionShape())
			return true;
	}

	return false;
}

void CPhysicsEnvironment::TraceRay(const Ray_t &ray, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace) {
	if (!ray.m_IsRay || !pTrace) return;

	btVector3 vecStart, vecEnd;
	ConvertPosToBull(ray.m_Start + ray.m_StartOffset, vecStart);
	ConvertPosToBull(ray.m_Start + ray.m_StartOffset + ray.m_Delta, vecEnd);

	// TODO: Override this class to use the mask and trace filter.
	btCollisionWorld::ClosestRayResultCallback cb(vecStart, vecEnd);
	m_pBulletDynamicsWorld->rayTest(vecStart, vecEnd, cb);

	pTrace->fraction = cb.m_closestHitFraction;
	ConvertPosToHL(cb.m_hitPointWorld, pTrace->endpos);
	ConvertDirectionToHL(cb.m_hitNormalWorld, pTrace->plane.normal);
}

// Is this function ever called?
// TODO: This is a bit more complex, bullet doesn't support compound sweep tests.
void CPhysicsEnvironment::SweepCollideable(const CPhysCollide *pCollide, const Vector &vecAbsStart, const Vector &vecAbsEnd, const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace) {
	NOT_IMPLEMENTED
}

class CFilteredConvexResultCallback : public btCollisionWorld::ClosestConvexResultCallback {
	public:
		CFilteredConvexResultCallback(IPhysicsTraceFilter *pFilter, unsigned int mask, const btVector3 &convexFromWorld, const btVector3 &convexToWorld):
		btCollisionWorld::ClosestConvexResultCallback(convexFromWorld, convexToWorld) {
			m_pTraceFilter = pFilter;
			m_mask = mask;
		}

		virtual bool needsCollision(btBroadphaseProxy *proxy0) const {
			btCollisionObject *pColObj = (btCollisionObject *)proxy0->m_clientObject;
			CPhysicsObject *pObj = (CPhysicsObject *)pColObj->getUserPointer();
			if (pObj && !m_pTraceFilter->ShouldHitObject(pObj, m_mask)) {
				return false;
			}

			bool collides = (proxy0->m_collisionFilterGroup & m_collisionFilterMask) != 0;
			collides = collides && (m_collisionFilterGroup & proxy0->m_collisionFilterMask);

			return collides;
		}

	private:
		IPhysicsTraceFilter *m_pTraceFilter;
		unsigned int m_mask;
};

void CPhysicsEnvironment::SweepConvex(const CPhysConvex *pConvex, const Vector &vecAbsStart, const Vector &vecAbsEnd, const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace) {
	if (!pConvex || !pTrace) return;
	
	btVector3 vecStart, vecEnd;
	ConvertPosToBull(vecAbsStart, vecStart);
	ConvertPosToBull(vecAbsEnd, vecEnd);

	btMatrix3x3 matAng;
	ConvertRotationToBull(vecAngles, matAng);

	btTransform transStart, transEnd;
	transStart.setOrigin(vecStart);
	transStart.setBasis(matAng);

	transEnd.setOrigin(vecEnd);
	transEnd.setBasis(matAng);

	btConvexShape *pShape = (btConvexShape *)pConvex;

	CFilteredConvexResultCallback cb(pTraceFilter, fMask, vecStart, vecEnd);
	m_pBulletDynamicsWorld->convexSweepTest(pShape, transStart, transEnd, cb, 0.0001f);

	pTrace->fraction = cb.m_closestHitFraction;
	ConvertPosToHL(cb.m_hitPointWorld, pTrace->endpos);
	ConvertDirectionToHL(cb.m_hitNormalWorld, pTrace->plane.normal);
}

void CPhysicsEnvironment::GetPerformanceSettings(physics_performanceparams_t *pOutput) const {
	if (!pOutput) return;

	*pOutput = m_perfparams;
}

void CPhysicsEnvironment::SetPerformanceSettings(const physics_performanceparams_t *pSettings) {
	if (!pSettings) return;

	m_perfparams = *pSettings;
}

void CPhysicsEnvironment::ReadStats(physics_stats_t *pOutput) {
	if (!pOutput) return;

	*pOutput = m_stats;
}

void CPhysicsEnvironment::ClearStats() {
	memset(&m_stats, 0, sizeof(m_stats));
}

unsigned int CPhysicsEnvironment::GetObjectSerializeSize(IPhysicsObject *pObject) const {
	NOT_IMPLEMENTED
	return 0;
}

void CPhysicsEnvironment::SerializeObjectToBuffer(IPhysicsObject *pObject, unsigned char *pBuffer, unsigned int bufferSize) {
	NOT_IMPLEMENTED
}

IPhysicsObject *CPhysicsEnvironment::UnserializeObjectFromBuffer(void *pGameData, unsigned char *pBuffer, unsigned int bufferSize, bool enableCollisions) {
	NOT_IMPLEMENTED
	return NULL;
}

void CPhysicsEnvironment::EnableConstraintNotify(bool bEnable) {
	// Notify game about broken constraints?
	m_bConstraintNotify = bEnable;
}

// FIXME: What do? Source SDK mods call this every frame in debug builds.
void CPhysicsEnvironment::DebugCheckContacts() {
	
}

// UNEXPOSED
btDiscreteDynamicsWorld *CPhysicsEnvironment::GetBulletEnvironment() const
{
	return m_pBulletDynamicsWorld;
}

// UNEXPOSED
float CPhysicsEnvironment::GetInvPSIScale() const
{
	return m_invPSIScale;
}

// UNEXPOSED
void CPhysicsEnvironment::BulletTick(btScalar dt) {
	// Dirty hack to spread the controllers throughout the current simulation step
	if (m_simPSICurrent) {
		m_invPSIScale = 1.0f / static_cast<float>(m_simPSICurrent);
		m_simPSICurrent--;
	} else {
		m_invPSIScale = 0;
	}

	m_pPhysicsDragController->Tick(dt);

	for (int i = 0; i < m_controllers.Count(); i++)
		m_controllers[i]->Tick(dt);

	for (int i = 0; i < m_fluids.Count(); i++)
		m_fluids[i]->Tick(dt);

	m_inSimulation = false;

	// Update object sleep states
	m_pObjectTracker->Tick();

	if (!m_bUseDeleteQueue) {
		CleanupDeleteList();
	}

	//lwss: This part of the code is used by the server to send FRICTION sounds/dust
	// these are separate from collisions.
	DoCollisionEvents(dt);
	//lwss end.

	if (m_pCollisionEvent)
		m_pCollisionEvent->PostSimulationFrame();

	m_inSimulation = true;
	m_curSubStep++;
}

// UNEXPOSED
CPhysicsDragController *CPhysicsEnvironment::GetDragController() const
{
	return m_pPhysicsDragController;
}

CCollisionSolver *CPhysicsEnvironment::GetCollisionSolver() const
{
	return m_pCollisionSolver;
}

btVector3 CPhysicsEnvironment::GetMaxLinearVelocity() const {
	return btVector3(HL2BULL(m_perfparams.maxVelocity), HL2BULL(m_perfparams.maxVelocity), HL2BULL(m_perfparams.maxVelocity));
}

btVector3 CPhysicsEnvironment::GetMaxAngularVelocity() const {
	return btVector3(HL2BULL(m_perfparams.maxAngularVelocity), HL2BULL(m_perfparams.maxAngularVelocity), HL2BULL(m_perfparams.maxAngularVelocity));
}

// UNEXPOSED
// Purpose: To be the biggest eyesore ever
// Bullet doesn't provide many callbacks such as the ones we're looking for, so
// we have to iterate through all the contact manifolds and generate the callbacks ourselves.
// FIXME: Remove this function and implement callbacks in bullet code
void CPhysicsEnvironment::DoCollisionEvents(float dt) {
	if (m_pCollisionEvent) {
		const int numManifolds = m_pBulletDynamicsWorld->getDispatcher()->getNumManifolds();
		for (int i = 0; i < numManifolds; i++) {
			btPersistentManifold *contactManifold = m_pBulletDynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
			if (contactManifold->getNumContacts() <= 0)
				continue;

			const btCollisionObject *obA = contactManifold->getBody0();
			const btCollisionObject *obB = contactManifold->getBody1();
			if (!obA || !obB) continue;

			CPhysicsObject *physObA = static_cast<CPhysicsObject*>(obA->getUserPointer());
			CPhysicsObject *physObB = static_cast<CPhysicsObject*>(obB->getUserPointer());

			// These are our own internal objects, don't do callbacks on them.
			if (obA->getInternalType() == btCollisionObject::CO_GHOST_OBJECT || obB->getInternalType() == btCollisionObject::CO_GHOST_OBJECT)
				continue;

			if (!(physObA->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION)  || !(physObB->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION))
				continue;

			for (int j = 0; j < contactManifold->getNumContacts(); j++) {
				btManifoldPoint manPoint = contactManifold->getContactPoint(j);

				// FRICTION CALLBACK
				// FIXME: We need to find the energy used by the friction! Bullet doesn't provide this in the manifold point.
				// This may not be the proper variable but whatever, as of now it works.
				const float energy = abs(manPoint.m_appliedImpulseLateral1);
				if (energy > 0.05f) {
					CPhysicsCollisionData data(&manPoint);
					m_pCollisionEvent->Friction(static_cast<CPhysicsObject*>(obA->getUserPointer()),
												ConvertEnergyToHL(energy),
												static_cast<CPhysicsObject*>(obA->getUserPointer())->GetMaterialIndex(),
												static_cast<CPhysicsObject*>(obB->getUserPointer())->GetMaterialIndex(),
												&data);
				}

				// TODO: Collision callback
				// Source wants precollision and postcollision callbacks (pre velocity and post velocity, etc.)
				// How do we generate a callback before the collision happens?
			}
		}
	}
}

// ==================
// EVENTS
// ==================

// UNEXPOSED
void CPhysicsEnvironment::HandleConstraintBroken(CPhysicsConstraint *pConstraint) const
{
	if (m_bConstraintNotify && m_pConstraintEvent)
		m_pConstraintEvent->ConstraintBroken(pConstraint);
}

// UNEXPOSED
void CPhysicsEnvironment::HandleFluidStartTouch(CPhysicsFluidController *pController, CPhysicsObject *pObject) const
{
	if (pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE) return;

	if (m_pCollisionEvent)
		m_pCollisionEvent->FluidStartTouch(pObject, pController);
}

// UNEXPOSED
void CPhysicsEnvironment::HandleFluidEndTouch(CPhysicsFluidController *pController, CPhysicsObject *pObject) const
{
	if (pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE) return;

	if (m_pCollisionEvent)
		m_pCollisionEvent->FluidEndTouch(pObject, pController);
}

// UNEXPOSED
void CPhysicsEnvironment::HandleObjectEnteredTrigger(CPhysicsObject *pTrigger, CPhysicsObject *pObject) const
{
	if (pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE) return;

	if (m_pCollisionEvent)
		m_pCollisionEvent->ObjectEnterTrigger(pTrigger, pObject);
}

// UNEXPOSED
void CPhysicsEnvironment::HandleObjectExitedTrigger(CPhysicsObject *pTrigger, CPhysicsObject *pObject) const
{
	if (pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE) return;

	if (m_pCollisionEvent)
		m_pCollisionEvent->ObjectLeaveTrigger(pTrigger, pObject);
}

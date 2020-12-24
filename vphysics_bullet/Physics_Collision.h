#ifndef PHYSICS_COLLISION_H
#define PHYSICS_COLLISION_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

// NOTE: There can only be up to 16 unique collision groups (data type of short)!
enum ECollisionGroups {
	COLGROUP_NONE	= 0,
	COLGROUP_WORLD	= 1<<1,
};

// Because the old vphysics had to do this.
struct bboxcache_t {
	CPhysCollide *	pCollide;
	Vector			mins, maxs;
};

class CPhysCollide {
	public:
		CPhysCollide(btCollisionShape *pShape);

		const btCollisionShape *GetCollisionShape() const {
			return m_pShape;
		}

		btCollisionShape *GetCollisionShape() {
			return m_pShape;
		}

		const btCompoundShape *GetCompoundShape() const {
			Assert(IsCompound());
			if (!IsCompound())
				return NULL;

			return (btCompoundShape *)m_pShape;
		}

		btCompoundShape *GetCompoundShape() {
			Assert(IsCompound());
			if (!IsCompound())
				return NULL;

			return (btCompoundShape *)m_pShape;
		}

		const btConvexShape *GetConvexShape() const {
			Assert(IsConvex());
			if (!IsConvex())
				return NULL;

			return (btConvexShape *)m_pShape;
		}

		btConvexShape *GetConvexShape() {
			Assert(IsConvex());
			if (!IsConvex())
				return NULL;

			return (btConvexShape *)m_pShape;
		}

		void SetRotationInertia(const btVector3 &inertia) {
			m_rotInertia = inertia;
		}

		btVector3 &GetRotationInertia() {
			return m_rotInertia;
		}

		const btVector3 &GetRotationInertia() const {
			return m_rotInertia;
		}

		void SetMassCenter(const btVector3 &center) {
			m_massCenter = center;
		}

		btVector3 &GetMassCenter() {
			return m_massCenter;
		}

		const btVector3 &GetMassCenter() const {
			return m_massCenter;
		}

		bool IsCompound() const {
			return m_pShape->isCompound();
		}

		bool IsConvex() const {
			return m_pShape->isConvex();
		}

	private:
		btCollisionShape *m_pShape;

		btVector3 m_rotInertia;
		btVector3 m_massCenter;
};

class CPhysicsCollision : public IPhysicsCollision32 {
	public:
		CPhysicsCollision();
		~CPhysicsCollision();

		CPhysConvex *			ConvexFromVerts(Vector **ppVerts, int vertCount);
		CPhysConvex *			ConvexFromVerts(const Vector *pVerts, int vertCount); // A more sensible version!
		CPhysConvex *			ConvexFromPlanes(float *pPlanes, int planeCount, float mergeDistance);
		float					ConvexVolume(CPhysConvex *pConvex);
		float					ConvexSurfaceArea(CPhysConvex *pConvex);
		void					SetConvexGameData(CPhysConvex *pConvex, unsigned int gameData);
		void					ConvexFree(CPhysConvex *pConvex);
		CPhysConvex *			ConvexFromConvexPolyhedron(const CPolyhedron &ConvexPolyhedron);
		CPolyhedron *			PolyhedronFromConvex(CPhysConvex *const pConvex, bool bUseTempPolyhedron);
		void					ConvexesFromConvexPolygon(const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **pOutput);

		CPhysPolysoup *			PolysoupCreate();
		void					PolysoupDestroy(CPhysPolysoup *pSoup);
		void					PolysoupAddTriangle(CPhysPolysoup *pSoup, const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits);

		CPhysCollide *			ConvertPolysoupToCollide(CPhysPolysoup *pSoup, bool useMOPP); // Deprecated: useMOPP
		CPhysCollide *			ConvertConvexToCollide(CPhysConvex **pConvex, int convexCount);
		CPhysCollide *			ConvertConvexToCollideParams(CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &convertParams);

		void					AddConvexToCollide(CPhysCollide *pCollide, const CPhysConvex *pConvex, const matrix3x4_t *xform = NULL);
		void					RemoveConvexFromCollide(CPhysCollide *pCollide, const CPhysConvex *pConvex);
		void					RemoveConvexFromCollide(CPhysCollide *pCollide, int index);

		CPhysCollide *			CreateCollide(); // Create an empty collision shape (to be used with AddConvexToCollide/RemoveConvexFromCollide)
		void					DestroyCollide(CPhysCollide *pCollide);
		int						CollideSize(CPhysCollide *pCollide);
		int						CollideWrite(char *pDest, CPhysCollide *pCollide, bool swap = false);
		CPhysCollide *			UnserializeCollide(char *pBuffer, int size, int index);
		float					CollideVolume(CPhysCollide *pCollide);
		float					CollideSurfaceArea(CPhysCollide *pCollide);
		Vector					CollideGetExtent(const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction);
		void					CollideGetAABB(Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles);
        // lwss - ADDED, these are all new for CSGO
        virtual float           CollideGetRadius( const CPhysCollide *pCollide );
        virtual void			*VCollideAllocUserData( vcollide_t *pVCollide, size_t userDataSize );
        virtual void			VCollideFreeUserData( vcollide_t *pVCollide );
        virtual void			VCollideCheck( vcollide_t *pVCollide, const char *pName );
        virtual bool			TraceBoxAA( const Ray_t &ray, const CPhysCollide *pCollide, trace_t *ptr );
        // lwss end
		void					CollideGetMassCenter(CPhysCollide *pCollide, Vector *pOutMassCenter);
		void					CollideSetMassCenter(CPhysCollide *pCollide, const Vector &massCenter);
		Vector					CollideGetOrthographicAreas(const CPhysCollide *pCollide);
		void					CollideSetOrthographicAreas(CPhysCollide *pCollide, const Vector &areas);
		void					CollideSetScale(CPhysCollide *pCollide, const Vector &scale);
		void					CollideGetScale(const CPhysCollide *pCollide, Vector &scale);
		int						CollideIndex(const CPhysCollide *pCollide);
		int						GetConvexesUsedInCollideable(const CPhysCollide *pCollideable, CPhysConvex **pOutputArray, int iOutputArrayLimit);

		// Some functions for old vphysics behavior. Do not use anything like these for newer collision shapes.

		CPhysCollide *			GetCachedBBox(const Vector &mins, const Vector &maxs);
		void					AddCachedBBox(CPhysCollide *pModel, const Vector &mins, const Vector &maxs);
		bool					IsCachedBBox(CPhysCollide *pModel);
		void					ClearBBoxCache();
		bool					GetBBoxCacheSize(int *pCachedSize, int *pCachedCount);

		// API for disabling old vphysics behavior.
		void					EnableBBoxCache(bool enable);
		bool					IsBBoxCacheEnabled();

		CPhysConvex *			BBoxToConvex(const Vector &mins, const Vector &maxs);
		CPhysCollide *			BBoxToCollide(const Vector &mins, const Vector &maxs);

		// Misc collision shapes

		CPhysConvex *			CylinderToConvex(const Vector &mins, const Vector &maxs); // Cylinder. Flat sides are on top/bottom (z axis)
		CPhysConvex *			ConeToConvex(const float radius, const float height);
		CPhysConvex *			SphereToConvex(const float radius);

		void					TraceBox(const Vector &start, const Vector &end, const Vector &mins, const Vector &maxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr);
		void					TraceBox(const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr);
		void					TraceBox(const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr);
		void					TraceCollide(const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr);
		void					TraceConvex(const Vector &start, const Vector &end, const CPhysConvex *pSweepConvex, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace);

		bool					IsBoxIntersectingCone(const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone);

		void					VCollideLoad(vcollide_t *pOutput, int solidCount, const char *pBuffer, int size, bool swap = false);
		void					VCollideUnload(vcollide_t *pVCollide);

		IVPhysicsKeyParser *	VPhysicsKeyParserCreate(const char *pKeyData);
        IVPhysicsKeyParser *    VPhysicsKeyParserCreate( vcollide_t *pVCollide ); // lwss - NEW for csgo

        void					VPhysicsKeyParserDestroy(IVPhysicsKeyParser *pParser);

		int						CreateDebugMesh(CPhysCollide const *pCollisionModel, Vector **outVerts);
		void					DestroyDebugMesh(int vertCount, Vector *outVerts);

		ICollisionQuery *		CreateQueryModel(CPhysCollide *pCollide);
		void					DestroyQueryModel(ICollisionQuery *pQuery);

		IPhysicsCollision *		ThreadContextCreate();
		void					ThreadContextDestroy(IPhysicsCollision *pThreadContex);

		CPhysCollide *			CreateVirtualMesh(const virtualmeshparams_t &params);
		bool					SupportsVirtualMesh();

		void					OutputDebugInfo(const CPhysCollide *pCollide);
		unsigned int			ReadStat(int statID);

	private:
		CUtlVector<bboxcache_t> m_bboxCache;
		bool					m_enableBBoxCache;
};

extern CPhysicsCollision g_PhysicsCollision;

#endif // PHYSICS_COLLISION_H

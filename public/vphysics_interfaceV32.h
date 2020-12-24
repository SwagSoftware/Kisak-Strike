#ifndef VPHYSICS_INTERFACEV32_H
#define VPHYSICS_INTERFACEV32_H

#ifdef _MSC_VER
	#pragma once
#endif

#include "tier1/interface.h"
#include "vphysics_interface.h"

// Purpose: Updated interface to vphysics.dll
// This interface MUST be ADDITIVE ONLY. DO NOT change old function signatures.
// To use this new interface, typecast the old interfaces to the newer ones
// ex. IPhysics32 *newPhysics = (IPhysics32 *)oldPhysics;

// THIS INTERFACE IS NOT FINALIZED! FUNCTIONS MAY CHANGE!

class IPhysicsSoftBody;
class IPhysicsConstraint;
class IPhysicsConstraintGroup;
class IPhysicsUserConstraint;
class IPhysicsVehicleController;

struct softbodyparams_t;
struct constraint_gearparams_t;

abstract_class IPhysics32 : public IPhysics {
	public:
		virtual int		GetActiveEnvironmentCount() = 0;
};

abstract_class IPhysicsEnvironment32 : public IPhysicsEnvironment {
	public:
#if 0
		// Create a convex soft body from vertices. Vertices are in world space!
		virtual IPhysicsSoftBody *	CreateSoftBodyFromVertices(const Vector *vertices, int numVertices, const softbodyparams_t *pParams) = 0;
		// Resolution is the amount of nodes in the rope. Higher number means less passthrough and a finer rope.
		virtual IPhysicsSoftBody *	CreateSoftBodyRope(const Vector &start, const Vector &end, int resolution, const softbodyparams_t *pParams) = 0;
		virtual IPhysicsSoftBody *	CreateSoftBodyPatch(const Vector *corners, int resx, int resy, const softbodyparams_t *pParams) = 0;
		virtual void				DestroySoftBody(IPhysicsSoftBody *pSoftBody) = 0;
#endif
		// Constraint group is not required (can be NULL)
		virtual IPhysicsConstraint *CreateGearConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_gearparams_t &gear) = 0;
		virtual IPhysicsConstraint *CreateUserConstraint(IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, IPhysicsUserConstraint *pConstraint) = 0;

		virtual void	SweepConvex(const CPhysConvex *pConvex, const Vector &vecAbsStart, const Vector &vecAbsEnd, const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace) = 0;

		virtual int		GetObjectCount() const = 0;
};

abstract_class IPhysicsObject32 : public IPhysicsObject {
	public:
		// You need to call EnableGravity(false) first so we stop using the environment's gravity.
		// To use the environment's gravity again, call EnableGravity(true)
		// (Yes I know it's confusing, nothing I can do about it without breaking backwards compatability)
		// This will allow you to override the gravity force acted upon the object.
		virtual void		SetLocalGravity(const Vector &gravityVector) = 0;
		virtual Vector		GetLocalGravity() const = 0;
		
		// Purpose: Set/Get the speeds at which any object is travelling slower than will fall asleep.
		// linear velocity is in in/s and angular velocity is in degrees/s
		// Parameters are optional in both functions.
		virtual void		SetSleepThresholds(const float *linVel, const float *angVel) = 0;
		virtual void		GetSleepThresholds(float *linVel, float *angVel) const = 0;

		// Get a modifiable version of the collision mesh we're using. If you change it at all, remember to call UpdateCollide()
		virtual CPhysCollide *	GetCollide() = 0;
		// Set the collision model of an object. You don't need to call UpdateCollide after this.
		virtual void		SetCollide(CPhysCollide *pCollide) = 0;

		// Call this if you have recently changed the collision shape we're using.
		virtual void		UpdateCollide() = 0;

		virtual IPhysicsEnvironment32 *GetEnvironment() const = 0;

		virtual IPhysicsVehicleController *GetVehicleController() const = 0;
};

// Note: If you change anything about a collision shape that an IPhysicsObject is using, call UpdateCollide on that object.

abstract_class IPhysicsCollision32 : public IPhysicsCollision {
	public:
		// Adds a convex to a collide, with an optional transform to offset the convex.
		virtual void			AddConvexToCollide(CPhysCollide *pCollide, const CPhysConvex *pConvex, const matrix3x4_t *xform = NULL) = 0;
		virtual void			RemoveConvexFromCollide(CPhysCollide *pCollide, const CPhysConvex *pConvex) = 0;

		// This will scale a collide
		virtual void			CollideSetScale(CPhysCollide *pCollide, const Vector &scale) = 0;
		virtual void			CollideGetScale(const CPhysCollide *pCollide, Vector &scale) = 0;

		// API to disable old vphysics behavior for bboxes
		// (holds all bboxes in a cache, and it'll either give you one already created or create a new one)
		// The bbox cache doesn't provide any speed boost like it does in the old vphysics
		virtual void			EnableBBoxCache(bool enable) = 0;
		virtual bool			IsBBoxCacheEnabled() = 0;

		// New collision shapes
		// NOTE: VPhysics DOES NOT keep track of these, unlike BBoxes! You must destroy them with DestroyCollide!

		virtual CPhysConvex *	CylinderToConvex(const Vector &mins, const Vector &maxs) = 0;
		virtual CPhysConvex *	ConeToConvex(const float radius, const float height) = 0;
		virtual CPhysConvex *	SphereToConvex(const float radius) = 0;
};

#endif // VPHYSICS_INTERFACEV32_H
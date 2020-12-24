#ifndef PHYSICS_KEYPARSER_H
#define PHYSICS_KEYPARSER_H
#if defined(_MSC_VER) || (defined(__GNUC__) && __GNUC__ > 3)
	#pragma once
#endif

#include <vcollide_parse.h>
#include <vphysics/vehicles.h>

#define MAX_KEYVALUE	1024

class KeyValues;

class CPhysicsKeyParser : public IVPhysicsKeyParser
{
	public:
		CPhysicsKeyParser(const char *pKeyValues);
		~CPhysicsKeyParser();

		void			NextBlock();

		const char *	GetCurrentBlockName();
		bool			Finished();
		void			ParseSolid(solid_t *pSolid, IVPhysicsKeyHandler *unknownKeyHandler);
		void			ParseFluid(fluid_t *pFluid, IVPhysicsKeyHandler *unknownKeyHandler);
		void			ParseRagdollConstraint(constraint_ragdollparams_t *pConstraint, IVPhysicsKeyHandler *unknownKeyHandler);
		void			ParseSurfaceTable(int *table, IVPhysicsKeyHandler *unknownKeyHandler);
		void			ParseCustom(void *pCustom, IVPhysicsKeyHandler *unknownKeyHandler);
		void			ParseVehicle(vehicleparams_t *pVehicle, IVPhysicsKeyHandler *unknownKeyHandler);
		void			SkipBlock() { NextBlock(); };

        //lwss add
        void		ParseCollisionRules( ragdollcollisionrules_t *pRules, IVPhysicsKeyHandler *unknownKeyHandler );
        void		ParseRagdollAnimatedFriction( ragdollanimatedfriction_t *pFriction, IVPhysicsKeyHandler *unknownKeyHandler	);
        //lwss end
		// Unexposed functions
	public:
		void			ParseVehicleAxle(vehicle_axleparams_t &axle, KeyValues *kv);
		void			ParseVehicleWheel(vehicle_wheelparams_t &wheel, KeyValues *kv);
		void			ParseVehicleSuspension(vehicle_suspensionparams_t &suspension, KeyValues *kv);
		void			ParseVehicleBody(vehicle_bodyparams_t &body, KeyValues *kv);
		void			ParseVehicleEngine(vehicle_engineparams_t &engine, KeyValues *kv);
		void			ParseVehicleEngineBoost(vehicle_engineparams_t &engine, KeyValues *kv);
		void			ParseVehicleSteering(vehicle_steeringparams_t &steering, KeyValues *kv);

	private:
		KeyValues *		m_pKeyValues;
		KeyValues *		m_pCurrentBlock;
};

#endif // PHYSICS_KEYPARSER_H

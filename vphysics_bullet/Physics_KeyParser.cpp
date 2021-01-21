#include "StdAfx.h"

#include <tier1/keyvalues.h>
#include <vphysics/constraints.h>
#include <filesystem_helpers.h>

#include "Physics_KeyParser.h"
#include "Physics_SurfaceProps.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

static void ReadVector(const char *pString, Vector &out) {
	float x, y, z;
	sscanf(pString, "%f %f %f", &x, &y, &z);
	out.x = x;
	out.y = y;
	out.z = z;
}

static void ReadVector4D(const char *pString, Vector4D &out) {
	float x, y, z, w;
	sscanf(pString, "%f %f %f %f", &x, &y, &z, &w);
	out.x = x;
	out.y = y;
	out.z = z;
	out.w = w;
}

CPhysicsKeyParser::CPhysicsKeyParser(const char *pKeyValues) {
	m_pKeyValues = new KeyValues("CPhysicsKeyParser");
	m_pKeyValues->LoadFromBuffer("(unknown file)", pKeyValues);
	m_pCurrentBlock = m_pKeyValues;
}

CPhysicsKeyParser::~CPhysicsKeyParser() {
	if (m_pKeyValues)
		m_pKeyValues->deleteThis();
}

void CPhysicsKeyParser::NextBlock() {
	if (m_pCurrentBlock)
		m_pCurrentBlock = m_pCurrentBlock->GetNextKey();
}

const char *CPhysicsKeyParser::GetCurrentBlockName() {
	if (m_pCurrentBlock) {
		return m_pCurrentBlock->GetName();
	}

	return NULL;
}

bool CPhysicsKeyParser::Finished() {
	return m_pCurrentBlock == NULL;
}

void CPhysicsKeyParser::ParseSolid(solid_t *pSolid, IVPhysicsKeyHandler *unknownKeyHandler) {
	if (unknownKeyHandler)
		unknownKeyHandler->SetDefaults(pSolid);
	else
		memset(pSolid, 0, sizeof(*pSolid));

	for (KeyValues *data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!Q_stricmp(key, "index"))
			pSolid->index = data->GetInt();
		else if (!Q_stricmp(key, "name"))
			strncpy(pSolid->name, data->GetString(), sizeof(pSolid->name));
		else if (!Q_stricmp(key, "parent"))
			strncpy(pSolid->parent, data->GetString(), sizeof(pSolid->parent));
		else if (!Q_stricmp(key, "mass"))
			pSolid->params.mass = data->GetFloat();
		else if (!Q_stricmp(key, "massCenterOverride"))
			ReadVector(data->GetString(), pSolid->massCenterOverride);
		else if (!Q_stricmp(key, "surfaceprop"))
			strncpy(pSolid->surfaceprop, data->GetString(), sizeof(pSolid->surfaceprop));
		else if (!Q_stricmp(key, "damping"))
			pSolid->params.damping = data->GetFloat();
		else if (!Q_stricmp(key, "rotdamping"))
			pSolid->params.rotdamping = data->GetFloat();
		else if (!Q_stricmp(key, "inertia"))
			pSolid->params.inertia = data->GetFloat();
		else if (!Q_stricmp(key, "volume"))
			pSolid->params.volume = data->GetFloat();
		else if (!Q_stricmp(key, "drag"))
			pSolid->params.dragCoefficient = data->GetFloat();
		//else if (!Q_stricmp(key, "rollingdrag")) // This is in vphysics.so but it doesn't seem to set any variables.
		//else if (!Q_stricmp(key, "massbias")) // TODO: What is this
		else if (unknownKeyHandler)
			unknownKeyHandler->ParseKeyValue(pSolid, key, data->GetString());
		else
			DevWarning("KeyParser ParseSolid: Unknown Key %s\n", key);
	}

	NextBlock();
}

void CPhysicsKeyParser::ParseFluid(fluid_t *pFluid, IVPhysicsKeyHandler *unknownKeyHandler) {
	if (unknownKeyHandler)
		unknownKeyHandler->SetDefaults(pFluid);
	else
		memset(pFluid, 0, sizeof(*pFluid));

	pFluid->index = -1;

	for (KeyValues *data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!Q_stricmp(key, "index"))
			pFluid->index = data->GetInt();
		else if (!Q_stricmp(key, "surfaceprop"))
			strncpy(pFluid->surfaceprop, data->GetString(), sizeof(pFluid->surfaceprop));
		else if (!Q_stricmp(key, "damping"))
			pFluid->params.damping = data->GetFloat();
		else if (!Q_stricmp(key, "contents"))
			pFluid->params.contents = data->GetInt();
		else if (!Q_stricmp(key, "surfaceplane"))
			ReadVector4D(data->GetString(), pFluid->params.surfacePlane);
		else if (!Q_stricmp(key, "currentvelocity"))
			ReadVector(data->GetString(), pFluid->params.currentVelocity);
		else if (unknownKeyHandler)
			unknownKeyHandler->ParseKeyValue(pFluid, key, data->GetString());
		else
			DevWarning("KeyParser ParseFluid: Unknown Key %s\n", key);
	}

	NextBlock();
}

void CPhysicsKeyParser::ParseRagdollConstraint(constraint_ragdollparams_t *pConstraint, IVPhysicsKeyHandler *unknownKeyHandler) {
	if (unknownKeyHandler)
		unknownKeyHandler->SetDefaults(pConstraint);
	else {
		memset(pConstraint, 0, sizeof(*pConstraint));
		pConstraint->childIndex = -1;
		pConstraint->parentIndex = -1;
	}

	for (KeyValues *data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!Q_stricmp(key, "parent"))
			pConstraint->parentIndex = data->GetInt();
		else if (!Q_stricmp(key, "child"))
			pConstraint->childIndex = data->GetInt();
		else if (!Q_stricmp(key, "xmin"))
			pConstraint->axes[0].minRotation = data->GetFloat();
		else if (!Q_stricmp(key, "xmax"))
			pConstraint->axes[0].maxRotation = data->GetFloat();
		else if (!Q_stricmp(key, "xfriction")) {
			pConstraint->axes[0].angularVelocity = 0;
			pConstraint->axes[0].torque = data->GetFloat();
		}
		else if (!Q_stricmp(key, "ymin"))
			pConstraint->axes[1].minRotation = data->GetFloat();
		else if (!Q_stricmp(key, "ymax"))
			pConstraint->axes[1].maxRotation = data->GetFloat();
		else if (!Q_stricmp(key, "yfriction")) {
			pConstraint->axes[1].angularVelocity = 0;
			pConstraint->axes[1].torque = data->GetFloat();
		}
		else if (!Q_stricmp(key, "zmin"))
			pConstraint->axes[2].minRotation = data->GetFloat();
		else if (!Q_stricmp(key, "zmax"))
			pConstraint->axes[2].maxRotation = data->GetFloat();
		else if (!Q_stricmp(key, "zfriction")) {
			pConstraint->axes[2].angularVelocity = 0;
			pConstraint->axes[2].torque = data->GetFloat();
		}
		else if (unknownKeyHandler)
			unknownKeyHandler->ParseKeyValue(pConstraint, key, data->GetString());
		else
			DevWarning("KeyParser ParseRagdollConstraint: Unknown Key %s\n", key);
	}

	NextBlock();
}

void CPhysicsKeyParser::ParseSurfaceTable(int *table, IVPhysicsKeyHandler *unknownKeyHandler) {
	for (KeyValues *data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey()) {
		if (data->GetInt() < 128)
			table[data->GetInt()] = g_SurfaceDatabase.GetSurfaceIndex(data->GetName());
	}

	NextBlock();
}

// Purpose: Recursive function to loop through all the keyvalues!
static void RecursiveKeyLoop(KeyValues *pBlock, void *pCustom, IVPhysicsKeyHandler *unknownKeyHandler) {
	for (KeyValues *pKey = pBlock; pKey; pKey = pKey->GetNextKey()) {
		const char *key = pKey->GetName();
		const char *value = pKey->GetString();
		unknownKeyHandler->ParseKeyValue(pCustom, key, value);

		// Recurse
		if (pKey->GetFirstSubKey())
			RecursiveKeyLoop(pKey->GetFirstSubKey(), pCustom, unknownKeyHandler);
	}
}

void CPhysicsKeyParser::ParseCustom(void *pCustom, IVPhysicsKeyHandler *unknownKeyHandler) {
	if (!unknownKeyHandler) return;

	unknownKeyHandler->SetDefaults(pCustom);
	RecursiveKeyLoop(m_pCurrentBlock, pCustom, unknownKeyHandler);
	NextBlock();
}

void CPhysicsKeyParser::ParseVehicle(vehicleparams_t *pVehicle, IVPhysicsKeyHandler *unknownKeyHandler) {
	if (unknownKeyHandler)
		unknownKeyHandler->SetDefaults(pVehicle);
	else
		memset(pVehicle, 0, sizeof(*pVehicle));

	for (KeyValues *data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!Q_stricmp(key, "wheelsperaxle"))
			pVehicle->wheelsPerAxle = data->GetInt();
		else if (!Q_stricmp(key, "body"))
			ParseVehicleBody(pVehicle->body, data);
		else if (!Q_stricmp(key, "engine"))
			ParseVehicleEngine(pVehicle->engine, data);
		else if (!Q_stricmp(key, "steering"))
			ParseVehicleSteering(pVehicle->steering, data);
		else if (!Q_stricmp(key, "axle") && pVehicle->axleCount < VEHICLE_MAX_AXLE_COUNT)
			ParseVehicleAxle(pVehicle->axles[pVehicle->axleCount++], data);
		else if (unknownKeyHandler)
			unknownKeyHandler->ParseKeyValue(pVehicle, key, data->GetString());
		else
			DevWarning("KeyParser ParseVehicle: Unknown Key %s\n", key);
	}

	NextBlock();
}


//lwss add
void CPhysicsKeyParser::ParseCollisionRules( ragdollcollisionrules_t *pRules, IVPhysicsKeyHandler *unknownKeyHandler )
{
    char temp[MAX_KEYVALUE];
    for( KeyValues *data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey() ) {
        const char *key = data->GetName();

        if( !Q_stricmp( key, "selfcollisions" ) )
        {
            pRules->bSelfCollisions = 0;
        }
        else if ( !Q_stricmp( key, "collisionpair" ) )
        {
            if( pRules->bSelfCollisions )
            {
                const char *token = nexttoken( temp, data->GetString(), ',' );
                int tokenValue = atoi( temp );
                nexttoken( temp, token, ',' );
                int tokenValue2 = atoi( temp );
                pRules->pCollisionSet->EnableCollisions( tokenValue, tokenValue2 );
            }
        }
        //lwss: this is never the case in this codebase
        //else
        //{
        //    if( unknownKeyHandler )
        //    {
        //        unknownKeyHandler->ParseKeyValue( pRules, key, value );
        //    }
        //}
    }
    NextBlock();
}
void CPhysicsKeyParser::ParseRagdollAnimatedFriction( ragdollanimatedfriction_t *pFriction, IVPhysicsKeyHandler *unknownKeyHandler	)
{
    char value[MAX_KEYVALUE];

    if ( unknownKeyHandler )
        unknownKeyHandler->SetDefaults( pFriction );
    else
        memset( pFriction, 0, sizeof(*pFriction) );

    for( KeyValues *data = m_pCurrentBlock->GetFirstSubKey(); data; data = data->GetNextKey() ) {
        const char *key = data->GetName();

        if( !Q_stricmp( key, "animfrictionmin" ) )
        {
            pFriction->minFriction = data->GetFloat();
        }
        else if( !Q_stricmp( key, "animfrictionmax" ) )
        {
            pFriction->maxFriction = data->GetFloat();
        }
        else if( !Q_stricmp( key, "animfrictiontimein" ) )
        {
            pFriction->timeIn = data->GetFloat();
        }
        else if( !Q_stricmp( key, "animfrictiontimeout" ) )
        {
            pFriction->timeOut = data->GetFloat();
        }
        else if( !Q_stricmp( key, "animfrictiontimehold" ) )
        {
            pFriction->timeHold = data->GetFloat();
        }
        else
        {
            if ( unknownKeyHandler )
            {
                unknownKeyHandler->ParseKeyValue( pFriction, key, value );
            }
        }
    }
    NextBlock();
}
//lwss end


void CPhysicsKeyParser::ParseVehicleAxle(vehicle_axleparams_t &axle, KeyValues *kv) {
	for (KeyValues *data = kv->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!Q_stricmp(key, "wheel"))
			ParseVehicleWheel(axle.wheels, data);
		else if (!Q_stricmp(key, "suspension"))
			ParseVehicleSuspension(axle.suspension, data);
		else if (!Q_stricmp(key, "torquefactor"))
			axle.torqueFactor = data->GetFloat();
		else if (!Q_stricmp(key, "brakefactor"))
			axle.brakeFactor = data->GetFloat();
		else
			DevWarning("KeyParser ParseVehicleAxle: Unknown Key %s\n", key);
	}
}

void CPhysicsKeyParser::ParseVehicleWheel(vehicle_wheelparams_t &wheel, KeyValues *kv) {
	for (KeyValues *data = kv->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!Q_stricmp(key, "radius"))
			wheel.radius = data->GetFloat();
		else if (!Q_stricmp(key, "mass"))
			wheel.mass = data->GetFloat();
		else if (!Q_stricmp(key, "inertia"))
			wheel.inertia = data->GetFloat();
		else if (!Q_stricmp(key, "damping"))
			wheel.damping = data->GetFloat();
		else if (!Q_stricmp(key, "rotdamping"))
			wheel.rotdamping = data->GetFloat();
		else if (!Q_stricmp(key, "frictionscale"))
			wheel.frictionScale = data->GetFloat(); // UNUSED!
		else if (!Q_stricmp(key, "material"))
			wheel.materialIndex = data->GetInt();
		else if (!Q_stricmp(key, "skidmaterial"))
			wheel.skidMaterialIndex = data->GetInt();
		else if (!Q_stricmp(key, "brakematerial"))
			wheel.brakeMaterialIndex = data->GetInt();
		else
			DevWarning("KeyParser ParseVehicleWheel: Unknown Key %s\n", key);
	}
}

void CPhysicsKeyParser::ParseVehicleSuspension(vehicle_suspensionparams_t &suspension, KeyValues *kv) {
	for (KeyValues *data = kv->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!Q_stricmp(key, "springConstant"))
			suspension.springConstant = data->GetFloat();
		else if (!Q_stricmp(key, "springDamping"))
			suspension.springDamping = data->GetFloat();
		else if (!Q_stricmp(key, "stabilizerConstant"))
			suspension.stabilizerConstant = data->GetFloat();
		else if (!Q_stricmp(key, "springDampingCompression"))
			suspension.springDampingCompression = data->GetFloat();
		else if (!Q_stricmp(key, "maxBodyForce"))
			suspension.maxBodyForce = data->GetFloat();
		else
			DevWarning("KeyParser ParseVehicleSuspension: Unknown Key %s\n", key);
	}
}

void CPhysicsKeyParser::ParseVehicleBody(vehicle_bodyparams_t &body, KeyValues *kv) {
	for (KeyValues *data = kv->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!Q_stricmp(key, "countertorquefactor"))
			body.counterTorqueFactor = data->GetFloat();
		else if (!Q_stricmp(key, "massCenterOverride"))
			ReadVector(data->GetString(), body.massCenterOverride);
		else if (!Q_stricmp(key, "massOverride"))
			body.massOverride = data->GetFloat();
		else if (!Q_stricmp(key, "addgravity"))
			body.addGravity = data->GetFloat();
		else if (!Q_stricmp(key, "maxAngularVelocity"))
			body.maxAngularVelocity = data->GetFloat();
		else if (!Q_stricmp(key, "tiltforce"))
			body.tiltForce = data->GetFloat();
		else if (!Q_stricmp(key, "tiltforceheight"))
			body.tiltForceHeight = data->GetFloat();
		else if (!Q_stricmp(key, "keepuprighttorque"))
			body.keepUprightTorque = data->GetFloat();
		else
			DevWarning("KeyParser ParseVehicleBody: Unknown Key %s\n", key);
	}
}

void CPhysicsKeyParser::ParseVehicleEngine(vehicle_engineparams_t &engine, KeyValues *kv) {
	for (KeyValues *data = kv->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!Q_stricmp(key, "horsepower"))
			engine.horsepower = data->GetFloat();
		else if (!Q_stricmp(key, "maxrpm"))
			engine.maxRPM = data->GetFloat();
		else if (!Q_stricmp(key, "maxspeed"))
			engine.maxSpeed = data->GetFloat();
		else if (!Q_stricmp(key, "maxReverseSpeed"))
			engine.maxRevSpeed = data->GetFloat();
		else if (!Q_stricmp(key, "autobrakeSpeedGain"))
			engine.autobrakeSpeedGain = data->GetFloat();
		else if (!Q_stricmp(key, "autobrakeSpeedFactor"))
			engine.autobrakeSpeedFactor = data->GetFloat();
		else if (!Q_stricmp(key, "autotransmission"))
			engine.isAutoTransmission = data->GetInt() > 0;
		else if (!Q_stricmp(key, "throttletime"))
			engine.throttleTime = data->GetFloat();
		else if (!Q_stricmp(key, "axleratio"))
			engine.axleRatio = data->GetFloat();
		else if (!Q_stricmp(key, "gear") && engine.gearCount < VEHICLE_MAX_GEAR_COUNT)
			engine.gearRatio[engine.gearCount++] = data->GetFloat();
		else if (!Q_stricmp(key, "shiftuprpm"))
			engine.shiftUpRPM = data->GetFloat();
		else if (!Q_stricmp(key, "shiftdownrpm"))
			engine.shiftDownRPM = data->GetFloat();
		else if (!Q_stricmp(key, "boost"))
			ParseVehicleEngineBoost(engine, data);
		else
			DevWarning("KeyParser ParseVehicleEngine: Unknown Key %s\n", key);
	}
}

void CPhysicsKeyParser::ParseVehicleEngineBoost(vehicle_engineparams_t &engine, KeyValues *kv) {
	for (KeyValues *data = kv->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!Q_stricmp(key, "force"))
			engine.boostForce = data->GetFloat();
		else if (!Q_stricmp(key, "duration"))
			engine.boostDuration = data->GetFloat();
		else if (!Q_stricmp(key, "delay"))
			engine.boostDelay = data->GetFloat();
		else if (!Q_stricmp(key, "torqueboost"))
			engine.torqueBoost = data->GetInt() > 0;
		else if (!Q_stricmp(key, "maxspeed"))
			engine.boostMaxSpeed = data->GetFloat();
		else
			DevWarning("KeyParser ParseVehicleEngineBoost: Unknown Key %s\n", key);
	}
}

void CPhysicsKeyParser::ParseVehicleSteering(vehicle_steeringparams_t &steering, KeyValues *kv) {
	for (KeyValues *data = kv->GetFirstSubKey(); data; data = data->GetNextKey()) {
		const char *key = data->GetName();

		if (!Q_stricmp(key, "degreesSlow"))
			steering.degreesSlow = data->GetFloat();
		else if (!Q_stricmp(key, "degreesFast"))
			steering.degreesFast = data->GetFloat();
		else if (!Q_stricmp(key, "degreesBoost"))
			steering.degreesBoost = data->GetFloat();
		else if (!Q_stricmp(key, "steeringExponent"))
			steering.steeringExponent = data->GetFloat();
		else if (!Q_stricmp(key, "slowcarspeed"))
			steering.speedSlow = data->GetFloat();
		else if (!Q_stricmp(key, "fastcarspeed"))
			steering.speedFast = data->GetFloat();
		else if (!Q_stricmp(key, "slowSteeringRate"))
			steering.steeringRateSlow = data->GetFloat();
		else if (!Q_stricmp(key, "fastSteeringRate"))
			steering.steeringRateFast = data->GetFloat();
		else if (!Q_stricmp(key, "steeringRestRateSlow"))
			steering.steeringRestRateSlow = data->GetFloat();
		else if (!Q_stricmp(key, "steeringRestRateFast"))
			steering.steeringRestRateFast = data->GetFloat();
		else if (!Q_stricmp(key, "turnThrottleReduceSlow"))
			steering.turnThrottleReduceSlow = data->GetFloat();
		else if (!Q_stricmp(key, "turnThrottleReduceFast"))
			steering.turnThrottleReduceFast = data->GetFloat();
		else if (!Q_stricmp(key, "brakeSteeringRateFactor"))
			steering.brakeSteeringRateFactor = data->GetFloat();
		else if (!Q_stricmp(key, "throttleSteeringRestRateFactor"))
			steering.throttleSteeringRestRateFactor = data->GetFloat();
		else if (!Q_stricmp(key, "boostSteeringRestRateFactor"))
			steering.boostSteeringRestRateFactor = data->GetFloat();
		else if (!Q_stricmp(key, "boostSteeringRateFactor"))
			steering.boostSteeringRateFactor = data->GetFloat();
		else if (!Q_stricmp(key, "powerSlideAccel"))
			steering.powerSlideAccel = data->GetFloat();
		else if (!Q_stricmp(key, "skidallowed"))
			steering.isSkidAllowed = data->GetInt() > 0;
		else if (!Q_stricmp(key, "dustcloud"))
			steering.dustCloud = data->GetInt() > 0;
		else
			DevWarning("KeyParser ParseVehicleSteering: Unknown Key %s\n", key);
	}
}

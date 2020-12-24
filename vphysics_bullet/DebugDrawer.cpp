#include "StdAfx.h"

#include <edict.h>

#include <LinearMath/btQuickprof.h>

#include "Physics_Environment.h"
#include "DebugDrawer.h"
#include "convert.h"

#ifdef DEBUGDRAW_RENDER_SDL
	#include <SDL.h>
	#include <SDL_opengl.h>
#endif // DEBUGDRAW_RENDER_SDL

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

#if DEBUG_DRAW

// BUG: Debug draw causes memory leak in the server

static ConVar cvar_renderoverlay("vphysics_renderoverlay", "0", FCVAR_CHEAT | FCVAR_ARCHIVE, "Render debug overlay");
static ConVar cvar_overlaywireframe("vphysics_overlay_wireframe", "0", FCVAR_CHEAT, "Render wireframe on the overlay (lags on most maps!)");
static ConVar cvar_overlaydepthtest("vphysics_overlay_nodepthtest", "0", FCVAR_CHEAT | FCVAR_ARCHIVE, "No depth test when rendering the overlay");

CDebugDrawer::CDebugDrawer(btCollisionWorld *world) : m_debugMode(0), m_overlay(NULL) {
	setDebugMode(/*DBG_DrawAabb |*/ DBG_DrawConstraintLimits | DBG_DrawConstraints | DBG_DrawContactPoints |
				DBG_DrawNormals);

#ifdef DEBUGDRAW_RENDER_SDL
	m_pDisplay = NULL;

	SDL_Init(SDL_INIT_VIDEO);
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 5);
	SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 5);
	SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 5);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
	SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	m_pDisplay = SDL_SetVideoMode(640, 480, 32, SDL_OPENGL);

	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glClearColor(0, 0, 0, 0);
	glViewport(0, 0, 640, 480);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-1, 1, -1, 1, 1, 10000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 17, -60, 0, 0, 0, 0, 1, 0);
#endif

	m_world = world;
	m_world->setDebugDrawer(this);

#ifndef BT_NO_PROFILE
	// m_pProfIterator = CProfileManager::Get_Iterator();
#endif
}

CDebugDrawer::~CDebugDrawer() {
	m_world->setDebugDrawer(NULL);
#ifndef BT_NO_PROFILE
	// CProfileManager::Release_Iterator(m_pProfIterator);
#endif

#ifdef DEBUGDRAW_RENDER_SDL
	SDL_FreeSurface(m_pDisplay);
	SDL_Quit();
#endif
}

void CDebugDrawer::drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &fromColor, const btVector3 &toColor) {
#ifdef DEBUGDRAW_RENDER_SDL
	glBegin(GL_LINES);
		glColor3f(fromColor.getX(), fromColor.getY(), fromColor.getZ());
		glVertex3d(from.getX(), from.getY(), from.getZ());
		glColor3f(toColor.getX(), toColor.getY(), toColor.getZ());
		glVertex3d(to.getX(), to.getY(), to.getZ());
	glEnd();
#else
	Vector HLFrom;
	Vector HLTo;
	ConvertPosToHL(from, HLFrom);
	ConvertPosToHL(to, HLTo);

	m_overlay->AddLineOverlay(HLFrom, HLTo, fromColor.x() * 255, fromColor.y() * 255, fromColor.z() * 255, cvar_overlaydepthtest.GetBool(), 0);
#endif
}

void CDebugDrawer::drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &color) {
	drawLine(from, to, color, color);
}

#ifdef DEBUGDRAW_RENDER_SDL
void CDebugDrawer::drawSphere(const btVector3 &p, btScalar radius, const btVector3 &color) {
	glColor4f (color.getX(), color.getY(), color.getZ(), btScalar(1.0f));
	glPushMatrix ();
	glTranslatef (p.getX(), p.getY(), p.getZ());

	int lats = 5;
	int longs = 5;

	int i, j;
	for(i = 0; i <= lats; i++) {
		btScalar lat0 = SIMD_PI * (-btScalar(0.5) + (btScalar) (i - 1) / lats);
		btScalar z0  = radius*sin(lat0);
		btScalar zr0 =  radius*cos(lat0);

		btScalar lat1 = SIMD_PI * (-btScalar(0.5) + (btScalar) i / lats);
		btScalar z1 = radius*sin(lat1);
		btScalar zr1 = radius*cos(lat1);

		glBegin(GL_QUAD_STRIP);
		for(j = 0; j <= longs; j++) {
			btScalar lng = 2 * SIMD_PI * (btScalar) (j - 1) / longs;
			btScalar x = cos(lng);
			btScalar y = sin(lng);

			glNormal3f(x * zr0, y * zr0, z0);
			glVertex3f(x * zr0, y * zr0, z0);
			glNormal3f(x * zr1, y * zr1, z1);
			glVertex3f(x * zr1, y * zr1, z1);
		}
		glEnd();
	}

	glPopMatrix();
}
#endif

void CDebugDrawer::drawBox(const btVector3 &boxMin, const btVector3 &boxMax, const btVector3 &color, btScalar alpha) {
#ifdef DEBUGDRAW_RENDER_SDL
	btVector3 halfExtent = (boxMax - boxMin) * btScalar(0.5f);
	btVector3 center = (boxMax + boxMin) * btScalar(0.5f);
	//glEnable(GL_BLEND);     // Turn blending On
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	glColor4f (color.getX(), color.getY(), color.getZ(), alpha);
	glPushMatrix ();
	glTranslatef (center.getX(), center.getY(), center.getZ());
	glScaled(2*halfExtent[0], 2*halfExtent[1], 2*halfExtent[2]);
	glPopMatrix ();
	//glDisable(GL_BLEND);
#else
	Vector HLBoxMin;
	Vector HLBoxMax;
	ConvertPosToHL(boxMin, HLBoxMin);
	ConvertPosToHL(boxMax, HLBoxMax);
#endif
}

void CDebugDrawer::drawTriangle(const btVector3 &a, const btVector3 &b, const btVector3 &c, const btVector3 &color, btScalar alpha) {
#ifdef DEBUGDRAW_RENDER_SDL
	const btVector3	n=btCross(b-a, c-a).normalized();
	glBegin(GL_TRIANGLES);		
		glColor4f(color.getX(), color.getY(), color.getZ(), alpha);
		glNormal3d(n.getX(), n.getY(), n.getZ());
		glVertex3d(a.getX(), a.getY(), a.getZ());
		glVertex3d(b.getX(), b.getY(), b.getZ());
		glVertex3d(c.getX(), c.getY(), c.getZ());
	glEnd();
#else
	Vector HLA;
	Vector HLB;
	Vector HLC;
	ConvertPosToHL(a, HLA);
	ConvertPosToHL(b, HLB);
	ConvertPosToHL(c, HLC);
	m_overlay->AddTriangleOverlay(HLA, HLB, HLC, color.x() * 255, color.y() * 255, color.z() * 255, alpha * 255, cvar_overlaydepthtest.GetBool(), 0);
#endif
}

void CDebugDrawer::setDebugMode(int debugMode) {
	m_debugMode = debugMode;
}

void CDebugDrawer::draw3dText(const btVector3 &location, const char *textString) {
#ifdef DEBUGDRAW_RENDER_SDL
	glRasterPos3f(location.x(),  location.y(),  location.z());
#else
	Vector HLLocation;
	ConvertPosToHL(location, HLLocation);

	m_overlay->AddTextOverlay(HLLocation, 0, textString);
#endif
}

void CDebugDrawer::reportErrorWarning(const char *warningString) {
	Warning(warningString);
}

void CDebugDrawer::drawContactPoint(const btVector3 &pointOnB, const btVector3 &normalOnB, btScalar distance, int lifeTime, const btVector3 &color) {
	btVector3 to = pointOnB + normalOnB * (distance + 0.2); //pointOnB + normalOnB * 1;
	const btVector3 &from = pointOnB;
#ifdef DEBUGDRAW_RENDER_SDL
	glColor4f(color.getX(), color.getY(), color.getZ(),1.f);
	glBegin(GL_LINES);
		glVertex3d(from.getX(), from.getY(), from.getZ());
		glVertex3d(to.getX(), to.getY(), to.getZ());
	glEnd();
#else
	drawLine(from, to, color);
#endif
}

void CDebugDrawer::SetDebugOverlay(IVPhysicsDebugOverlay *pOverlay) {
	m_overlay = pOverlay;
}

IVPhysicsDebugOverlay *CDebugDrawer::GetDebugOverlay() {
	return m_overlay;
}

void CDebugDrawer::DrawWorld() {
#ifdef DEBUGDRAW_RENDER_SDL
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	setDebugMode(DBG_DrawWireframe);
	m_world->debugDrawWorld();
	glFlush();
	SDL_GL_SwapBuffers();
#else
	if (cvar_renderoverlay.GetBool()) {
		// Shit fuck I don't care, it works
		cvar_overlaywireframe.GetBool() ? setDebugMode(getDebugMode() | DBG_DrawWireframe) : setDebugMode(getDebugMode() & ~(DBG_DrawWireframe));
		if (m_overlay) {
			m_world->debugDrawWorld();
		}
	}
#endif
}

#endif // DEBUG_DRAW

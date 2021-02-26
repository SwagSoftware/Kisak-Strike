//========= Copyright � 1996-2005, Valve Corporation, All rights reserved. ====
//
// Defines the cordon tool. The cordon tool defines a rectangular
// volume that acts as a visibility filter. Only objects that intersect
// the cordon are rendered in the views. When saving the MAP file while
// the cordon tool is active, only brushes that intersect the cordon
// bounds are saved. The cordon box is replaced by brushes in order to
// seal the map.
//
//=============================================================================

#ifndef CORDON3D_H
#define CORDON3D_H
#pragma once


#include "Box3D.h"
#include "ToolInterface.h"


class CChunkFile;
class CSaveInfo;
class CMapWorld;
class CMapView2D;
class CMapView3D;


enum ChunkFileResult_t;


class Cordon3D : public Box3D
{
	typedef Box3D BaseClass;

	public:

		Cordon3D(void);

		// CBaseTool implementation.
		virtual void OnActivate();
		virtual ToolID_t GetToolID(void) { return TOOL_EDITCORDON; }
		virtual void RefreshToolState();

		virtual bool OnLMouseDown2D(CMapView2D *pView, UINT nFlags, const Vector2D &vPoint);
		virtual bool OnLMouseUp2D(CMapView2D *pView, UINT nFlags, const Vector2D &vPoint);
		virtual bool OnMouseMove2D(CMapView2D *pView, UINT nFlags, const Vector2D &vPoint);
		virtual bool OnKeyDown2D(CMapView2D *pView, UINT nChar, UINT nRepCnt, UINT nFlags);
		virtual bool OnKeyDown3D(CMapView3D *pView, UINT nChar, UINT nRepCnt, UINT nFlags);

		virtual void RenderTool2D( CRender2D *pRender );
		
	private:

		void OnDelete();
		void OnEscape();

		static Vector m_vecLastMins;	// Last mins & maxs the user dragged out with this tool;
		static Vector m_vecLastMaxs;	// used to fill in the third axis when starting a new box.
};

#endif // CORDON3D_H

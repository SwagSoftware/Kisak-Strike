
/*
 *  3D Studio object reader
 */

#ifndef _3DSLOAD_H
#define _3DSLOAD_H

typedef unsigned char   byte;
typedef unsigned short  word;
typedef unsigned long   dword;
typedef float           float32;

typedef struct
{
    word p0,p1,p2;
	word flags;
	// ...

} H3dsFace;

typedef struct
{
    union { float32 x; dword ix; };
    union { float32 y; dword iy; };
    union { float32 z; dword iz; };
	// ...

} H3dsVert;

typedef struct
{
    union { float32 u; dword iu; };
    union { float32 v; dword iv; };
	// ...

} H3dsMap;

typedef struct
{
	int faces;              // number of faces in facelist
	int verts;              // number of vertices in vertlist
	int maps;               // number of mapping coordinates in maplist
	int matrix;             // 1 if transformation matrix is loaded
	H3dsFace * facelist;
	H3dsVert * vertlist;
	H3dsMap  * maplist;
	float32 TraMatrix[3*4]; // 3*3 rotation matrix, 3*1 translation matrix
	char name[16];          // object name, zero terminated

	// This vertice don't really belong here in the loader structure.
	// It is only used in the converter to hold the centre point
	// for this mesh object.
	H3dsVert centre;

} H3dsMeshObj;

typedef struct
{
	int meshobjs;           // number of meshobjects in meshobjlist
	H3dsMeshObj * meshobjlist;

    // Same comment as above. This vertice holds the centre point of
	// all objects in the scene.
	H3dsVert centre;

} H3dsScene;

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  H3dsScene * HRead3dsScene(void * ptr, int what, dword size);
 *
 *  ptr  - pointer to either a FILE structure or a memory buffer
 *  what - 0 if 3DS-data from file, 1 if 3DS-data from memory buffer
 *  size - size in bytes of 3DS data
 *
 *  Note! If you load from file, you must open the file in binary
 *        mode and determine the size of the 3DS-data (i.e. the filesize)
 *        upon calling this function. The filepointer is assumed to be
 *        at the wery start of the 3DS-data (i.e. start of file if it's a
 *        .3DS-file). The file is not closed by the function.
 *
 *  The function returns a pointer to a H3dsScene structure wich holds
 *  all data for each object. This structure can be freed with the
 *  HFree3dsScene() function.
 *
 *  In case of error the function frees all allocated memory and returns 0.
 *  The goal has been to make the functions well behaved and to not cause
 *  any crashes in an error situation.
 */

H3dsScene * HRead3dsScene(void * ptr, int what, dword size);
void HFree3dsScene(H3dsScene * scene);

#ifdef __cplusplus
}
#endif

#endif

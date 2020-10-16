
/*
 *  3D Studio object reader
 */

#define FILELOAD
#define MEMLOAD

#include <stdlib.h>
#include <string.h>
#include <setjmp.h>

#include <ivu_types.hxx>

#include "3dsimport_load.hxx"


static void  (* dread)  (void * dest, int len);
static void  (* dsetpos)(dword pos);
static dword (* dgetpos)(void);

static jmp_buf EnvState;


#ifdef FILELOAD
#include <stdio.h>

static FILE * InFile=0;

static void FileRead(void * dest, int len)
{
	if(fread(dest, len, 1, InFile) != 1) {
		//printf("Error reading file\n");
		longjmp(EnvState, 1);
	}
}

static void FileSetpos(dword pos)
{
	if(fseek(InFile, (long)pos, SEEK_SET) != 0) {
		//printf("Error moving filepointer\n");
		longjmp(EnvState, 1);
	}
}

static dword FileGetpos(void)
{
	long pos;
	if((pos=ftell(InFile)) == -1L) {
		//printf("Error getting fileposition\n");
		longjmp(EnvState, 1);
	}
	return (dword) pos;
}
#endif

#ifdef MEMLOAD
static byte * MemBuffer=0;
static dword MemBufferLen=0, MemBufferIndex=0;

static void MemRead(void * dest, int len)
{
	// Never allow any reads past the membuffer.
	if(MemBufferIndex+len > MemBufferLen) {
		//printf("Bad memread\n");
		longjmp(EnvState, 1);
	}
	memcpy(dest, &MemBuffer[MemBufferIndex], len);
	MemBufferIndex+=len;
}

static void MemSetpos(dword pos)
{
	// It should be legal to move the position one byte
	// past the membuffer. The mem must not be read though.
	if(pos > MemBufferLen) {
		//printf("Bad memsetpos\n");
		longjmp(EnvState, 1);
	}
	MemBufferIndex=pos;
}

static dword MemGetpos(void)
{
	return MemBufferIndex;
}
#endif



static H3dsScene * Scene;

H3dsMeshObj * GetMeshObj(void)
{
	void * mem;
	if((mem=p_realloc(Scene->meshobjlist,
					sizeof(H3dsMeshObj)*(Scene->meshobjs+1))) == 0) {
		//printf("Error reallocating mem\n");
		longjmp(EnvState, 1);
	}
	Scene->meshobjlist=(H3dsMeshObj *) mem;
	H3dsMeshObj * obj=&Scene->meshobjlist[Scene->meshobjs++];
	memset(obj, 0, sizeof(H3dsMeshObj));
	return obj;
}

static void * getmem(int size)
{
	void * mem;
	if((mem=p_malloc(size))==0) {
		//printf("Error allocating mem\n");
		longjmp(EnvState, 1);
	}
	return mem;
}

// Each 3DS data-chunk starts with a 6 byte header.
// The first item in the header is a 2 byte (word) id-number.
// After that follows a dword wich gives the size of
// the data-chunk including the header. The size can be used
// as an relative offset to the next chunk.

// tab 4
enum {
	CHUNK_RGBF      	= 0x0010,
	CHUNK_RGBB      	= 0x0011,
//  CHUNK_RBGB2     	= 0x0012,	// ?? NOT HLS.
	CHUNK_MAIN      	= 0x4D4D,
     CHUNK_OBJMESH      = 0x3D3D,
      CHUNK_BKGCOLOR    = 0x1200,
	  CHUNK_AMBCOLOR  	= 0x2100,
	  CHUNK_OBJBLOCK  	= 0x4000,
	   CHUNK_TRIMESH   	= 0x4100,
		CHUNK_VERTLIST  = 0x4110,
		CHUNK_FACELIST  = 0x4120,
		CHUNK_FACEMAT   = 0x4130,
		CHUNK_MAPLIST   = 0x4140,
		CHUNK_SMOOLIST  = 0x4150,
		CHUNK_TRMATRIX  = 0x4160,
	   CHUNK_LIGHT     	= 0x4600,
		CHUNK_SPOTLIGHT = 0x4610,
	   CHUNK_CAMERA    	= 0x4700,
	 CHUNK_MATERIAL  	= 0xAFFF,
	  CHUNK_MATNAME   	= 0xA000,
	  CHUNK_AMBIENT   	= 0xA010,
	  CHUNK_DIFFUSE   	= 0xA020,
	  CHUNK_SPECULAR  	= 0xA030,
	  CHUNK_TEXTURE   	= 0xA200,
	  CHUNK_BUMPMAP   	= 0xA230,
	  CHUNK_MAPFILE   	= 0xA300,
	 CHUNK_KEYFRAMER 	= 0xB000,
	  CHUNK_FRAMES      = 0xB008
};


static void ReadVertList(dword, H3dsMeshObj * meshobj)
{
	word nv;
	dread(&nv, sizeof(nv));
	meshobj->verts=nv;
	int k=nv;
	meshobj->vertlist=(H3dsVert *) getmem(sizeof(H3dsVert)*k);
	for(int n=0; n<k; n++) {
		dread(&meshobj->vertlist[n], sizeof(float32)*3);
	}
}

static void ReadFaceList(dword, H3dsMeshObj * meshobj)
{
	word nv;
	dread(&nv, sizeof(nv));
	meshobj->faces=nv;
	int k=nv;
	meshobj->facelist=(H3dsFace *) getmem(sizeof(H3dsFace)*k);
	for(int n=0; n<k; n++) {
		dread(&meshobj->facelist[n], sizeof(word)*4);
	}
}

static void ReadMapList(dword, H3dsMeshObj * meshobj)
{
	word nv;
	dread(&nv, sizeof(nv));
	meshobj->maps=nv;
	int k=nv;
	meshobj->maplist=(H3dsMap *) getmem(sizeof(H3dsMap)*k);
	for(int n=0; n<k; n++) {
		dread(&meshobj->maplist[n], sizeof(float32)*2);
	}
}

static void ReadTraMatrix(dword, H3dsMeshObj * meshobj)
{
	dread(&meshobj->TraMatrix, sizeof(float32)*12);
	meshobj->matrix=1;
}

static void ReadTriMeshBlocks(dword p, char * name)
{
	word id;
	dword len, pc;
	H3dsMeshObj * meshobj=GetMeshObj();
	strcpy(meshobj->name, name);
	while((pc=dgetpos()) < p)
	{
		dread(&id, sizeof(id));
		dread(&len, sizeof(len));
		switch((int)id)
		{
			case CHUNK_VERTLIST: ReadVertList (pc+len, meshobj); break;
			case CHUNK_FACELIST: ReadFaceList (pc+len, meshobj); break;
			case CHUNK_MAPLIST:  ReadMapList  (pc+len, meshobj); break;
			case CHUNK_TRMATRIX: ReadTraMatrix(pc+len, meshobj); break;

			//case CHUNK_FACEMAT:
			//case CHUNK_SMOOLIST:
			default: dsetpos(pc+len);
		}
	}
}

static void ReadObjBlocks(dword p)
{
	word id;
	dword len, pc;
	char name[16];

	// The object name is the first item
	int n=0;
	do {
		dread(&name[n++], 1);
	} while(name[n-1]!='\0' && n < int(sizeof(name)));
	name[n-1]='\0';

	while((pc=dgetpos()) < p)
	{
		dread(&id, sizeof(id));
		dread(&len, sizeof(len));
		switch((int)id)
		{
			case CHUNK_TRIMESH:	ReadTriMeshBlocks(pc+len, name); break;

			//case CHUNK_LIGHT:
			//case CHUNK_CAMERA:
			default: dsetpos(pc+len);
		}
	}
}

static void ReadObjMeshBlocks(dword p)
{
	word id;
	dword len, pc;

	while((pc=dgetpos()) < p)
	{
		dread(&id, sizeof(id));
		dread(&len, sizeof(len));
		switch((int)id)
		{
			case CHUNK_OBJBLOCK: ReadObjBlocks(pc+len); break;

			//case CHUNK_AMBCOLOR:
			//case CHUNK_BKGCOLOR:
			default: dsetpos(pc+len);
		}
	}
}

static void ReadMainBlocks(dword p)
{
	word id;
	dword len, pc;

	while((pc=dgetpos()) < p)
	{
		dread(&id, sizeof(id));
		dread(&len, sizeof(len));
		switch((int)id)
		{
			case CHUNK_OBJMESH: ReadObjMeshBlocks(pc+len); break;

			//case CHUNK_MATERIAL:
			//case CHUNK_KEYFRAMER:
			default: dsetpos(pc+len);
		}
	}
}

void HFree3dsScene(H3dsScene * scene)
{
	if(scene) {
		if(scene->meshobjlist) {
			for(int n=scene->meshobjs-1; n>=0; n--) {
				H3dsMeshObj * mobj = &scene->meshobjlist[n];
				if(mobj->maplist)  P_FREE(mobj->maplist);
				if(mobj->facelist) P_FREE(mobj->facelist);
				if(mobj->vertlist) P_FREE(mobj->vertlist);
			}
			P_FREE(scene->meshobjlist);
		}
		P_FREE(scene);
	}
}

H3dsScene * HRead3dsScene(void * ptr, int what, dword size)
{
	if(ptr==0)
		return 0;

	#ifdef FILELOAD
	if(what==0) {
		// Load from file
		InFile=(FILE *) ptr;
		dread=FileRead;
		dsetpos=FileSetpos;
		dgetpos=FileGetpos;
	}
	else
	#endif
	#ifdef MEMLOAD
	if(what==1) {
		// Read from mem
		MemBufferIndex=0;
		MemBufferLen=size;
		MemBuffer=(byte *) ptr;
		dread=MemRead;
		dsetpos=MemSetpos;
		dgetpos=MemGetpos;
	}
	else
	#endif
	{
		return 0;
	}

	if((Scene=(H3dsScene *) p_malloc(sizeof(H3dsScene)))==0)
		return 0;
	memset(Scene, 0, sizeof(H3dsScene));

	int retval = setjmp(EnvState);

	if(retval==0) {
		// Return address set, start loading 3DS data.
		word id;
		dword len, pc;
		pc=dgetpos();
		dread(&id, sizeof(id));
		dread(&len, sizeof(len));
		if((int)id!=CHUNK_MAIN) {
			HFree3dsScene(Scene);
			//printf("Not 3ds data\n");
			return 0;
		}
		ReadMainBlocks(pc+len);
	}
	else {
        // There was an error, free the allocated mem and return NULL.
		HFree3dsScene(Scene);
		return 0;
	}

	return Scene;
}

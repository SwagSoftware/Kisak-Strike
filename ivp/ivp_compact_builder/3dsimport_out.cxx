
/*
 *  Object output
 */


/*

   Raw object structure 


   +---------------+
   |    header     |  sizeof(header)   
   +---------------+
   |               |
   |   vertices    |  verts*3*sizeof(word) if no mapping
   |               |  verts*5*sizeof(word) if mapping
   +---------------+
   |               |
   | sub objects   |  if only one object: nothing here
   | center points |  else: objects*3*sizeof(word)
   |               |
   +---------------+
   | main object   |  3*sizeof(word)
   | center point  |  
   +---------------+
   |               |  each object starts with an objheader sizeof(objheader)
   |   subobject   |  then follows all faces
   |   faces       |  objheader.faces*3*sizeof(word)
   |               |
   +---------------+
   |               |
   | next subobject|  ....
   |               |
   +---------------+

*/

typedef struct
{
    int id;             // 0x49524d24  '$MRI'
    int flag;           // Type of data, bitfield
    int verts;          // Number of vertices (not including centre points)
    int objects;        // Number of objects
	int vtxptr;         // Offset to verices (from start of header)
	int centreptr;		// Offset to centre points (from start of header)
	int objptr;         // Offset to objects (from start of header)
    int reserved;

} header;

typedef struct
{
	short faces;        // Number of faces in object
	char  color;        // Object color
	char  reserved;

} objheader;

// ****
#ifndef GEKKO
// ****
void OutpHead(FILE * o, H3dsScene * scene, int verts, char * name, int f)
{
    fprintf(o,  "extern short %s____i[];\n"
                "extern short %s___ii[];\n"
                "extern short %s__iii[];\n"
                "extern \"C\" int %s[];\n"
                "int %s[8]={\n"
				" 0x49524d24,\n"
				" 0x%x,\n"
				" %d,\n"
				" %d,\n"
				" (int)%s____i-(int)%s,\n"
				" (int)%s___ii-(int)%s,\n"
                " (int)%s__iii-(int)%s,\n"
                " 0\n"
                "};\n",
				name,
				name,
				name,
				name,
				name,
				f,
				verts,
				scene->meshobjs,
				name,name,
				name,name,
				name,name);
}

void OutpHeadAsm(FILE * o, H3dsScene * scene, int verts, char * name, int f)
{
    fprintf(o,  "public %s_\n"
                "%s_ label dword\n"
                " dd 49524d24h\n"
                " dd 0%xh\n"
                " dd %d\n"
                " dd %d\n"
                " dd %s____i-%s_\n"
                " dd %s___ii-%s_\n"
                " dd %s__iii-%s_\n"
                " dd 0\n",
                name,
				name,
				f,
				verts,
				scene->meshobjs,
				name,name,
				name,name,
				name,name);
}

void OutpHeadBin(FILE * o, H3dsScene * scene, int verts, int f)
{
	header h;
	int centres = (scene->meshobjs > 1) ? scene->meshobjs+1 : 1;
    int vrtsize = (f & 1) ? sizeof(word)*5 : sizeof(word)*3;
    h.id        = 0x49524d24;
    h.flag      = f;
    h.verts     = verts;
    h.objects   = scene->meshobjs;
    h.vtxptr    = sizeof(h);
	h.centreptr = sizeof(h)+vrtsize*verts;
    h.objptr    = sizeof(h)+vrtsize*verts+centres*sizeof(word)*3;
    h.reserved  = 0;
    fwrite(&h, 1, sizeof(h), o);
}

void OutpVerts(FILE * o, H3dsMapVert * vrtmap, int verts,
               char * name, int /*f*/)
{
	fprintf(o, "short %s____i[]={\n", name);
	for(int n=0; n<verts; n++) {
		H3dsMapVert * vm = &vrtmap[n];
		fprintf(o, "%f,%f,%f", vm->x,vm->y,vm->z);
//		if(f & 1)
//            fprintf(o, ",0x%04x,0x%04x,\n", (int)vm->iu & 0xffff,
//                                            (int)vm->iv & 0xffff);
//		else
			fprintf(o, ",\n");
	}
	fprintf(o, "};\n");
}

void OutpVertsAsm(FILE * o, H3dsMapVert * vrtmap, int verts,
                  char * name, int f)
{
    fprintf(o, "%s____i label word\n", name);
	for(int n=0; n<verts; n++) {
		H3dsMapVert * vm = &vrtmap[n];
        fprintf(o, " dw %6d,%6d,%6d", (int)vm->ix,(int)vm->iy,(int)vm->iz);
		if(f & 1)
            fprintf(o, ",0%04xh,0%04xh\n", (int)vm->iu & 0xffff,
                                           (int)vm->iv & 0xffff);
		else
            fprintf(o, "\n");
	}
}

void OutpVertsBin(FILE * o, H3dsMapVert * vrtmap, int verts, int f)
{
	word buf[5];
	for(int n=0; n<verts; n++) {
		H3dsMapVert * vm = &vrtmap[n];
        buf[0] = vm->ix & 0xffff;
        buf[1] = vm->iy & 0xffff;
        buf[2] = vm->iz & 0xffff;
        buf[3] = vm->iu & 0xffff;
        buf[4] = vm->iv & 0xffff;
		if(f & 1)
            fwrite(buf, 1, sizeof(word)*5, o);
		else
			fwrite(buf, 1, sizeof(word)*3, o);
	}
}

void OutpCentres(FILE * o, H3dsScene * scene, char * name)
{
	fprintf(o, "short %s___ii[]={\n", name);
	for(int n=0; n<scene->meshobjs; n++) {
		H3dsMeshObj * mo = &scene->meshobjlist[n];
		fprintf(o, "%6d,%6d,%6d,\n",
                (int) mo->centre.ix,
                (int) mo->centre.iy,
                (int) mo->centre.iz);
	}
	if(scene->meshobjs > 1)
		fprintf(o, "%6d,%6d,%6d\n",
                (int) scene->centre.ix,
                (int) scene->centre.iy,
                (int) scene->centre.iz);
	fprintf(o, "};\n");
}

void OutpCentresAsm(FILE * o, H3dsScene * scene, char * name)
{
    fprintf(o, "%s___ii label word\n", name);
	for(int n=0; n<scene->meshobjs; n++) {
		H3dsMeshObj * mo = &scene->meshobjlist[n];
        fprintf(o, " dw %6d,%6d,%6d\n",
                (int) mo->centre.ix,
                (int) mo->centre.iy,
                (int) mo->centre.iz);
	}
	if(scene->meshobjs > 1)
        fprintf(o, "dw %6d,%6d,%6d\n",
                (int) scene->centre.ix,
                (int) scene->centre.iy,
                (int) scene->centre.iz);
}

void OutpCentresBin(FILE * o, H3dsScene * scene)
{
    word buf[3];
	for(int n=0; n<scene->meshobjs; n++) {
		H3dsMeshObj * mo = &scene->meshobjlist[n];
        buf[0] = (word) mo->centre.ix;
        buf[1] = (word) mo->centre.iy;
        buf[2] = (word) mo->centre.iz;
        fwrite(buf, 1, sizeof(word)*3, o); 
    }
    if(scene->meshobjs > 1) {
        buf[0] = (word) scene->centre.ix;
        buf[1] = (word) scene->centre.iy;
        buf[2] = (word) scene->centre.iz;
        fwrite(buf, 1, sizeof(word)*3, o); 
    }        
}

void OutpObjects(FILE * o, H3dsScene * scene, char * name)
{
    fprintf(o, "short %s__iii[]={\n", name);
    for(int n=0; n<scene->meshobjs; n++) {
		H3dsMeshObj * mo = &scene->meshobjlist[n];
        fprintf(o, "// %s\n", mo->name);
        fprintf(o, "%6d,\n%d + 256 * %d,\n", (int)mo->faces, 0, 0);
        for(int i=0; i<mo->faces; i++) {
            H3dsFace * fa = &mo->facelist[i];
            fprintf(o, "%6d,%6d,%6d,\n",
                       (int)fa->p0,
                       (int)fa->p1,
                       (int)fa->p2);
        }
    }
    fprintf(o, "};\n");
}

void OutpObjectsAsm(FILE * o, H3dsScene * scene, char * name)
{
    fprintf(o, "%s__iii label word\n", name);
    for(int n=0; n<scene->meshobjs; n++) {
		H3dsMeshObj * mo = &scene->meshobjlist[n];
        fprintf(o, "; %s\n", mo->name);
        fprintf(o, "%6d,\n%d + 256 * %d,\n", (int)mo->faces, 0, 0);
        for(int i=0; i<mo->faces; i++) {
            H3dsFace * fa = &mo->facelist[i];
            fprintf(o, " dw %6d,%6d,%6d\n",
                       (int)fa->p0,
                       (int)fa->p1,
                       (int)fa->p2);
        }
    }
}

void OutpObjectsBin(FILE * o, H3dsScene * scene)
{
    word buf[3];
    for(int n=0; n<scene->meshobjs; n++) {
		H3dsMeshObj * mo = &scene->meshobjlist[n];
        buf[0] = (word) mo->faces;
        buf[1] = (word) 0;
        fwrite(buf, 1, sizeof(word)*2, o); 
        for(int i=0; i<mo->faces; i++) {
            H3dsFace * fa = &mo->facelist[i];
            buf[0] = (word) fa->p0;
            buf[1] = (word) fa->p1;
            buf[2] = (word) fa->p2;
            fwrite(buf, 1, sizeof(word)*3, o); 
        }
    }
}

// ****
#endif // GEKKO
// ****

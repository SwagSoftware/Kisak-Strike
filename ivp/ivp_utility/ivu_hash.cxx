// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <string.h>
#include <ivu_hash.hxx>
#include <ivu_string_hash.hxx>

class IVP_Hash_Elem {
public:
    IVP_Hash_Elem *next;
    void *value;
    char key[1];
};

int IVP_Hash::hash_index(const char *key)const {
	unsigned int c;		
	unsigned int index = 0xffffffffL;
	int i;
	for (i=key_size-1;i>=0;i--){
	    c = *((unsigned char *)(key++));
	    index = IVP_Hash_crctab[((int) index ^ c) & 0xff] ^ (index >> 8);
	}
	index = index % size;
	return index;
};

unsigned int IVP_Hash_crctab[] = {
 0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL, 0x076dc419L,
 0x706af48fL, 0xe963a535L, 0x9e6495a3L, 0x0edb8832L, 0x79dcb8a4L,
 0xe0d5e91eL, 0x97d2d988L, 0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L,
 0x90bf1d91L, 0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
 0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L, 0x136c9856L,
 0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL, 0x14015c4fL, 0x63066cd9L,
 0xfa0f3d63L, 0x8d080df5L, 0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L,
 0xa2677172L, 0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
 0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L, 0x32d86ce3L,
 0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L, 0x26d930acL, 0x51de003aL,
 0xc8d75180L, 0xbfd06116L, 0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L,
 0xb8bda50fL, 0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
 0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL, 0x76dc4190L,
 0x01db7106L, 0x98d220bcL, 0xefd5102aL, 0x71b18589L, 0x06b6b51fL,
 0x9fbfe4a5L, 0xe8b8d433L, 0x7807c9a2L, 0x0f00f934L, 0x9609a88eL,
 0xe10e9818L, 0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
 0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL, 0x6c0695edL,
 0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L, 0x65b0d9c6L, 0x12b7e950L,
 0x8bbeb8eaL, 0xfcb9887cL, 0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L,
 0xfbd44c65L, 0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
 0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL, 0x4369e96aL,
 0x346ed9fcL, 0xad678846L, 0xda60b8d0L, 0x44042d73L, 0x33031de5L,
 0xaa0a4c5fL, 0xdd0d7cc9L, 0x5005713cL, 0x270241aaL, 0xbe0b1010L,
 0xc90c2086L, 0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
 0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L, 0x59b33d17L,
 0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL, 0xedb88320L, 0x9abfb3b6L,
 0x03b6e20cL, 0x74b1d29aL, 0xead54739L, 0x9dd277afL, 0x04db2615L,
 0x73dc1683L, 0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
 0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L, 0xf00f9344L,
 0x8708a3d2L, 0x1e01f268L, 0x6906c2feL, 0xf762575dL, 0x806567cbL,
 0x196c3671L, 0x6e6b06e7L, 0xfed41b76L, 0x89d32be0L, 0x10da7a5aL,
 0x67dd4accL, 0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
 0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L, 0xd1bb67f1L,
 0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL, 0xd80d2bdaL, 0xaf0a1b4cL,
 0x36034af6L, 0x41047a60L, 0xdf60efc3L, 0xa867df55L, 0x316e8eefL,
 0x4669be79L, 0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
 0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL, 0xc5ba3bbeL,
 0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L, 0xc2d7ffa7L, 0xb5d0cf31L,
 0x2cd99e8bL, 0x5bdeae1dL, 0x9b64c2b0L, 0xec63f226L, 0x756aa39cL,
 0x026d930aL, 0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
 0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L, 0x92d28e9bL,
 0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L, 0x86d3d2d4L, 0xf1d4e242L,
 0x68ddb3f8L, 0x1fda836eL, 0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L,
 0x18b74777L, 0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
 0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L, 0xa00ae278L,
 0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L, 0xa7672661L, 0xd06016f7L,
 0x4969474dL, 0x3e6e77dbL, 0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L,
 0x37d83bf0L, 0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
 0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L, 0xbad03605L,
 0xcdd70693L, 0x54de5729L, 0x23d967bfL, 0xb3667a2eL, 0xc4614ab8L,
 0x5d681b02L, 0x2a6f2b94L, 0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL,
 0x2d02ef8dL, 0x8350e449L, 0xe2438426L, 0xe5338a1bL, 0xc460be49L,
 0xe77abe1aL, 0x8b40b91bL, 0xde7da649L, 0xec719f49L, 0x9a2af45cL,
 0x862af447L, 0x8b56bb08L, 0xc770a219L, 0xd2339b59L, 0x852af447L,
 0x8b47a500L, 0xd833b90cL, 0xd367ed1eL, 0xca60ed0cL, 0xc570bf10L,
 0xdb67a80dL, 0x8b64a41dL, 0xc3338826L, 0xf933fd11L, 0xea51fc5aL,
 0xe857fb50L, 0x852da8a9L, 0x5bc46db8L, 0x304ef5ceL, 0x19beb02bL,
 0xd31fabc6L, 0x51c967dbL, 0x8a6559acL, 0x5caf5e87L, 0x50eb6284L,
 0x7a437207L, 0x05ebdb0bL, 0x0d806876L, 0x0b1f02f2L, 0x4d698260L,
 0x5bf00380L, 0x0a7115f7L, 0xf09b4147L, 0x725e51e0L, 0xeda02684L,
 0xe381c380L, 0x35ce48efL, 0xf3c74223L, 0x124df932L, 0x6a1d3474L,
 0x3d948118L, 0x7ffb3775L, 0x36f5c78cL, 0xd20b0270L, 0x5e455226L,
 0x873c578fL, 0x25dd8f39L, 0x772ea31dL, 0x6fa928f3L, 0x0ff37b2aL,
 0x63d3e42aL, 0x72ea90b9L, 0xfaefed17L, 0x86a9a8d2L, 0x21bbc001L,
 0x4d6810c5L, 0x9cd08ab5L, 0x0eeeed06L, 0x14597ff0L, 0xee02a7ffL,
 0xa498db55L, 0xec6f35bbL, 0x8d15f664L, 0x290a74d3L, 0x76115aeaL,
 0xaaa43ea9L, 0xa9b4962eL, 0x13d513aeL, 0xf9e6c9f3L, 0xd9eea0f0L,
 0xaebb6beeL, 0x69728ec9L, 0xcfc9e80cL, 0x390aebfcL, 0x6b911013L,
 0xe2ae8ff5L, 0x1854fa4eL, 0xeeb70d22L, 0xe57b1e7bL, 0x62047f46L,
 0x5acb59ecL, 0x5aa8d6daL, 0x95ebe3eeL, 0x1d62e802L, 0x134f5f97L,
 0xa9408739L, 0x100dafefL, 0x9e05784eL, 0x45586188L, 0xacdb5b2fL,
 0x6ad60310L, 0x9ce02986L, 0xfb6b223bL, 0x60296ee1L, 0xf314dde7L,
 0xb5d57b64L, 0xb9772273L, 0x8694ac86L, 0xe623a7ecL, 0x8f5cbb1cL,
 0x637505fdL, 0x0b7ed390L, 0xdf3c45fbL, 0x5e536d60L, 0x0f82e9bbL,
 0xb2960f0eL, 0x1e3d21b3L, 0xd5709c2eL, 0x62091602L, 0x2ce9a895L,
 0x81fd5d48L, 0x8422e6baL, 0x695c6bb5L, 0x7f7988e2L, 0x3321aa7eL,
 0x3dab9308L, 0x79280132L, 0x9c209408L, 0x89fa4e38L, 0xb0bdebb7L,
 0x56de71baL, 0x9211adb8L, 0x9ed424a5L, 0x2a5ae23cL, 0x2eef7dd5L,
 0x9b18d23aL, 0x5fb16878L, 0xf9d82239L, 0x31bee8baL, 0x1b966b8cL,
 0xe5837eb9L, 0x71f1eb08L, 0x74c4d378L, 0x492da9beL, 0x4a18255dL,
 0x7cca491bL, 0x2cb98b51L, 0x9afed2e9L, 0xd4c09c33L, 0x1d6476feL,
 0xe4c9655aL, 0xb21dc693L, 0xb2e7fd99L, 0x65a7a93eL, 0xcbb85df8L,
 0x32a3de70L, 0x2f77be52L, 0x77231e45L, 0xa06ead4cL, 0x207fecccL,
 0xca425a58L, 0xace8450eL, 0x0309342bL, 0x611e9626L, 0xd9dc4626L,
 0xab4cb165L, 0x77cfff00L, 0xe240ddc3L, 0x0fbb11c5L, 0x0f76d827L,
 0x2b1c5d22L, 0xb6f856e6L, 0x173413d4L, 0x2c48040cL, 0x92e802ebL,
 0xfbcce39cL, 0x23186f47L, 0xdaabb198L, 0x631df38cL, 0x51cabbdeL,
 0xaa46b8e0L, 0x1a532382L, 0x96ada3e7L, 0x348e4940L, 0x033b9a72L,
 0x4bc7c0c0L, 0x49585db4L, 0x8e145b4eL, 0xf61e7d79L, 0xe6b336e2L,
 0xcb78d769L, 0xa28de4dbL, 0x6c9200c1L, 0xfcc99e53L, 0x8cf0d424L,
 0x40a3b820L, 0x16359465L, 0x6c455a41L, 0x6aa2e4a2L, 0xfa40b370L,
 0x7f5e0f45L, 0x8204aa39L, 0x5192508fL, 0x9907fc99L, 0x9098316aL,
 0x4a86e1cdL, 0xe346bcf3L, 0x03fc7917L, 0x9664670fL, 0x24007ae3L,
 0x3f45bfaeL, 0xc1b774ccL, 0x9ba5657aL, 0xe6c675fcL, 0x8039fdbdL,
 0xb1f79e31L, 0xf6ee7285L, 0xb964b5cfL, 0x66f8a250L, 0x3b3a13a9L,
 0x4b9b1947L, 0xf59e0ee0L, 0x1ce792b8L, 0x71a1e650L, 0x1bf9ebabL,
 0xc98ca758L, 0x20c9a775L, 0xaf59108dL, 0x79a1d95bL, 0xebcb719eL,
 0x650f07c0L, 0x65e62fe9L, 0x4a19892cL, 0x01bdbb5fL, 0x21f29ee7L,
 0x8622b969L, 0x489afe4cL, 0xc9fcd058L, 0xa7fb16b2L, 0x75ac91a9L,
 0x62b3ca1cL, 0xfb6e66b3L, 0xcbeda7e4L, 0x9acc437fL, 0xc2d2f6a5L,
 0x194a30d6L, 0xe9c9564bL, 0x31e42e0cL, 0x32f91783L, 0xc29508a3L,
 0x2d2295d1L, 0xbaf50313L, 0x02188a35L, 0xce55d1c1L, 0x8aab06d5L,
 0x1aa3ef22L, 0x30effb6fL
};


IVP_Hash::IVP_Hash(int sizei, int key_sizei,void *not_found_valuei){
    size = sizei;
    key_size = key_sizei;
    not_found_value = not_found_valuei;
    elems = (IVP_Hash_Elem **)p_calloc(sizeof(void *),size);
}

IVP_Hash::~IVP_Hash(){
    int i;
    for (i=0;i<size;i++){
	if (!elems[i]) continue;
	IVP_Hash_Elem *next, *el;
	for (el = elems[i]; el; el = next){
	    next = el->next;
	    P_FREE(el);
	}
    }
    P_FREE(elems);
}

void *IVP_Hash::find(const char *key)const{
    int i = hash_index(key);
    IVP_ASSERT(i>=0);
    IVP_Hash_Elem *el;
    for (el = elems[i];el;el=el->next){
	if ( !memcmp(el->key,key,key_size))break;
    }
    if(el){
	return el->value;
    }else{
	return not_found_value;
    }
    
}

void IVP_Hash::add(const char *key, void *val){
    int i = hash_index(key);

//    printf("hash index: '%d', key: %lx key_string: '%s'\n", i, key, key);
    
    IVP_Hash_Elem *el = (IVP_Hash_Elem *)p_malloc(sizeof(IVP_Hash_Elem) + key_size); //-1
    memcpy(el->key,key,key_size);
    el->next = elems[i];
    elems[i] = el;
    el->value = val;
}


void IVP_Hash::remove(const char *key){
    int i = hash_index(key);
    IVP_Hash_Elem *el,*last_el;
    last_el = 0;
    for (el = elems[i];el;el=el->next){
	if ( !memcmp(el->key,key,key_size)){
	    if (last_el){
		last_el->next = el->next;
	    }else{
		elems[i] = el->next;
	    }
	    el->next = 0;
	    P_FREE(el);
	    return;
	}
	last_el = el;
    }
}

int IVP_U_String_Hash::hash_index(const char *key)const {
	unsigned int c;		
	unsigned int index = 0xffffffffL;
	int i;
	int key_size = strlen((char *)key);
	for (i=key_size-1;i>=0;i--){
	    c = *((unsigned char *)(key++));
	    index = IVP_Hash_crctab[((int) index ^ c) & 0xff] ^ (index >> 8);
	}
	index = index % size;
	return index;
};

IVP_U_String_Hash::IVP_U_String_Hash(int sizei, void *not_found_valuei){
    size = sizei;
    not_found_value = not_found_valuei;
    elems = (IVP_Hash_Elem **)p_calloc(sizeof(void *),size);
}

IVP_U_String_Hash::~IVP_U_String_Hash(){
    int i;
    for (i=0;i<size;i++){
	if (!elems[i]) continue;
	IVP_Hash_Elem *next, *el;
	for (el = elems[i]; el; el = next){
	    next = el->next;
	    P_FREE(el);
	}
    }
    P_FREE(elems);
}

void *IVP_U_String_Hash::find(const char *key)const{
    int i = hash_index(key);
    IVP_ASSERT(i>=0);
    IVP_Hash_Elem *el;
    for (el = elems[i];el;el=el->next){
	if ( !strcmp(&el->key[0],key))break;
    }
    if(el){
	return el->value;
    }else{
	return not_found_value;
    }
    
}

void IVP_U_String_Hash::add(const char *key, void *val){
    int i = hash_index(key);
    int keysize = strlen(key);
    IVP_Hash_Elem *el = (IVP_Hash_Elem *)p_malloc(sizeof(IVP_Hash_Elem) + keysize);
    memcpy(el->key,key,keysize+1);
    el->next = elems[i];
    elems[i] = el;
    el->value = val;
}


void IVP_U_String_Hash::remove(const char *key){
    int i = hash_index(key);
    IVP_Hash_Elem *el,*last_el;
    last_el = 0;
    for (el = elems[i];el;el=el->next){
	if ( !strcmp(&el->key[0],key)){
	    if (last_el){
		last_el->next = el->next;
	    }else{
		elems[i] = el->next;
	    }
	    el->next = 0;
	    P_FREE(el);
	    return;
	}
	last_el = el;
    }
}



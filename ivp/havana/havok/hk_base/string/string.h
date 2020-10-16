#ifndef HK_BASE_STRING_H
#define HK_BASE_STRING_H

class hk_String {
public:
	static int strcmp( const char *, const char * );
	static void strcpy(char *, const char *);
	static void memcpy(void *dest, const void *source, int size);

};
typedef hk_String hkString;

#endif /* HK_BASE_STRING_H */


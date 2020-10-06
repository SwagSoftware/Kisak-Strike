
#include <hk_base/base.h>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>

#ifdef WIN32
#	ifndef WIN32_LEAN_AND_MEAN
#		define	WIN32_LEAN_AND_MEAN
#	endif
#	ifdef _XBOX
#		include <xtl.h>
#	else
#		include <windows.h>
#	endif
#endif

hk_Console *hk_Console::m_console = HK_NULL;
hk_Console hk_Console::m_default_console_buffer;

hk_Console *hk_Console::get_instance()
{
	if ( !m_console )
	{
		m_console = &m_default_console_buffer;
	}
	return m_console;
}
#define MAX_ERROR_BUFFER_LEN 2048

void hk_Console::printf( const char *fmt, ...)
{
	va_list vlist;
	va_start(vlist, fmt);
	//XXX fixme
	// scan format list for havok vector matrix tokens %V %M
	vprintf(fmt, vlist);
	va_end(vlist);
#ifdef WIN32
    char buffer[MAX_ERROR_BUFFER_LEN];
    va_start(vlist,fmt);	
    vsprintf(buffer, fmt,vlist);
    va_end(vlist);
    OutputDebugString(buffer);
#endif
}

void hk_Console::flush()
{
#ifndef WIN32
#ifndef HK_PS2
	fflush(stdout);
#endif
#endif
}


void hk_Console::exit( int code )
{
	::exit(code);
}

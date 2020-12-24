/*
Copyright (c) 2013 Dr.Chat / Erwin Coumans  http://bullet.googlecode.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <stdio.h>
#include <stdarg.h>

#include "btDebug.h"

#if defined(__CELLOS_LV2__) && defined(__SPU__)
#include <spu_printf.h>
#define printf spu_printf
#endif

void btDefDbgMsg(const char *str)
{
	printf("%s", str);
}

btDbgMsgFn g_pDbgMsg = btDefDbgMsg;

void btDefDbgWarning(const char *str)
{
	printf("%s", str);
}

btDbgMsgFn g_pDbgWarn = btDefDbgWarning;

void btSetDbgMsgFn(btDbgMsgFn fn)
{
	fn ? g_pDbgMsg = fn : g_pDbgMsg = btDefDbgMsg;
}

void btSetDbgWarnFn(btDbgMsgFn fn)
{
	fn ? g_pDbgWarn = fn : g_pDbgWarn = btDefDbgWarning;
}

void btDbgMsgInternal(const char *fmt, ...)
{
	char buff[2048];

	va_list va;
	va_start(va, fmt);
	vsnprintf(buff, 2048, fmt, va);
	va_end(va);

	g_pDbgMsg(buff);
}

void btDbgWarningInternal(const char *fmt, ...)
{
	char buff[2048];

	va_list va;
	va_start(va, fmt);
	vsnprintf(buff, 2048, fmt, va);
	va_end(va);

	g_pDbgWarn(buff);
}
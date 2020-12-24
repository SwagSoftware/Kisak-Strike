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

#ifndef BT_DEBUG_H
#define BT_DEBUG_H

// TODO: Do we want versions of these with source information?
// Can't really do this due to limitations of compiler macros (vararg support for macros changes per compiler)

void btDbgMsgInternal(const char *fmt, ...);
void btDbgWarningInternal(const char *fmt, ...);

#define btDbgMsg btDbgMsgInternal
#define btDbgWarning btDbgWarningInternal

typedef void (*btDbgMsgFn)(const char *str);

// Sets a user function for bullet debug messages
// Call with NULL to reset back to default bullet ones.

void btSetDbgMsgFn(btDbgMsgFn fn);
void btSetDbgWarnFn(btDbgMsgFn fn);

#endif  // BT_DEBUG_H
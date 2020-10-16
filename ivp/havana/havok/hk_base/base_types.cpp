#include <hk_base/base.h>


void hk_assert(bool test, const char* cond, int line, const char* file)
{
	if (test==false)
	{
		hk_Console::get_instance()->printf("%s:%i: assertion failed : '%s'\n", file, line, cond);
		hk_Console::get_instance()->flush();
		HK_BREAK;
	}
}

void hk_check(bool test, const char* cond, int line, const char* file)
{
	if (test==false)
	{
		hk_Console::get_instance()->printf("%s(%i): check failed : '%s'\n", file, line, cond);
		hk_Console::get_instance()->flush();
	}
}


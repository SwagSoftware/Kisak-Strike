#ifndef HK_BASE_MEMORY_UTIL_H
#define HK_BASE_MEMORY_UTIL_H

class hk_Memory_Util
{
	public:
		static void print_statistics(hk_Memory *mem = hk_Memory::get_instance(),
									class hk_Console *con = hk_Console::get_instance());
};

#endif /*HK_BASE_MEMORY_UTIL_H*/


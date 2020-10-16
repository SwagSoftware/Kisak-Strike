#include <hk_base/base.h>
#include <hk_base/memory/memory_util.h>


void hk_Memory_Util::print_statistics( hk_Memory *mem, hk_Console *out )
{
	const char *enum_to_string[HK_MEMORY_CLASS_MAX];
	{
		for (int i = 0; i< HK_MEMORY_CLASS_MAX; i++)	{		enum_to_string[i] = HK_NULL;		}
#define H(a) enum_to_string[a] = #a
		H(HK_MEMORY_CLASS_UNKNOWN);
		H(HK_MEMORY_CLASS_DUMMY);

		H(HK_MEMORY_CLASS_ARRAY);
		H(HK_MEMORY_CLASS_HASH);
		
		H(HK_MEMORY_CLASS_SORTED_SET);
		H(HK_MEMORY_CLASS_DENSE_VECTOR);
		H(HK_MEMORY_CLASS_ENTITY);
		H(HK_MEMORY_CLASS_RIGID_BODY);
		H(HK_MEMORY_CLASS_EFFECTOR);

		H(HK_MEMORY_CLASS_CORE);
		H(HK_MEMORY_CLASS_CONSTRAINT);

		H(HK_MEMORY_CLASS_MANAGER);
		H(HK_MEMORY_CLASS_SIMUNINT);
		H(HK_MEMORY_CLASS_SIM_SLOT);

		H(HK_MEMORY_CLASS_PSI_TIME);

		H(HK_MEMORY_CLASS_BROAD_PHASE);
		H(HK_MEMORY_CLASS_CA_DISPATCHER);

		//havok compat
		H(HK_MEMORY_CLASS_GENERIC);
		H(HK_MEMORY_CLASS_STL);
		H(HK_MEMORY_CLASS_MATH);
		H(HK_MEMORY_CLASS_EVENT);
		H(HK_MEMORY_CLASS_ACTION);
		H(HK_MEMORY_CLASS_GEOMETRY);
		H(HK_MEMORY_CLASS_CDINFO);
		H(HK_MEMORY_CLASS_DISPLAY);
		H(HK_MEMORY_CLASS_EXPORT);

		H(HK_MEMORY_CLASS_USR1);
		H(HK_MEMORY_CLASS_USR2);
		H(HK_MEMORY_CLASS_USR3);
		H(HK_MEMORY_CLASS_USR4);
#undef H
	}

	int max_mem = 0;
	int current_mem = 0;
	{ // summary
		for (int i = 0; i< HK_MEMORY_CLASS_MAX; i++)
		{
			if ( i == HK_MEMORY_CLASS_DUMMY) continue;
			max_mem += mem->m_statistics[i].m_max_size_in_use;
			current_mem += mem->m_statistics[i].m_size_in_use;
		}
	}

	out->printf( "\nMemory Summary\n" );
	out->printf(   "**************\n" );

	out->printf( "\n%32s: Bytes allocated %i, max bytes allocated %i\n","SUMMARY", current_mem, max_mem );


	{ // details per type
		out->printf( "\nDetails per type\n" );
		out->printf(   "****************\n" );
		for (int i = 0; i< HK_MEMORY_CLASS_MAX; i++)
		{
			if ( i == HK_MEMORY_CLASS_DUMMY) continue;

			hk_Memory::hk_Memory_Statistics &s = mem->m_statistics[i];
			if ( !s.m_max_size_in_use ) {
				continue;
			}

			const char *type_name = enum_to_string[ i ];
			if (!type_name){
				type_name = "hk_Memory::print_statistics does not know type";
			}
			out->printf( "%32s: blocks: %4i  size: %5i  max_size %5i  avg_size %5i  allocs %6i\n",
				type_name,
				s.m_blocks_in_use, s.m_size_in_use, 
				s.m_max_size_in_use, (s.m_blocks_in_use) ? s.m_size_in_use / s.m_blocks_in_use + 1 : 0, 
				s.m_n_allocates);
		}
	}

	{ // details per size
		out->printf( "\nDetails per size\n" );
		out->printf(   "****************\n" );

		for (int i = 0; i < HK_MEMORY_MAX_ROW; i++)
		{
			int free_blocks = 0;
			for ( hk_Memory::hk_Memory_Elem *el = mem->m_free_list[i]; el; el = el->m_next ){
				free_blocks ++;
			}
			out->printf( "%32s  blocks %4i  size %5i  free_elems %4i  total %6i\n",
				"", 
				mem->m_blocks_in_use[i],
				mem->m_row_to_size[i],
				free_blocks,
				mem->m_blocks_in_use[i] * mem->m_row_to_size[i]);
		}
	}
}


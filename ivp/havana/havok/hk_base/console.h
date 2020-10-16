#ifndef HK_BASE_CONSOLE_H
#define HK_BASE_CONSOLE_H

class hk_Console 
{
	public:

		virtual void printf( const char *format_string, ...);
		virtual void exit(int );
		virtual void flush();

		static hk_Console* get_instance();

	protected:

		static hk_Console* m_console;
		static hk_Console m_default_console_buffer;
			//: just to avoid a call to malloc
};

#define hkprintf hk_Console::get_instance()->printf
#endif /* HK_BASE_CONSOLE_H */


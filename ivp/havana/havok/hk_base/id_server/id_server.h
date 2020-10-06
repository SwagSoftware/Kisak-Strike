#ifndef HK_ID_SERVER
#define HK_ID_SERVER

// A very basic classs which just generates ID's 
// The only reason this exists in a singleton class
// is so we can link with DLL's easily

class hk_ID_Server
{
	public:

		static inline hk_ID_Server* get_instance();
		//: provide access to this class

		inline hk_id get_new_id();
		//: allocate a new id

	protected:

		hk_ID_Server();
		//: Hide the constructor

	private:

		static hk_ID_Server m_instance;
		// A pointer to the single instance of the ID server

		static hk_id m_next_id;
		// The next valid unique ID
};

#include <hk_base/id_server/id_server.inl>

#endif // HK_ID_SERVER

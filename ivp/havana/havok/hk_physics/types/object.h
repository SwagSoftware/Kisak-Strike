#ifndef HK_PHYSICS_OBJECT_H
#define HK_PHYSICS_OBJECT_H

// object is something which will be managed by the scene management
class hk_Object : public hkBaseObject
{
private:
	protected:
		hk_Environment *m_environment;

	protected:
		inline virtual ~hk_Object(){
		}

		inline hk_Object(hk_Environment *env): m_environment(env)	{	}

	public:
		inline hk_Environment *get_environment(){
			return m_environment;
		}
};


class hk_Sim_Object : public hk_Object
{
	public:

		virtual hk_sim_freq get_minimum_simulation_frequency(){ return 0;}
		
		inline void set_client_data( const hk_client );
		inline hk_client get_client_data() const;
		hk_id get_id() const { return m_id; };

		virtual void destroy_with_activation(){ delete this; };
		virtual void destroy_without_activation(){ delete this;};

		hk_Sim_Object(hk_Environment *env)
			: hk_Object(env)
		{
		}

		virtual ~hk_Sim_Object()
		{
		}

	private:

		friend class hk_Object_Factory;
		hk_id     m_id;
		hk_client m_client;
};

#endif //HK_PHYSICS_OBJECT_H

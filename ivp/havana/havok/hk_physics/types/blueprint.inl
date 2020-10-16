hk_Blueprint::hk_Blueprint( hk_type type, int size )
	:	m_id(0), m_type(type), m_length(size)
{
	// Get a new unique ID for this blueprint
}

hk_id hk_Blueprint::get_id() const
{
    return m_id;
}

hk_type hk_Blueprint::get_type() const
{
    return m_type;
}

unsigned int hk_Blueprint::get_size() const
{
    return m_length;
}

hk_size_t hk_Blueprint::size_of() const  
{
    return m_length;
}


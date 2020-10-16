///
/// INLINE FUNCTION DEFINITIONS
///

hk_ID_Server* hk_ID_Server::get_instance()
{
    return &m_instance;
}

hk_id hk_ID_Server::get_new_id()
{
    return m_next_id++;
}

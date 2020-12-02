#include "KisakURL.h"

#include <interfaces/interfaces.h>
#include "vgui/ISystem.h"

// RmlUI warns about leaks unless we add an instancer
// this kind of a HACK
class DummyInstancer : public Rml::ElementInstancer
{
    virtual Rml::ElementPtr InstanceElement(Rml::Element *parent, const Rml::String &tag, const Rml::XMLAttributes &attributes )
    {
        return nullptr;
    }
    virtual void ReleaseElement(Rml::Element *element)
    {
        // unneeded
    }
};
static DummyInstancer dummyInstancer;


KisakURL::KisakURL(const Rml::String &tag)  : Rml::Element( tag ), m_URL(), m_pElemLink( nullptr )
{
}
KisakURL::~KisakURL() noexcept
{
    this->RemoveEventListener( Rml::EventId::Click, this );
    RemoveChild( m_pElemLink );
    delete m_pElemLink;
    m_pElemLink = nullptr;
}
void KisakURL::ProcessEvent(Rml::Event &event)
{
    // Click listener on text
    g_pVGuiSystem->OpenURL( m_URL.c_str() );
    Msg("Opened URL: %s\n", m_URL.c_str() );
}


KisakURLInstancer::~KisakURLInstancer() noexcept
{
}
Rml::ElementPtr KisakURLInstancer::InstanceElement(Rml::Element *parent, const Rml::String &tag, const Rml::XMLAttributes &attributes)
{
    KisakURL *kisakUrl = new KisakURL( tag );
    kisakUrl->SetProperty("color", "#1a0dab;");
    kisakUrl->SetProperty("cursor", "pointer;");

    kisakUrl->m_URL = Rml::Get( attributes, "url", Rml::String("https://itsyourbirthday.today/") );

    // Remove spaces
    kisakUrl->m_URL.erase( std::remove( kisakUrl->m_URL.begin(), kisakUrl->m_URL.end(), ' '), kisakUrl->m_URL.end() );

    kisakUrl->m_pElemLink = new Rml::Element("b");
    kisakUrl->m_pElemLink->SetInstancer( &dummyInstancer );
    kisakUrl->SetInnerRML( Rml::Get( attributes, "text", kisakUrl->m_URL ) );
    kisakUrl->AppendChild( Rml::ElementPtr( kisakUrl->m_pElemLink ) );

    // add Click listener
    kisakUrl->AddEventListener( Rml::EventId::Click, kisakUrl );

    return Rml::ElementPtr( kisakUrl );
}
void KisakURLInstancer::ReleaseElement(Rml::Element *element)
{
    delete element;
}
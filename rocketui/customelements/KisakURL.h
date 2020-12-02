#ifndef KISAKSTRIKE_KISAKURL_H
#define KISAKSTRIKE_KISAKURL_H

#include <RmlUi/Core/Element.h>
#include <RmlUi/Core/ElementInstancer.h>
#include <RmlUi/Core/EventListener.h>

class KisakURL : public Rml::Element, public Rml::EventListener
{
public:
    KisakURL(const Rml::String& tag);
    virtual ~KisakURL();
    virtual void ProcessEvent(Rml::Event& event);

    Rml::String m_URL;
    Rml::Element *m_pElemLink;
};

class KisakURLInstancer : public Rml::ElementInstancer
{
public:
    virtual ~KisakURLInstancer();

    /// Instances an element given the tag name and attributes
    /// @param tag Name of the element to instance
    /// @param attributes vector of name value pairs
    Rml::ElementPtr InstanceElement(Rml::Element* parent, const Rml::String& tag, const Rml::XMLAttributes& attributes) override;

    /// Releases the given element
    /// @param element to release
    void ReleaseElement(Rml::Element* element) override;
};

#endif //KISAKSTRIKE_KISAKURL_H

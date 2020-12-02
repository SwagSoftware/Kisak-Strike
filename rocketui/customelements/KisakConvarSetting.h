#ifndef KISAKSTRIKE_KISAKCONVARSETTING_H
#define KISAKSTRIKE_KISAKCONVARSETTING_H

#include <RmlUi/Core/Element.h>
#include <RmlUi/Core/ElementInstancer.h>
#include <RmlUi/Core/EventListener.h>
#include <RmlUi/Core/Elements/ElementFormControlSelect.h>
#include <RmlUi/Core/Elements/ElementFormControlInput.h>

class ConVarRef;

class KisakConvarSetting : public Rml::Element
{
public:
    KisakConvarSetting( const Rml::String& tag );
    virtual ~KisakConvarSetting();

    void ParseBasicAttributes( const Rml::XMLAttributes &attributes );

    Rml::String m_displayName;
    ConVarRef *m_refConvar;
    Rml::Element *m_pElemDescription;
    Rml::Element *m_pElemBR;
};
class KisakConvarSettingSlider : public KisakConvarSetting, public Rml::EventListener
{
public:
    KisakConvarSettingSlider(const Rml::String& tag);
    virtual ~KisakConvarSettingSlider();
    virtual void ProcessEvent(Rml::Event& event);

    float m_sliderMax;
    float m_sliderMin;
    Rml::ElementFormControlInput *m_pElemSlider;
};
class KisakConvarSettingSliderInstancer : public Rml::ElementInstancer
{
public:
    virtual ~KisakConvarSettingSliderInstancer();

    /// Instances an element given the tag name and attributes
    /// @param tag Name of the element to instance
    /// @param attributes vector of name value pairs
    Rml::ElementPtr InstanceElement(Rml::Element* parent, const Rml::String& tag, const Rml::XMLAttributes& attributes) override;

    /// Releases the given element
    /// @param element to release
    void ReleaseElement(Rml::Element* element) override;
};

//up to 32 enum values :)
#define MAX_ENUM_VALUES 32
class KisakConvarSettingEnum : public KisakConvarSetting, public Rml::EventListener
{
public:
    KisakConvarSettingEnum(const Rml::String& tag);
    virtual ~KisakConvarSettingEnum();
    virtual void ProcessEvent(Rml::Event& event);

    int m_enumValues[MAX_ENUM_VALUES];
    Rml::String m_enumValueNames[MAX_ENUM_VALUES];
    int m_enumValuesCount;
    Rml::ElementFormControlSelect *m_pElemSelect;
};
class KisakConvarSettingEnumInstancer : public Rml::ElementInstancer
{
public:
    virtual ~KisakConvarSettingEnumInstancer();

    /// Instances an element given the tag name and attributes
    /// @param tag Name of the element to instance
    /// @param attributes vector of name value pairs
    Rml::ElementPtr InstanceElement(Rml::Element* parent, const Rml::String& tag, const Rml::XMLAttributes& attributes) override;

    /// Releases the given element
    /// @param element to release
    void ReleaseElement(Rml::Element* element) override;
};


//TODO
//class KisakConvarSettingKeybind : KisakConvarSetting
//{
//    // use parent constructor
//    using KisakConvarSetting::KisakConvarSetting;
//};


#endif //KISAKSTRIKE_KISAKCONVARSETTING_H

#include "KisakConvarSetting.h"

#include <strtools.h>
#include <convar.h>

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

KisakConvarSetting::KisakConvarSetting(const Rml::String &tag) : Rml::Element( tag ), m_refConvar( nullptr ), m_pElemBR( nullptr ), m_pElemDescription( nullptr )
{
}
KisakConvarSetting::~KisakConvarSetting() noexcept
{
    RemoveChild( m_pElemBR );
    delete m_pElemBR;
    m_pElemBR = nullptr;

    RemoveChild( m_pElemDescription );
    delete m_pElemDescription;
    m_pElemDescription = nullptr;

    delete m_refConvar;
    m_refConvar = nullptr;
}
void KisakConvarSetting::ParseBasicAttributes(const Rml::XMLAttributes &attributes)
{
    m_displayName = Rml::Get(attributes, "desc", Rml::String( "No Description" ) );
    m_refConvar = new ConVarRef(Rml::Get(attributes, "convar", Rml::String( "rocket_rml_error_no_convar") ).c_str() );

    // Description Text
    m_pElemDescription = new Rml::Element("b");
    m_pElemDescription->SetInstancer( &dummyInstancer );
    m_pElemDescription->SetInnerRML( m_displayName );
    this->AppendChild( Rml::ElementPtr( m_pElemDescription ) );

    // Add newline
    m_pElemBR = new Rml::Element("br");
    m_pElemBR->SetInstancer( &dummyInstancer );
    this->AppendChild( Rml::ElementPtr( m_pElemBR ) );
}
KisakConvarSettingSlider::KisakConvarSettingSlider(const Rml::String &tag) : KisakConvarSetting( tag ), m_pElemSlider( nullptr )
{
}
KisakConvarSettingSlider::~KisakConvarSettingSlider() noexcept
{
    if( m_pElemSlider )
        m_pElemSlider->RemoveEventListener( Rml::EventId::Change, this );

    RemoveChild( m_pElemSlider );
    delete m_pElemSlider;
    m_pElemSlider = nullptr;
}
void KisakConvarSettingSlider::ProcessEvent(Rml::Event& event)
{
    // called for Change events from the slider.
    Rml::String value = event.GetParameter< Rml::String >("value", "none" );
    float fValue;

    if( value == "none" || value.empty() )
        return;

    // for some reason the slider is [0->100]
    fValue = std::stof( value );

    // happens if you're manually typing in a value
    if( fValue < m_sliderMin || fValue > m_sliderMax )
        return;

    // Set the convar value
    m_refConvar->SetValue( fValue );
}
KisakConvarSettingSliderInstancer::~KisakConvarSettingSliderInstancer() noexcept
{
}
Rml::ElementPtr KisakConvarSettingSliderInstancer::InstanceElement(Rml::Element *parent, const Rml::String &tag, const Rml::XMLAttributes &attributes)
{
    KisakConvarSettingSlider *slider = new KisakConvarSettingSlider( tag );

    // Parse and create Description and newline elements and init the convar
    slider->ParseBasicAttributes( attributes );

    // Create the slider
    slider->m_pElemSlider = new Rml::ElementFormControlInput("input");
    slider->m_pElemSlider->SetInstancer( &dummyInstancer );

    // this will trigger the input to be "slider" type and create a new subclass.
    // Make sure it's done first otherwise the properties aren't set.
    slider->m_pElemSlider->SetAttribute("type", "range");

    // Parse attributes for slider-properties
    const float min = Rml::Get(attributes, "slider-min", 0.0f );
    const float max = Rml::Get(attributes, "slider-max", 0.0f );
    const float step = Rml::Get(attributes, "slider-step", 0.0f );
    const float currentValue = slider->m_refConvar->GetFloat();
    slider->m_pElemSlider->SetAttribute("min", Rml::ToString(min));
    slider->m_pElemSlider->SetAttribute("max", Rml::ToString(max));
    slider->m_pElemSlider->SetAttribute("step", Rml::ToString(step));
    slider->m_pElemSlider->SetAttribute("value", Rml::ToString(currentValue));

    slider->m_sliderMax = max;
    slider->m_sliderMin = min;

    // Event listener for changes
    slider->m_pElemSlider->AddEventListener(Rml::EventId::Change, slider );



    slider->AppendChild( Rml::ElementPtr( slider->m_pElemSlider ) );

    return Rml::ElementPtr( slider );
}
void KisakConvarSettingSliderInstancer::ReleaseElement(Rml::Element *element)
{
    delete element;
}



KisakConvarSettingEnum::KisakConvarSettingEnum(const Rml::String &tag) : KisakConvarSetting( tag ), m_pElemSelect( nullptr )
{
}
KisakConvarSettingEnum::~KisakConvarSettingEnum()
{
    if( m_pElemSelect )
        m_pElemSelect->RemoveEventListener( Rml::EventId::Change, this );

    RemoveChild( m_pElemSelect );
    delete m_pElemSelect;
    m_pElemSelect = nullptr;
}
void KisakConvarSettingEnum::ProcessEvent(Rml::Event& event)
{
    // Only registered for CHANGE event when the dropdown gets updated.
    Rml::String value = event.GetParameter< Rml::String >("value", "none" );
    int iValue;

    if( value == "none" )
        return;

    iValue = std::stoi( value );

    // Convar is already set to this value, don't want to fire any convar change handlers
    if( iValue == m_refConvar->GetInt() )
        return;

    Msg("KisakConvarSettingEnum - Setting %s to %d\n", m_refConvar->GetName(), iValue );
    m_refConvar->SetValue( iValue );
}
KisakConvarSettingEnumInstancer::~KisakConvarSettingEnumInstancer() noexcept
{
}

Rml::ElementPtr KisakConvarSettingEnumInstancer::InstanceElement(Rml::Element *parent, const Rml::String &tag, const Rml::XMLAttributes &attributes)
{
    KisakConvarSettingEnum *settingEnum = new KisakConvarSettingEnum( tag );

    // Parse and create Description and newline elements and init the convar
    settingEnum->ParseBasicAttributes( attributes );

    settingEnum->m_enumValuesCount = Rml::Get( attributes, "enum-values-count", 0 );

    // Create the dropdown.
    settingEnum->m_pElemSelect = new Rml::ElementFormControlSelect("select");
    settingEnum->m_pElemSelect->SetInstancer( &dummyInstancer );

    // Add a change event listener
    settingEnum->m_pElemSelect->AddEventListener( Rml::EventId::Change, settingEnum );

    // Add it as a child of this custom element
    settingEnum->AppendChild( Rml::ElementPtr( settingEnum->m_pElemSelect ) );

    bool foundValue = false;
    for( int i = 0; (i < settingEnum->m_enumValuesCount) && (i < MAX_ENUM_VALUES); i++ )
    {
        char attributeStr[64];

        // get the number value for each enum
        V_snprintf( attributeStr, sizeof(attributeStr), "enum-value-%d", i );
        settingEnum->m_enumValues[i] = Rml::Get(attributes, attributeStr, 0);

        // now get the name for each one.
        V_snprintf( attributeStr, sizeof(attributeStr), "enum-name-%d", i );
        settingEnum->m_enumValueNames[i] = Rml::Get(attributes, attributeStr, Rml::String("RML ERROR! UNKNOWN_ENUM_NAME" ) );

        // Add a new dropdown option..
        settingEnum->m_pElemSelect->Add( settingEnum->m_enumValueNames[i], Rml::ToString(settingEnum->m_enumValues[i]) );

        // if this dropdown == convar, set it as selected.
        if( settingEnum->m_enumValues[i] == settingEnum->m_refConvar->GetInt() )
        {
            foundValue = true;
            settingEnum->m_pElemSelect->SetSelection( i );
        }
    }

    // if we didn't find the value, it means the convar is currently assigned to a value that isn't in the .rml
    // Add a "Choose" option and set it as selected
    if( !foundValue )
    {
        settingEnum->m_pElemSelect->Add("Choose...", "none");
        settingEnum->m_pElemSelect->SetSelection( settingEnum->m_enumValuesCount );
    }
    return Rml::ElementPtr( settingEnum );
}
void KisakConvarSettingEnumInstancer::ReleaseElement(Rml::Element *element)
{
    delete element;
}

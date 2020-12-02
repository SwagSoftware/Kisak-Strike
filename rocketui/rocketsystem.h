#ifndef KISAKSTRIKE_ROCKETSYSTEM_H
#define KISAKSTRIKE_ROCKETSYSTEM_H

#include <RmlUi/Core/SystemInterface.h>

#if USE_SDL
#include <SDL_mouse.h>
#else
#error fixme
#endif

class RocketSystem : public Rml::SystemInterface
{
    /** singleton support **/
public:
    static RocketSystem m_Instance;
#if USE_SDL
    SDL_Cursor *m_pCursors[SDL_NUM_SYSTEM_CURSORS];
#endif
    /// Get the number of seconds elapsed since the start of the application
    /// @returns Seconds elapsed
    double GetElapsedTime() override;

    /// Log the specified message.
    /// @param[in] type Type of log message, ERROR, WARNING, etc.
    /// @param[in] message Message to log.
    /// @return True to continue execution, false to break into the debugger.
    virtual bool LogMessage(Rml::Log::Type type, const Rml::String& message) override;

    //lwss- Allocates various system cursors
    void InitCursors();
    //lwss- Free system Cursors from above
    void FreeCursors();

    /// Set mouse cursor.
    /// @param[in] cursor_name Cursor name to activate.
    void SetMouseCursor(const Rml::String& cursor_name) override;

    /// Set clipboard text.
    /// @param[in] text Text to apply to clipboard.
    void SetClipboardText(const Rml::String& text) override;

    /// Get clipboard text.
    /// @param[out] text Retrieved text from clipboard.
    void GetClipboardText(Rml::String& text) override;
};


#endif //KISAKSTRIKE_ROCKETSYSTEM_H

#ifndef KISAKSTRIKE_RKHUD_RADAR_H
#define KISAKSTRIKE_RKHUD_RADAR_H

#include <rocketui/rocketui.h>
#include "hudelement.h"

extern ConVar cl_drawhud;

struct SpottedInfo
{
    // info from ProcessSpottedEntityUpdate
    int entId;
    int classId;
    int originX;
    int originY;
    int originZ;
    int angleYaw;
    bool defuser;
    bool playerWithDefuser;
    bool playerWithC4;

    float timelastSpotted;
};

class RkHudRadar : public CHudElement
{
public:
    explicit RkHudRadar( const char *value );
    virtual ~RkHudRadar();

    // Overrides from CHudElement
    void LevelInit(void);
    virtual void LevelShutdown(void);
    virtual void SetActive(bool bActive);
    virtual bool ShouldDraw(void);
    void ShowPanel(bool bShow, bool force);

    // Hooked msg
    bool MsgFunc_ProcessSpottedEntityUpdate( const CCSUsrMsg_ProcessSpottedEntityUpdate &msg );

    void UpdateRadarFrame();
    void UpdateRadarSize();

    Rml::ElementDocument    *m_pInstance;
    bool		    m_bVisible;

    float           m_radarX;
    float           m_radarY;
    float           m_radarWidth;
    float           m_radarHeight;
    float           m_radarCenterX;
    float           m_radarCenterY;
    Rml::Element    *m_playerElements[MAX_PLAYERS];
    SpottedInfo     m_spottedPlayers[MAX_PLAYERS];
    Rml::Element    *m_defuserElement;
    SpottedInfo     m_spottedDefuser;
    Rml::Element    *m_c4Element;
    SpottedInfo     m_spottedC4;

    // Cached elements from instance.
    Rml::Element *m_elemBody;
    Rml::Element *m_elemContainer;

    CUserMessageBinder m_UMCMsgProcessSpottedEntityUpdate;
};


#endif //KISAKSTRIKE_RKHUD_RADAR_H

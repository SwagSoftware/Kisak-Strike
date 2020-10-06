// Copyright (C) Ipion Software GmbH 2000. All rights reserved.

// IVP_EXPORT_PUBLIC

#ifndef IVP_STATISTICSMANAGER_CONSOLE_CALLBACK_INCLUDED
#define IVP_STATISTICSMANAGER_CONSOLE_CALLBACK_INCLUDED


/********************************************************************************
 *	Name:	    	IVP_Statisticsmanager_Console_Callback   	
 *	Description:	A callback called whenever the clustering visualizer
 *                      needs to update its display.
 ********************************************************************************/

class IVP_Statisticsmanager_Console_Callback : public IVP_BetterStatisticsmanager_Callback_Interface {
private:
    void output_request(IVP_BetterStatisticsmanager_Data_Entity *entity);
    void enable();
    void disable();

public:
    IVP_Statisticsmanager_Console_Callback();
};

#endif


#pragma once

#include <process.h>  
#include <iostream>  
#include <tchar.h>
#include "processMavlink.h"


#define CIRCLE_TIME 32


using namespace std;

enum Mode {
	DELIVER_COPTER_INFO_TO_GCS,
	DELIVER_GCS_COMMAND_TO_HAWK,
	CONTROL_NOTHING,
};


class CPositionController
{
public:
	CPositionController(CMavLinkProcessor* CMavLinkProcessorPtr1, Mode* ModePtr);
	CPositionController(CMavLinkProcessor* CMavLinkProcessorPtr1, CMavLinkProcessor* CMavLinkProcessorPtr2, Mode* ModePtr);
	CPositionController(CMavLinkProcessor* CMavLinkProcessorPtr1, CMavLinkProcessor* CMavLinkProcessorPtr2, CMavLinkProcessor* CMavLinkProcessorPtr3, Mode* ModePtr);
	~CPositionController();
	static CMavLinkProcessor* SerialPortPtrHAWK;	//coving UAV 1-4
	static CMavLinkProcessor* SerialPortPtrGCS;	//coving UAV 5-8
	static CMavLinkProcessor* SerialPortPtr3;	//coving UAV 9-12

	static Mode* ControlModePtr;

	bool OpenControllerThread();
	bool CloseControllerTread();
	
	bool modeEnd = 0;

private:

	/** 线程退出标志变量 */   
	static bool s_controlExit;  

	/** 线程句柄 */   
	volatile HANDLE    m_controlThread;  

	static UINT WINAPI ControllerThread(void* pParam);

	void TransferAttitude (int Attitude[ATTITUDE_DIM], double Attitude_trans[ATTITUDE_DIM]);

};
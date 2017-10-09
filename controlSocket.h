#pragma once

#include <process.h>  
#include <iostream>  
#include <tchar.h>
#include "processMavlink.h"
#include <winsock2.h> 
#include <WS2tcpip.h>
#pragma comment(lib,"ws2_32.lib") 


#define SOCKETMSG_ID_ARM_DISARM 1
#define SOCKETMSG_ID_TAKEOFF 2
#define SOCKETMSG_ID_LAND 3
#define SOCKETMSG_ID_SET_VELOCITY 4
#define SOCKETMSG_ID_GET_VELOCITY 5
#define SOCKETMSG_ID_SET_GLOBAL_POS 6
#define SOCKETMSG_ID_GET_GLOBAL_POS 7
#define SOCKETMSG_ID_HOVER 8
#define SOCKETMSG_ID_SET_HEADING 9
#define SOCKETMSG_ID_GET_HEADING 10

#define SYSID_THISCOPTER 1

using namespace std;

enum SocketMode {
	DELIVER_COPTER_INFO_TO_SOCKET,
	DELIVER_SOCKET_COMMAND_TO_HAWK,
	SOCKET_NOTHING,
};

const int nDouble = sizeof(double);
const int nFloat = sizeof(float);
const int nInt16 = sizeof(int16_t);

union ch_d {
	double dd;
	char ch[nDouble];
};

union ch_f {
	float ff;
	char ch[nFloat];
};

union ch_int16 {
	int16_t i16;
	char ch[nInt16];
};

class CSocketController
{
public:
	CSocketController(CMavLinkProcessor* CMavLinkProcessorPtr1, SocketMode* ModePtr);
	CSocketController(CMavLinkProcessor* CMavLinkProcessorPtr1, CMavLinkProcessor* CMavLinkProcessorPtr2, SocketMode* ModePtr);
	CSocketController(CMavLinkProcessor* CMavLinkProcessorPtr1, CMavLinkProcessor* CMavLinkProcessorPtr2, CMavLinkProcessor* CMavLinkProcessorPtr3, SocketMode* ModePtr);
	~CSocketController();
	static CMavLinkProcessor* SerialPortPtrHAWK;
	static CMavLinkProcessor* SerialPortPtrGCS;
	static CMavLinkProcessor* SerialPortPtr3;

	static SocketMode* SocketModePtr;

	bool OpenSocketThread();
	bool CloseSocketTread();

	bool modeEnd = 0;

	bool isSocketTalk = 0;

private:

	/** 线程退出标志变量 */
	static bool s_controlExit;

	/** 线程句柄 */
	volatile HANDLE    m_controlThread;

	static UINT WINAPI SocketThread(void* pParam);

};
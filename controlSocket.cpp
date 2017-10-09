#include "controlSocket.h"
#include <math.h>
#include <conio.h>
#include <fstream>
#include <mutex>



using namespace std;

bool CSocketController::s_controlExit = false;
CMavLinkProcessor* CSocketController::SerialPortPtrHAWK;
CMavLinkProcessor* CSocketController::SerialPortPtrGCS;
CMavLinkProcessor* CSocketController::SerialPortPtr3;
SocketMode* CSocketController::SocketModePtr;


CSocketController::CSocketController(CMavLinkProcessor* CMavLinkProcessorPtr1, SocketMode* ModePtr) : m_controlThread(INVALID_HANDLE_VALUE)
{
	m_controlThread = INVALID_HANDLE_VALUE;
	SerialPortPtrHAWK = CMavLinkProcessorPtr1;
	SocketModePtr = ModePtr;

}

CSocketController::CSocketController(CMavLinkProcessor* CMavLinkProcessorPtr1, CMavLinkProcessor* CMavLinkProcessorPtr2, SocketMode* ModePtr) : m_controlThread(INVALID_HANDLE_VALUE)
{
	m_controlThread = INVALID_HANDLE_VALUE;
	SerialPortPtrHAWK = CMavLinkProcessorPtr1;
	SerialPortPtrGCS = CMavLinkProcessorPtr2;
	SocketModePtr = ModePtr;




}

CSocketController::CSocketController(CMavLinkProcessor* CMavLinkProcessorPtr1, CMavLinkProcessor* CMavLinkProcessorPtr2, CMavLinkProcessor* CMavLinkProcessorPtr3, SocketMode* ModePtr) : m_controlThread(INVALID_HANDLE_VALUE)
{
	m_controlThread = INVALID_HANDLE_VALUE;
	SerialPortPtrHAWK = CMavLinkProcessorPtr1;
	SerialPortPtrGCS = CMavLinkProcessorPtr2;
	SerialPortPtr3 = CMavLinkProcessorPtr3;
	SocketModePtr = ModePtr;

}

CSocketController::~CSocketController()
{
	CloseSocketTread();
}

bool CSocketController::OpenSocketThread()
{
	/** 检测线程是否已经开启了 */
	if (m_controlThread != INVALID_HANDLE_VALUE)
	{
		/** 线程已经开启 */
		return false;
	}

	s_controlExit = false;
	/** 线程ID */
	UINT controlThreadId;
	/** 开启串口数据监听线程 */
	m_controlThread = (HANDLE)_beginthreadex(NULL, 0, SocketThread, this, 0, &controlThreadId);
	if (!m_controlThread)
	{
		return false;
	}
	/** 设置线程的优先级,高于普通线程 */

	if (!SetThreadPriority(m_controlThread, THREAD_PRIORITY_ABOVE_NORMAL))
	{
		return false;
	}

	return true;
}

bool CSocketController::CloseSocketTread()
{
	if (m_controlThread != INVALID_HANDLE_VALUE)
	{
		/** 通知线程退出 */
		s_controlExit = true;

		/** 等待线程退出 */
		Sleep(10);

		/** 置线程句柄无效 */
		CloseHandle(m_controlThread);
		m_controlThread = INVALID_HANDLE_VALUE;
	}
	return true;
}

UINT WINAPI CSocketController::SocketThread(void* pParam)
{
	/** 得到本类的指针 */
	CSocketController *PositionControllerPtr = reinterpret_cast<CSocketController*>(pParam);

	mavlink1_message_t msg;
	//---------for command long-------------
	float command_long_param_arm_disarm[7] = { 0,21196,0,0,0,0,0 };
	float command_long_param_takeoff[7] = { 0,0,0,0,0,0,15 };
	float command_long_param_setmode[7] = { 0,0,0,0,0,0,0 };
	float command_long_param_yaw[7] = { 0,0,0,0,0,0,0 };
	float command_long_param_land[7] = { 0,0,0,0,0,0,0 };

	//------------for set pos global int--------------
	int32_t lat_lng[2] = { 0, 0 };
	float param_set_pos_global_int[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };	//alt;vx;vy;vz;afx;afy;afz;yaw;yaw_rate
	uint8_t target_frame[3] = { 1, 1, 6 };	//target_system, target_component, coordinate_frame
	uint16_t type_mask = 4039;
	uint16_t type_mask_velosity = 4039;		//0b111111000111
	uint16_t type_mask_position = 0b111111111000;


	//socket初始化
	//初始化WSA windows自带的socket
	WORD sockVersion = MAKEWORD(2, 2);
	WSADATA wsaData;
	if (WSAStartup(sockVersion, &wsaData) != 0)
	{
		return 0;
	}
	//创建服务端套接字
	SOCKET slisten = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (slisten == INVALID_SOCKET)
	{
		printf("socket error !");
		return 0;
	}
	//服务端需要绑定ip和端口
	sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_port = htons(8888);
	sin.sin_addr.S_un.S_addr = INADDR_ANY; //监听任意的地址
	if (::bind(slisten, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR) //将服务端套接字与上面的ip和端口绑定 
	{
		printf("bind error !");
	}
	//开始监听
	if (listen(slisten, 5) == SOCKET_ERROR)  //用listen（） 监听前面绑定好的slisten套接字
	{
		printf("listen error !");
		return 0;
	}
	//循环接受数据
	SOCKET sClient;  //创建连接的套接字
	sockaddr_in remoteAddr;
	int nAddrlen = sizeof(remoteAddr); //用于接受客户端地址
	char revData[255]; //存储接受的数据

	// 线程循环,轮询方式读取串口数据  
	while (true)
	{
		Sleep(100);
		printf("等待连接...\n");
		sClient = accept(slisten, (SOCKADDR *)&remoteAddr, &nAddrlen); //和客户端 connect()对应
		if (sClient == INVALID_SOCKET)
		{
			printf("accept error !");
			continue;
		}
		char sendBuf[20] = { '\0' };
		printf("接受到一个连接：%s \r\n", inet_ntop(AF_INET, (void*)&remoteAddr.sin_addr, sendBuf, 16));
		//数据接收
		int ret = recv(sClient, revData, 255, 0);
		if (ret)
		{
			PositionControllerPtr->isSocketTalk = true;
			revData[ret] = 0x00;
			//printf(revData);

			//解析一下收到的包是get包还是命令包
			//是命令包，如command_long等指令,发命令给hawk;
			//是get类型包,发送数据给客户端
			switch (revData[0])
				//revData[0]为自己定义的socketmsg_id
			{	
			case SOCKETMSG_ID_ARM_DISARM:
			{
				const char *sendData = "ARM package received！ \n";
				send(sClient, sendData, strlen(sendData), 0);
				//SOCKETMSG_ID_ARM_DISARM 的 revData[1] 标志着解锁或锁上
				command_long_param_arm_disarm[0] = revData[1];
				SerialPortPtrHAWK->mavlink_msg_command_long_pack(255, 190, &msg, SYSID_THISCOPTER, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, command_long_param_arm_disarm);
				break;
			}

			case SOCKETMSG_ID_TAKEOFF:
			{
				const char *sendData = "TAKEOFF package received！ \n";
				send(sClient, sendData, strlen(sendData), 0);
				ch_d alt;
				for (int i = 0; i < nDouble; i++)
				{
					alt.ch[i] = revData[i+1];
				}
				command_long_param_takeoff[6] = alt.dd;
				SerialPortPtrHAWK->mavlink_msg_command_long_pack(255, 190, &msg, SYSID_THISCOPTER, 1, MAV_CMD_NAV_TAKEOFF, 0, command_long_param_takeoff);
				break;
			}

			case SOCKETMSG_ID_LAND:
			{
				const char *sendData = "LAND package received！ \n";
				send(sClient, sendData, strlen(sendData), 0);
				SerialPortPtrHAWK->mavlink_msg_set_mode_pack(255, 190, &msg, 9, SYSID_THISCOPTER, 1);// switch to land mode
				break;
			}

			case SOCKETMSG_ID_SET_VELOCITY:
			{
				const char *sendData = "VELOCITY package received！ \n";
				send(sClient, sendData, strlen(sendData), 0);
				ch_d Vx, Vy, Vz;
				for (int i = 0; i < nDouble; i++)
				{
					Vx.ch[i] = revData[i + 1];
					Vy.ch[i] = revData[i + 9];
					Vz.ch[i] = revData[i + 17];
				}
				target_frame[0] = SYSID_THISCOPTER;
				param_set_pos_global_int[1] = Vx.dd;
				param_set_pos_global_int[2] = Vy.dd;
				param_set_pos_global_int[3] = Vz.dd;
				SerialPortPtrHAWK->mavlink_msg_set_pos_global_int_pack(255, 190, &msg, 0, lat_lng, param_set_pos_global_int, type_mask, target_frame);
				break;
			}

			case SOCKETMSG_ID_GET_VELOCITY:
			{
				char velocity[6];
				for (int i = 0; i < 6; i++)
				{
					velocity[i] = SerialPortPtrHAWK->MavlinkMessage_33[26 + i];
				}
				const char *sendData = velocity;
				send(sClient, sendData, strlen(sendData), 0);
				break;
			}

			case SOCKETMSG_ID_SET_GLOBAL_POS:
			{
				const char *sendData = "SET_GLOBAL_POS package received！ \n";
				send(sClient, sendData, strlen(sendData), 0);
				ch_d Lng, Lat, Alt;
				for (int i = 0; i < nDouble; i++)
				{
					Lng.ch[i] = revData[i + 1];
					Lat.ch[i] = revData[i + 9];
					Alt.ch[i] = revData[i + 17];
				}
				target_frame[0] = SYSID_THISCOPTER;
				lat_lng[0] = Lat.dd * 10000000;	//Lat
				lat_lng[1] = Lng.dd * 10000000;	//Lng
				param_set_pos_global_int[0] = Alt.dd;
				SerialPortPtrHAWK->mavlink_msg_set_pos_global_int_pack(255, 190, &msg, 0, lat_lng, param_set_pos_global_int, type_mask_position, target_frame);
				break;

				break;
			}

			case SOCKETMSG_ID_GET_GLOBAL_POS:
			{
				char position[12];
				for (int i = 0; i < 8; i++)
				{
					position[i] = SerialPortPtrHAWK->MavlinkMessage_33[10 + i];
				}
				for (int i = 0; i < 4; i++)
				{
					position[i + 8] = SerialPortPtrHAWK->MavlinkMessage_33[22 + i];
				}
				const char *sendData = position;
				send(sClient, sendData, strlen(sendData), 0);
				break;
			}

			case SOCKETMSG_ID_HOVER:
			{
				const char *sendData = "HOVER package received！ \n";
				send(sClient, sendData, strlen(sendData), 0);
				target_frame[0] = SYSID_THISCOPTER;
				param_set_pos_global_int[1] = 0;
				param_set_pos_global_int[2] = 0;
				param_set_pos_global_int[3] = 0;
				SerialPortPtrHAWK->mavlink_msg_set_pos_global_int_pack(255, 190, &msg, 0, lat_lng, param_set_pos_global_int, type_mask, target_frame);
				break;
				break;
			}

			case SOCKETMSG_ID_SET_HEADING:
			{
				const char *sendData = "SET_HEADING package received！ \n";
				send(sClient, sendData, strlen(sendData), 0);
				ch_d Yaw;
				for (int i = 0; i < nDouble; i++)
				{
					Yaw.ch[i] = revData[i + 1];
				}
				command_long_param_yaw[0] = Yaw.dd; //目标角度: [0，360）, 0 为正北，90为正东，以此类推
				command_long_param_yaw[1] = 20; //转速：度 / s
				command_long_param_yaw[2] = 1; //转动方向， - 1位顺时针，1位逆时针
				command_long_param_yaw[3] = 0; //0为绝对角度，1为相对角度
				SerialPortPtrHAWK->mavlink_msg_command_long_pack(255, 190, &msg, SYSID_THISCOPTER, 1, 115, 0, command_long_param_yaw);
				break;
			}
				
				
			case SOCKETMSG_ID_GET_HEADING:
			{
				char hdg[2];
				for (int i = 0; i < 2; i++)
				{
					hdg[i] = SerialPortPtrHAWK->MavlinkMessage_33[32 + i];
				}
				const char *sendData = hdg;
				send(sClient, sendData, strlen(sendData), 0);
				break;
			}

			default:
				break;
			}
		}
		closesocket(sClient);  //关闭已接通的套接字
		//cout << "在线程中" << endl;
	}
	closesocket(slisten); //关闭监听的套接字
	WSACleanup();
	return 0;
}

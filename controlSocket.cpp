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
	/** ����߳��Ƿ��Ѿ������� */
	if (m_controlThread != INVALID_HANDLE_VALUE)
	{
		/** �߳��Ѿ����� */
		return false;
	}

	s_controlExit = false;
	/** �߳�ID */
	UINT controlThreadId;
	/** �����������ݼ����߳� */
	m_controlThread = (HANDLE)_beginthreadex(NULL, 0, SocketThread, this, 0, &controlThreadId);
	if (!m_controlThread)
	{
		return false;
	}
	/** �����̵߳����ȼ�,������ͨ�߳� */

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
		/** ֪ͨ�߳��˳� */
		s_controlExit = true;

		/** �ȴ��߳��˳� */
		Sleep(10);

		/** ���߳̾����Ч */
		CloseHandle(m_controlThread);
		m_controlThread = INVALID_HANDLE_VALUE;
	}
	return true;
}

UINT WINAPI CSocketController::SocketThread(void* pParam)
{
	/** �õ������ָ�� */
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


	//socket��ʼ��
	//��ʼ��WSA windows�Դ���socket
	WORD sockVersion = MAKEWORD(2, 2);
	WSADATA wsaData;
	if (WSAStartup(sockVersion, &wsaData) != 0)
	{
		return 0;
	}
	//����������׽���
	SOCKET slisten = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (slisten == INVALID_SOCKET)
	{
		printf("socket error !");
		return 0;
	}
	//�������Ҫ��ip�Ͷ˿�
	sockaddr_in sin;
	sin.sin_family = AF_INET;
	sin.sin_port = htons(8888);
	sin.sin_addr.S_un.S_addr = INADDR_ANY; //��������ĵ�ַ
	if (::bind(slisten, (LPSOCKADDR)&sin, sizeof(sin)) == SOCKET_ERROR) //��������׽����������ip�Ͷ˿ڰ� 
	{
		printf("bind error !");
	}
	//��ʼ����
	if (listen(slisten, 5) == SOCKET_ERROR)  //��listen���� ����ǰ��󶨺õ�slisten�׽���
	{
		printf("listen error !");
		return 0;
	}
	//ѭ����������
	SOCKET sClient;  //�������ӵ��׽���
	sockaddr_in remoteAddr;
	int nAddrlen = sizeof(remoteAddr); //���ڽ��ܿͻ��˵�ַ
	char revData[255]; //�洢���ܵ�����

	// �߳�ѭ��,��ѯ��ʽ��ȡ��������  
	while (true)
	{
		Sleep(100);
		printf("�ȴ�����...\n");
		sClient = accept(slisten, (SOCKADDR *)&remoteAddr, &nAddrlen); //�Ϳͻ��� connect()��Ӧ
		if (sClient == INVALID_SOCKET)
		{
			printf("accept error !");
			continue;
		}
		char sendBuf[20] = { '\0' };
		printf("���ܵ�һ�����ӣ�%s \r\n", inet_ntop(AF_INET, (void*)&remoteAddr.sin_addr, sendBuf, 16));
		//���ݽ���
		int ret = recv(sClient, revData, 255, 0);
		if (ret)
		{
			PositionControllerPtr->isSocketTalk = true;
			revData[ret] = 0x00;
			//printf(revData);

			//����һ���յ��İ���get�����������
			//�����������command_long��ָ��,�������hawk;
			//��get���Ͱ�,�������ݸ��ͻ���
			switch (revData[0])
				//revData[0]Ϊ�Լ������socketmsg_id
			{	
			case SOCKETMSG_ID_ARM_DISARM:
			{
				const char *sendData = "ARM package received�� \n";
				send(sClient, sendData, strlen(sendData), 0);
				//SOCKETMSG_ID_ARM_DISARM �� revData[1] ��־�Ž���������
				command_long_param_arm_disarm[0] = revData[1];
				SerialPortPtrHAWK->mavlink_msg_command_long_pack(255, 190, &msg, SYSID_THISCOPTER, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, command_long_param_arm_disarm);
				break;
			}

			case SOCKETMSG_ID_TAKEOFF:
			{
				const char *sendData = "TAKEOFF package received�� \n";
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
				const char *sendData = "LAND package received�� \n";
				send(sClient, sendData, strlen(sendData), 0);
				SerialPortPtrHAWK->mavlink_msg_set_mode_pack(255, 190, &msg, 9, SYSID_THISCOPTER, 1);// switch to land mode
				break;
			}

			case SOCKETMSG_ID_SET_VELOCITY:
			{
				const char *sendData = "VELOCITY package received�� \n";
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
				const char *sendData = "SET_GLOBAL_POS package received�� \n";
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
				const char *sendData = "HOVER package received�� \n";
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
				const char *sendData = "SET_HEADING package received�� \n";
				send(sClient, sendData, strlen(sendData), 0);
				ch_d Yaw;
				for (int i = 0; i < nDouble; i++)
				{
					Yaw.ch[i] = revData[i + 1];
				}
				command_long_param_yaw[0] = Yaw.dd; //Ŀ��Ƕ�: [0��360��, 0 Ϊ������90Ϊ�������Դ�����
				command_long_param_yaw[1] = 20; //ת�٣��� / s
				command_long_param_yaw[2] = 1; //ת������ - 1λ˳ʱ�룬1λ��ʱ��
				command_long_param_yaw[3] = 0; //0Ϊ���ԽǶȣ�1Ϊ��ԽǶ�
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
		closesocket(sClient);  //�ر��ѽ�ͨ���׽���
		//cout << "���߳���" << endl;
	}
	closesocket(slisten); //�رռ������׽���
	WSACleanup();
	return 0;
}

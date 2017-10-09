#include "controlPosition.h"
#include "processMavlink.h"
#include <math.h>
#include <conio.h>
#include <fstream>
#include <mutex>

#define M_PI 3.14159265358979323846
#define EARTH_RADIUS 6378.137

using namespace std;

bool CPositionController::s_controlExit = false;
CMavLinkProcessor* CPositionController::SerialPortPtrHAWK;
CMavLinkProcessor* CPositionController::SerialPortPtrGCS;
CMavLinkProcessor* CPositionController::SerialPortPtr3;
Mode* CPositionController::ControlModePtr;



CPositionController::CPositionController(CMavLinkProcessor* CMavLinkProcessorPtr1,Mode* ModePtr): m_controlThread(INVALID_HANDLE_VALUE) 
{
	m_controlThread = INVALID_HANDLE_VALUE;
	SerialPortPtrHAWK = CMavLinkProcessorPtr1;
	ControlModePtr = ModePtr;

}

CPositionController::CPositionController(CMavLinkProcessor* CMavLinkProcessorPtr1, CMavLinkProcessor* CMavLinkProcessorPtr2, Mode* ModePtr) : m_controlThread(INVALID_HANDLE_VALUE)
{
	m_controlThread = INVALID_HANDLE_VALUE;
	SerialPortPtrHAWK = CMavLinkProcessorPtr1;
	SerialPortPtrGCS = CMavLinkProcessorPtr2;
	ControlModePtr = ModePtr;

}

CPositionController::CPositionController(CMavLinkProcessor* CMavLinkProcessorPtr1, CMavLinkProcessor* CMavLinkProcessorPtr2, CMavLinkProcessor* CMavLinkProcessorPtr3, Mode* ModePtr) : m_controlThread(INVALID_HANDLE_VALUE)
{
	m_controlThread = INVALID_HANDLE_VALUE;
	SerialPortPtrHAWK = CMavLinkProcessorPtr1;
	SerialPortPtrGCS = CMavLinkProcessorPtr2;
	SerialPortPtr3 = CMavLinkProcessorPtr3;
	ControlModePtr = ModePtr;

}

CPositionController::~CPositionController()
{
	CloseControllerTread();  
}

bool CPositionController::OpenControllerThread()  
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
	m_controlThread = (HANDLE)_beginthreadex(NULL, 0, ControllerThread, this, 0, &controlThreadId);  
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

bool CPositionController::CloseControllerTread()  
{     
	if (m_controlThread != INVALID_HANDLE_VALUE)  
	{  
		/** ֪ͨ�߳��˳� */   
		s_controlExit = true;  

		/** �ȴ��߳��˳� */   
		Sleep(10);  

		/** ���߳̾����Ч */   
		CloseHandle( m_controlThread );  
		m_controlThread = INVALID_HANDLE_VALUE;  
	}  
	return true;  
} 

UINT WINAPI CPositionController::ControllerThread( void* pParam)
{  
	/** �õ������ָ�� */   
	CPositionController *PositionControllerPtr = reinterpret_cast<CPositionController*>(pParam);  
	int GlobalPos[GLOBALPOS_DIM] = { 0 };
	int SetMode[SETMODE_DIM] = { 0 };
	int CommandLong[COMMANDLONG_DIM] = { 0 };

	// �߳�ѭ��,��ѯ��ʽ��ȡ��������  
	while (true)   
	{  

		Sleep(100);

		
		switch (*ControlModePtr)
		{
			Sleep(100);

		case CONTROL_NOTHING:
			break;


		case DELIVER_COPTER_INFO_TO_GCS:

			SerialPortPtrGCS->deliverMessage(SerialPortPtrHAWK->MavlinkMessage_33, PAYLOAD_LENGTH_33 + 6, SerialPortPtrHAWK->MavlinkMessage_33_crc, 2);

			break;


		case DELIVER_GCS_COMMAND_TO_HAWK:

			if (SerialPortPtrGCS->GlobalPos_flag)
			{

				SerialPortPtrHAWK->deliverMessage(SerialPortPtrGCS->MavlinkMessage_GlobalPos, PAYLOAD_LENGTH_GLOBALPOS + 6, SerialPortPtrGCS->MavlinkMessage_GlobalPos_crc, 2);
					
			}
			if (SerialPortPtrGCS->CommandLong_flag)
			{

				SerialPortPtrHAWK->deliverMessage(SerialPortPtrGCS->MavlinkMessage_CommandLong, PAYLOAD_LENGTH_COMMANDLONG + 6, SerialPortPtrGCS->MavlinkMessage_CommandLong_crc, 2);
			}
			if (SerialPortPtrGCS->SetMode_flag)
			{

				SerialPortPtrHAWK->deliverMessage(SerialPortPtrGCS->MavlinkMessage_SetMode, PAYLOAD_LENGTH_SETMODE + 6, SerialPortPtrGCS->MavlinkMessage_SetMode_crc, 2);
			}
			SerialPortPtrGCS->GlobalPos_flag = 0;
			SerialPortPtrGCS->CommandLong_flag = 0;
			SerialPortPtrGCS->SetMode_flag = 0;
			SerialPortPtrGCS->isGCStalk = 0;
			*ControlModePtr = DELIVER_COPTER_INFO_TO_GCS;

			break;

			
		default:
			//cout<<"�ȴ�����"<<endl;
			break;
		}
		//cout << "���߳���" << endl;
	}  

	return 0;  
}  



void CPositionController::TransferAttitude (int Attitude[ATTITUDE_DIM], double Attitude_trans[ATTITUDE_DIM])
{

	Attitude_trans[2] = (double)Attitude[2]/10000000;
	Attitude_trans[3] = (double)Attitude[3]/10000000;
	Attitude_trans[0] = (double)Attitude[0];
	Attitude_trans[1] = (double)Attitude[1];
	Attitude_trans[4] = (double)Attitude[4] / 1000;
	Attitude_trans[5] = (double)Attitude[5] / 1000;
	Attitude_trans[6] = (double)Attitude[6] / 100;
	Attitude_trans[7] = (double)Attitude[7] / 100;
	Attitude_trans[8] = (double)Attitude[8] / 100;
	//Attitude_trans[9] = (double)Attitude[9] * M_PI / (100 * 180);	//����
	Attitude_trans[9] = (double)Attitude[9] / 100 ;	//�Ƕȣ�0.0-359.9������Ϊ180������Ϊ270

}

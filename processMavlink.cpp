#include "SerialPort.h"  
#include "processMavlink.h"
#include <process.h>  
#include <iostream>  
#include <tchar.h>
#include <mutex>

using namespace std;

/** 线程退出标志 */   
bool CMavLinkProcessor::s_bExit = false;  
/** 当串口无数据时,sleep至下次查询间隔的时间,单位:秒 */   
const UINT SLEEP_TIME_INTERVAL = 5;  
uint8_t CMavLinkProcessor::mavlink_sequence_number = 0;

char CMavLinkProcessor::Node_addr[UAV_NUM_G1][DATA_LENGTH_NODE_ADDR] = { 0 };
char CMavLinkProcessor::Frame_header_68[UAV_NUM_G1][8] = { 0 };
char CMavLinkProcessor::Frame_header_69[UAV_NUM_G1][24] = { 0 };


CMavLinkProcessor::CMavLinkProcessor(void)  
: m_hListenThread(INVALID_HANDLE_VALUE)  
{  
    m_hComm = INVALID_HANDLE_VALUE;  
    m_hListenThread = INVALID_HANDLE_VALUE;  
 
    InitializeCriticalSection(&m_csCommunicationSync);  

	//==========begin write====================
	time_delay = 0.0;
	last_timestamp = 0;
	alpha = 0.0;
	fh = 50;
	fs = 100;
	pos_NED_x = 0.0;
	pos_NED_y = 0.0;
	pos_NED_z = 0.0;
	pos_NED_Vx = 0.0;
	pos_NED_Vy = 0.0;
	pos_NED_Vz = 0.0;
	time_stamp_prev = 0.0;
	//=========end write===============
 
}  
 
CMavLinkProcessor::~CMavLinkProcessor(void)  
{  
    CloseListenTread();  
    ClosePort();  
    DeleteCriticalSection(&m_csCommunicationSync);  
}  
 
bool CMavLinkProcessor::InitPort( UINT portNo /*= 1*/,UINT baud /*= CBR_9600*/,char parity /*= 'N'*/,  
                            UINT databits /*= 8*/, UINT stopsbits /*= 1*/,DWORD dwCommEvents /*= EV_RXCHAR*/ )  
{  
 
    /** 临时变量,将制定参数转化为字符串形式,以构造DCB结构 */   
    char szDCBparam[50];  
    sprintf_s(szDCBparam, "baud=%d parity=%c data=%d stop=%d", baud, parity, databits, stopsbits);  
 
    /** 打开指定串口,该函数内部已经有临界区保护,上面请不要加保护 */   
    if (!openPort(portNo))  
    {  
        return false;  
    }  
 
    /** 进入临界段 */   
    EnterCriticalSection(&m_csCommunicationSync);  
 
    /** 是否有错误发生 */   
    BOOL bIsSuccess = TRUE;  
 
    /** 在此可以设置输入输出的缓冲区大小,如果不设置,则系统会设置默认值.  
     *  自己设置缓冲区大小时,要注意设置稍大一些,避免缓冲区溢出  
     */ 
    /*if (bIsSuccess )  
    {  
        bIsSuccess = SetupComm(m_hComm,10,10);  
    }*/ 
 
    /** 设置串口的超时时间,均设为0,表示不使用超时限制 */ 
    COMMTIMEOUTS  CommTimeouts;  
    CommTimeouts.ReadIntervalTimeout         = 0;  
    CommTimeouts.ReadTotalTimeoutMultiplier  = 0;  
    CommTimeouts.ReadTotalTimeoutConstant    = 0;  
    CommTimeouts.WriteTotalTimeoutMultiplier = 0;  
    CommTimeouts.WriteTotalTimeoutConstant   = 0;   
    if ( bIsSuccess)  
    {  
        bIsSuccess = SetCommTimeouts(m_hComm, &CommTimeouts);  
    }  
 
    DCB  dcb;  
    if ( bIsSuccess )  
    {  
        // 将ANSI字符串转换为UNICODE字符串  
        DWORD dwNum = MultiByteToWideChar (CP_ACP, 0, szDCBparam, -1, NULL, 0);  
        wchar_t *pwText = new wchar_t[dwNum] ;  
        if (!MultiByteToWideChar (CP_ACP, 0, szDCBparam, -1, pwText, dwNum))  
        {  
            bIsSuccess = TRUE;  
        }  

        /** 获取当前串口配置参数,并且构造串口DCB参数 */    
		bIsSuccess = GetCommState(m_hComm, &dcb) && BuildCommDCB(pwText, &dcb) ;//由于字符集问题，此句做过修改
		//bIsSuccess = GetCommState(m_hComm, &dcb) && BuildCommDCB((LPCSTR)pwText, &dcb) ;
        /** 开启RTS flow控制 */   
        dcb.fRtsControl = RTS_CONTROL_ENABLE;   
 
        /** 释放内存空间 */   
        delete [] pwText;  
    }  
 
    if ( bIsSuccess )  
    {  
        /** 使用DCB参数配置串口状态 */   
        bIsSuccess = SetCommState(m_hComm, &dcb);  
    }  
          
    /**  清空串口缓冲区 */ 
    PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);  
 
    /** 离开临界段 */   
    LeaveCriticalSection(&m_csCommunicationSync);  
 
    return bIsSuccess==TRUE;  
}  
 
bool CMavLinkProcessor::InitPort( UINT portNo ,const LPDCB& plDCB )  
{  
    /** 打开指定串口,该函数内部已经有临界区保护,上面请不要加保护 */   
    if (!openPort(portNo))  
    {  
        return false;  
    }  
      
    /** 进入临界段 */   
    EnterCriticalSection(&m_csCommunicationSync);  
 
    /** 配置串口参数 */   
    if (!SetCommState(m_hComm, plDCB))  
    {  
        return false;  
    }  
 
    /**  清空串口缓冲区 */ 
    PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);  
 
    /** 离开临界段 */   
    LeaveCriticalSection(&m_csCommunicationSync);  
 
    return true;  
}  
 
void CMavLinkProcessor::ClosePort()  
{  
    /** 如果有串口被打开，关闭它 */ 
    if( m_hComm != INVALID_HANDLE_VALUE )  
    {  
        CloseHandle( m_hComm );  
        m_hComm = INVALID_HANDLE_VALUE;  
    }  
}  
 
bool CMavLinkProcessor::openPort( UINT portNo )  
{  
    /** 进入临界段 */   
    EnterCriticalSection(&m_csCommunicationSync);  
 
    /** 把串口的编号转换为设备名 */   
    char szPort[50];  
    sprintf_s(szPort, "COM%d", portNo);  
 
    /** 打开指定的串口 */   
    m_hComm = CreateFileA(szPort,  /** 设备名,COM1,COM2等 */   
              GENERIC_READ | GENERIC_WRITE, /** 访问模式,可同时读写 */     
              0,                            /** 共享模式,0表示不共享 */   
              NULL,                         /** 安全性设置,一般使用NULL */   
              OPEN_EXISTING,                /** 该参数表示设备必须存在,否则创建失败 */   
              FILE_ATTRIBUTE_SYSTEM | FILE_FLAG_WRITE_THROUGH,      
              0);      
 
    /** 如果打开失败，释放资源并返回 */   
    if (m_hComm == INVALID_HANDLE_VALUE)  
    {  
        LeaveCriticalSection(&m_csCommunicationSync);  
        return false;  
    }  
 
    /** 退出临界区 */   
    LeaveCriticalSection(&m_csCommunicationSync);  
 
    return true;  
}  
 
bool CMavLinkProcessor::OpenListenThread1()  
{  
    /** 检测线程是否已经开启了 */   
    if (m_hListenThread != INVALID_HANDLE_VALUE)  
    {  
        /** 线程已经开启 */   
        return false;  
    }  
 
    s_bExit = false;  
    /** 线程ID */   
    UINT threadId;  
    /** 开启串口数据监听线程 */   
    m_hListenThread = (HANDLE)_beginthreadex(NULL, 0, ListenThread1, this, 0, &threadId);  
    if (!m_hListenThread)  
    {  
        return false;  
    }  
    /** 设置线程的优先级,高于普通线程 */   
    if (!SetThreadPriority(m_hListenThread, THREAD_PRIORITY_ABOVE_NORMAL))  
    {  
        return false;  
    }  
 
    return true;  
}  

bool CMavLinkProcessor::OpenListenThread2()
{
	/** 检测线程是否已经开启了 */
	if (m_hListenThread != INVALID_HANDLE_VALUE)
	{
		/** 线程已经开启 */
		return false;
	}

	s_bExit = false;
	/** 线程ID */
	UINT threadId;
	/** 开启串口数据监听线程 */
	m_hListenThread = (HANDLE)_beginthreadex(NULL, 0, ListenThread2, this, 0, &threadId);
	if (!m_hListenThread)
	{
		return false;
	}
	/** 设置线程的优先级,高于普通线程 */
	if (!SetThreadPriority(m_hListenThread, THREAD_PRIORITY_ABOVE_NORMAL))
	{
		return false;
	}

	return true;
}


 
bool CMavLinkProcessor::CloseListenTread()  
{     
    if (m_hListenThread != INVALID_HANDLE_VALUE)  
    {  
        /** 通知线程退出 */   
        s_bExit = true;  
 
        /** 等待线程退出 */   
        Sleep(10);  
 
        /** 置线程句柄无效 */   
        CloseHandle( m_hListenThread );  
        m_hListenThread = INVALID_HANDLE_VALUE;  
    }  
    return true;  
}  
 
UINT CMavLinkProcessor::GetBytesInCOM()  
{  
    DWORD dwError = 0;  /** 错误码 */   
    COMSTAT  comstat;   /** COMSTAT结构体,记录通信设备的状态信息 */   
    memset(&comstat, 0, sizeof(COMSTAT));  
 
    UINT BytesInQue = 0;  
    /** 在调用ReadFile和WriteFile之前,通过本函数清除以前遗留的错误标志 */   
    if ( ClearCommError(m_hComm, &dwError, &comstat) )  
    {  
        BytesInQue = comstat.cbInQue; /** 获取在输入缓冲区中的字节数 */   
    }  
 
    return BytesInQue;  
}  
 
/*
//robosence telem
UINT WINAPI CMavLinkProcessor::ListenThread1( void* pParam )  
{  
    // 得到本类的指针 
    CMavLinkProcessor *pSerialPort = reinterpret_cast<CMavLinkProcessor*>(pParam);  
	int CurrentByte = 0;
	bool IsGlobalPos = false, IsHeartbeat = false, IsInPacket = false;
	uint8_t Payload_Length=0;
	uint8_t *TempPtr;
	uint8_t CurrentSys = 0;
	uint8_t *TempPtr2;
 
	//---------------for frame 69--------------//
	uint8_t Header_69;
	uint8_t Reserved_69;
	uint16_t Flow_Control_69;
	uint16_t Gateway_Type_69;
	struct IP_Addr_69
	{
		uint8_t A0;
		uint8_t A1;
		uint8_t A2;
		uint8_t A3;
	} Gateway_Addr_69;
	struct IP_Addr_69 Server_Addr_69;
	struct MAC_Addr_69
	{
		uint8_t M0;
		uint8_t M1;
		uint8_t M2;
		uint8_t M3;
		uint8_t M4;
		uint8_t M5;
	} MAC_69;
	uint16_t Frame_Serial_Num_69;
	uint16_t Data_length_69 = 1;
	uint8_t Data_69[255] = { 0 };
	//Data_69 = (uint8_t*)malloc(255);
	uint8_t CRC_69 = 0;
	uint8_t CRC_check_69;
	//---------------for frame 69--------------//


	//---------------for frame 68--------------//
	uint8_t Header_68;
	uint8_t Flow_Control_68;
	struct IP_Addr_68
	{
		uint8_t A0;
		uint8_t A1;
	} Node_Addr_68;
	struct IP_Addr_68 Hub_Addr_68;
	uint8_t Frame_Serial_Num_68;
	uint8_t Data_length_68 = 1;
	uint8_t Data_68[255] = { 0 };
	//Data_68 = (uint8_t*)malloc(200);//能不能不要固定分配，而是根据Length分配
	uint8_t CRC_68 = 0;
	uint8_t CRC_check_68;
	//---------------for frame 68--------------//


    // 线程循环,轮询方式读取串口数据  
    while (!pSerialPort->s_bExit)   
    {  
        UINT BytesInQue = pSerialPort->GetBytesInCOM();  
        // 如果串口输入缓冲区中无数据,则休息一会再查询 

        if ( BytesInQue == 0 )  
        {  
            Sleep(SLEEP_TIME_INTERVAL); 
            continue;  
        }  
 
        // 读取输入缓冲区中的数据并输出显示 
        char cRecved = 0x00;  
		
        do 
        {  
            cRecved = 0x00;  
			if (pSerialPort->ReadChar(cRecved))
			{
				
				//printf("%02X ", cRecved);
				if ((unsigned char)cRecved == 0x69 && !IsInPacket)
				{
					CurrentByte = 1;
					CRC_69 = 0;
					CRC_69 += (uint8_t)cRecved;
					IsInPacket = true;
					Header_69 = (uint8_t)cRecved;
				}
				if (CurrentByte == 2 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Reserved_69 = (uint8_t)cRecved;
				}
				if (CurrentByte == 3 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Flow_Control_69 = (uint16_t)cRecved << 8;
					//printf("cRecved:%02X \n", cRecved);
					//printf("FL1:%04X \n", Flow_Control_69);
				}
				if (CurrentByte == 4 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Flow_Control_69 |= (uint8_t)cRecved;
					//printf("cRecved:%02X \n", cRecved);
					//printf("FL1:%04X \n", Flow_Control_69);
				}
				if (CurrentByte == 5 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Gateway_Type_69 = (uint16_t)cRecved << 8;
					//printf("cRecved:%02X \n", cRecved);
					//printf("Re1:%04X \n", Gateway_Type_69);
				}
				if (CurrentByte == 6 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					//printf("cRecved:%02X \n", (uint8_t)cRecved);
					Gateway_Type_69 |= (uint8_t)cRecved;
					//printf("cRecved:%02X \n", (uint8_t)cRecved);
					//printf("Re2:%04X \n", Gateway_Type_69);
				}
				if (CurrentByte == 7 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Gateway_Addr_69.A3 = (uint8_t)cRecved;
				}
				if (CurrentByte == 8 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Gateway_Addr_69.A2 = (uint8_t)cRecved;
				}
				if (CurrentByte == 9 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Gateway_Addr_69.A1 = (uint8_t)cRecved;
				}
				if (CurrentByte == 10 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Gateway_Addr_69.A0 = (uint8_t)cRecved;
				}
				if (CurrentByte == 11 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Server_Addr_69.A3 = (uint8_t)cRecved;
				}
				if (CurrentByte == 12 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Server_Addr_69.A2 = (uint8_t)cRecved;
				}
				if (CurrentByte == 13 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Server_Addr_69.A1 = (uint8_t)cRecved;
				}
				if (CurrentByte == 14 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Server_Addr_69.A0 = (uint8_t)cRecved;
				}
				if (CurrentByte == 15 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					MAC_69.M5 = (uint8_t)cRecved;
				}
				if (CurrentByte == 16 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					MAC_69.M4 = (uint8_t)cRecved;
				}
				if (CurrentByte == 17 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					MAC_69.M3 = (uint8_t)cRecved;
				}
				if (CurrentByte == 18 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					MAC_69.M2 = (uint8_t)cRecved;
				}
				if (CurrentByte == 19 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					MAC_69.M1 = (uint8_t)cRecved;
				}
				if (CurrentByte == 20 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					MAC_69.M0 = (uint8_t)cRecved;
				}
				if (CurrentByte == 21 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Frame_Serial_Num_69 = (uint16_t)cRecved << 8;
				}
				if (CurrentByte == 22 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Frame_Serial_Num_69 |= (uint16_t)cRecved;
				}
				if (CurrentByte == 23 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Data_length_69 = (uint16_t)cRecved << 8;
				}
				if (CurrentByte == 24 && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Data_length_69 |= (uint16_t)cRecved;
					//Data_69 = (uint8_t*)malloc(255);
				}
				if (CurrentByte >= 25 && CurrentByte <= (24+Data_length_69) && IsInPacket)
				{
					CRC_69 += (uint8_t)cRecved;
					Data_69[CurrentByte-25] = (uint8_t)cRecved;
				}
				if (CurrentByte == (25 + Data_length_69) && IsInPacket)
				{
					CRC_check_69 = (uint8_t)cRecved;
					if (CRC_69 == CRC_check_69)
					{
						for (int i = 0; i < (Data_length_69-2-8); i++)//-8是68帧的前8 Byte，-2是68帧的校验码和帧尾
						{
							Data_68[i] = Data_69[i + 8];//move Mavlink frames to Data_68
						}
						//cout << "\nCorrect" << endl;
						

						//match copter's sysid and radio's node address 
						if (pSerialPort->Frame_header_68[Data_68[3] - 1][0] == 0)
						{
							pSerialPort->Node_addr[Data_68[3] - 1][0] = Data_69[2];
							pSerialPort->Node_addr[Data_68[3] - 1][1] = Data_69[3];
							pSerialPort->Frame_header_68[Data_68[3] - 1][0] = Data_69[0];
							pSerialPort->Frame_header_68[Data_68[3] - 1][1] = Data_69[1];
							pSerialPort->Frame_header_68[Data_68[3] - 1][2] = Data_69[2];
							pSerialPort->Frame_header_68[Data_68[3] - 1][3] = Data_69[3];
							pSerialPort->Frame_header_68[Data_68[3] - 1][4] = Data_69[4];
							pSerialPort->Frame_header_68[Data_68[3] - 1][5] = Data_69[5];
							pSerialPort->Frame_header_68[Data_68[3] - 1][6] = Data_69[6];
							pSerialPort->Frame_header_68[Data_68[3] - 1][7] = Data_69[7];

							pSerialPort->Frame_header_69[Data_68[3] - 1][0] = Header_69;
							pSerialPort->Frame_header_69[Data_68[3] - 1][1] = Reserved_69;
							pSerialPort->Frame_header_69[Data_68[3] - 1][2] = Flow_Control_69 >> 8;	//高8位
							pSerialPort->Frame_header_69[Data_68[3] - 1][3] = Flow_Control_69 & 0x00ff;	//低8位
							pSerialPort->Frame_header_69[Data_68[3] - 1][4] = Gateway_Type_69 >> 8;
							//printf("Save：%04X \n", Gateway_Type_69);
							pSerialPort->Frame_header_69[Data_68[3] - 1][5] = Gateway_Type_69 & 0x00ff;
							pSerialPort->Frame_header_69[Data_68[3] - 1][6] = Gateway_Addr_69.A3;
							pSerialPort->Frame_header_69[Data_68[3] - 1][7] = Gateway_Addr_69.A2;
							pSerialPort->Frame_header_69[Data_68[3] - 1][8] = Gateway_Addr_69.A1;
							pSerialPort->Frame_header_69[Data_68[3] - 1][9] = Gateway_Addr_69.A0;
							pSerialPort->Frame_header_69[Data_68[3] - 1][10] = Server_Addr_69.A3;
							pSerialPort->Frame_header_69[Data_68[3] - 1][11] = Server_Addr_69.A2;
							pSerialPort->Frame_header_69[Data_68[3] - 1][12] = Server_Addr_69.A1;
							pSerialPort->Frame_header_69[Data_68[3] - 1][13] = Server_Addr_69.A0;
							pSerialPort->Frame_header_69[Data_68[3] - 1][14] = MAC_69.M5;
							pSerialPort->Frame_header_69[Data_68[3] - 1][15] = MAC_69.M4;
							pSerialPort->Frame_header_69[Data_68[3] - 1][16] = MAC_69.M3;
							pSerialPort->Frame_header_69[Data_68[3] - 1][17] = MAC_69.M2;
							pSerialPort->Frame_header_69[Data_68[3] - 1][18] = MAC_69.M1;
							pSerialPort->Frame_header_69[Data_68[3] - 1][19] = MAC_69.M0;
							pSerialPort->Frame_header_69[Data_68[3] - 1][20] = Frame_Serial_Num_69 >> 8;
							pSerialPort->Frame_header_69[Data_68[3] - 1][21] = Frame_Serial_Num_69 & 0x00ff;
							pSerialPort->Frame_header_69[Data_68[3] - 1][22] = Data_length_69 >> 8;
							pSerialPort->Frame_header_69[Data_68[3] - 1][23] = Data_length_69 & 0x00ff;
						}


						if (Data_68[0] == 0xfe)//如果是mavlink包
						{
							int k = 0;
							do   //将收到的在一起的很多个mavlink包拆成一个个的，取出其中的GPS包
							{
								if (Data_68[k] == 0xfe && Data_68[k + 5] == 0x00)//0x00为心跳包，总共17字节，数据域9字节
								{

									k += 17;
								}
								if (Data_68[k] == 0xfe && Data_68[k + 5] == 0x21)//0x21为GPS包，总共36字节，数据域28字节
								{
									CurrentSys = Data_68[k + 3];
									Payload_Length = Data_68[k + 1];
									for (int i = 0; i < 28; i++)
									{
										pSerialPort->Payload_store_33_g1[CurrentSys - 1][i] = Data_68[k + 6 + i];
									}
									k += 36;
								}
							} while (k < Data_length_69 - 2 - 8);
						}
						

					}
					else//丢弃这个包
					{
						//cout << "Abandon" << endl;
						IsInPacket = false;
						CurrentByte = 0;
					}
				}
				if (CurrentByte == (26 + Data_length_69) && IsInPacket)
				{
					if ((unsigned char)cRecved == 0x17)
					{
						IsInPacket = false;
						CurrentByte = 0;
					}
				}

				CurrentByte++;
                continue;  
				
            }  
        }while(--BytesInQue); 
		
		if (!IsInPacket)
			CurrentByte = 0;
    }  
 
    return 0;  
}  
*/




/*
//xbee
//not use checksum
//receive mavlink message from hawk, only 0x21 position message
//put the data received in serial port 1 to variables Payload_store_33[][]
UINT WINAPI CMavLinkProcessor::ListenThread1(void* pParam)
{
	// 得到本类的指针 
	CMavLinkProcessor *pSerialPort = reinterpret_cast<CMavLinkProcessor*>(pParam);
	int CurrentByte = 0;
	bool IsGlobalPos = false, IsHeartbeat = false, IsInPacket = false;
	uint8_t Payload_Length = 0;
	uint8_t *TempPtr;
	uint8_t CurrentSys = 0;
	uint8_t *TempPtr2;

	// 线程循环,轮询方式读取串口数据  
	while (!pSerialPort->s_bExit)
	{

		UINT BytesInQue = pSerialPort->GetBytesInCOM();
		// 如果串口输入缓冲区中无数据,则休息一会再查询
		if (BytesInQue == 0)
		{
			Sleep(SLEEP_TIME_INTERVAL);
			continue;
		}

		// 读取输入缓冲区中的数据并输出显示
		char cRecved = 0x00;

		do
		{
			cRecved = 0x00;
			if (pSerialPort->ReadChar(cRecved) == true)
			{

				//cout<<sizeof(cRecved);
				//std::cout << hex << cRecved ;  
				if ((unsigned char)cRecved == 0xfe && !IsInPacket)
				{
					//cout << endl;
					CurrentByte = 1;
					IsInPacket = true;
				}

				if (CurrentByte == MAVLINK_LENGTH_BYTE)
				{
					TempPtr = reinterpret_cast<uint8_t*>(&cRecved);
					Payload_Length = *TempPtr;

				}

				if (CurrentByte == MAVLINK_SYS_BYTE)
				{
					TempPtr2 = reinterpret_cast<uint8_t*>(&cRecved);
					CurrentSys = *TempPtr2;

				}

				if (CurrentByte == MAVLINK_HEADER_LENGTH)
				{

					if ((unsigned char)cRecved == 0x21)
					{
						//cout <<"global pos ";
						IsGlobalPos = true;
					}
					else
					{
						//cout << "heartbeat";
						IsGlobalPos = false;
					}

				}

				if (CurrentByte <= MAVLINK_HEADER_LENGTH + Payload_Length && CurrentByte > MAVLINK_HEADER_LENGTH && IsGlobalPos == true)
				{
					pSerialPort->Payload_store_33[CurrentSys - 1][CurrentByte - MAVLINK_HEADER_LENGTH - 1] = cRecved;
				}

				if (CurrentByte > MAVLINK_HEADER_LENGTH + Payload_Length + 1)
				{
					IsInPacket = false;
					IsGlobalPos = false;
				}

				//printf("%02x ", (unsigned char)cRecved);
				CurrentByte++;
				continue;

			}
		} while (--BytesInQue);
	}

	return 0;
}
*/



//xbee
//use checksum
//receive mavlink message from hawk, only 0x21 position message is sent
//put the data received in serial port 1 to variables Payload_store_33[][]
UINT WINAPI CMavLinkProcessor::ListenThread1(void* pParam)
{
	// 得到本类的指针 
	CMavLinkProcessor *pSerialPort = reinterpret_cast<CMavLinkProcessor*>(pParam);
	int CurrentByte = 0;
	bool IsInPacket = false, IsGlobalInfo = false, IsHeartbeat = false;


	// 线程循环,轮询方式读取串口数据  
	while (!pSerialPort->s_bExit)
	{
		UINT BytesInQue = pSerialPort->GetBytesInCOM();
		// 如果串口输入缓冲区中无数据,则休息一会再查询
		if (BytesInQue == 0)
		{
			Sleep(SLEEP_TIME_INTERVAL);
			continue;
		}

		// 读取输入缓冲区中的数据并输出显示
		char cRecved = 0x00;

		do
		{
			cRecved = 0x00;
			if (pSerialPort->ReadChar(cRecved) == true)
			{

				//cout<<sizeof(cRecved);
				//std::cout << hex << cRecved ;  
				if ((unsigned char)cRecved == 0xfe && !IsInPacket)
				{
					//cout << endl;
					CurrentByte = 1;
					IsInPacket = true;
					pSerialPort->MavlinkMessage_33[0] = cRecved;
					
				}

				if (CurrentByte == 2)
				{
					if (cRecved == 0x1C)	//#33
					{
						pSerialPort->MavlinkMessage_33[CurrentByte - 1] = cRecved;
						IsGlobalInfo = true;
					}else
					if (cRecved == 0x09)	//heartbeat
					{
						IsHeartbeat = true;
					}

				}

				if (CurrentByte > 2 && CurrentByte <= PAYLOAD_LENGTH_33 + 6 && IsGlobalInfo)
				{
					pSerialPort->MavlinkMessage_33[CurrentByte - 1] = cRecved;

				}

				if (CurrentByte > PAYLOAD_LENGTH_33 + 6 && CurrentByte <= PAYLOAD_LENGTH_33 + 8 && IsGlobalInfo)
				{
					pSerialPort->MavlinkMessage_33_crc[CurrentByte - PAYLOAD_LENGTH_33 - MAVLINK_HEADER_LENGTH - 1] = cRecved;

				}

				if (CurrentByte > PAYLOAD_LENGTH_33 + 8 || IsHeartbeat)
				{
					IsInPacket = false;
					IsGlobalInfo = false;
					IsHeartbeat = false;
					CurrentByte = 0;

				}

				//printf("%02x ", (unsigned char)cRecved);
				CurrentByte++;
				continue;
			}
		} while (--BytesInQue);
	}

	return 0;
}



//receive mavlink command message from GCS
UINT WINAPI CMavLinkProcessor::ListenThread2(void* pParam)
{
	/** 得到本类的指针 */
	CMavLinkProcessor *pSerialPort = reinterpret_cast<CMavLinkProcessor*>(pParam);
	int CurrentByte = 0;
	bool IsGlobalPos = false, IsHeartbeat = false, IsInPacket = false, IsCommandLong = false, IsSetMode = false;
	uint8_t Payload_Length = 0;
	uint8_t *TempPtr;
	uint8_t CurrentSys = 0;
	uint8_t *TempPtr2;

	// 线程循环,轮询方式读取串口数据  
	while (!pSerialPort->s_bExit)
	{

		UINT BytesInQue = pSerialPort->GetBytesInCOM();
		/** 如果串口输入缓冲区中无数据,则休息一会再查询 */
		if (BytesInQue == 0)
		{
			Sleep(SLEEP_TIME_INTERVAL);
			continue;
		}

		/** 读取输入缓冲区中的数据并输出显示 */
		char cRecved = 0x00;

		do
		{
			cRecved = 0x00;
			if (pSerialPort->ReadChar(cRecved) == true)
			{

				//cout<<sizeof(cRecved);
				//std::cout << hex << cRecved ;  
				if ((unsigned char)cRecved == 0xfe && !IsInPacket)
				{
					//cout << endl;
					CurrentByte = 1;
					IsInPacket = true;
					pSerialPort->MavlinkMessage_GlobalPos[0] = cRecved;
					pSerialPort->MavlinkMessage_CommandLong[0] = cRecved;
					pSerialPort->MavlinkMessage_SetMode[0] = cRecved;
					CurrentByte++;
					continue;
				}

				if (CurrentByte == MAVLINK_LENGTH_BYTE)
				{
					TempPtr = reinterpret_cast<uint8_t*>(&cRecved);
					Payload_Length = *TempPtr;
					pSerialPort->MavlinkMessage_GlobalPos[1] = cRecved;
					pSerialPort->MavlinkMessage_CommandLong[1] = cRecved;
					pSerialPort->MavlinkMessage_SetMode[1] = cRecved;
					CurrentByte++;
					continue;
				}

				if (CurrentByte == 3)
				{
					pSerialPort->MavlinkMessage_GlobalPos[2] = cRecved;
					pSerialPort->MavlinkMessage_CommandLong[2] = cRecved;
					pSerialPort->MavlinkMessage_SetMode[2] = cRecved;
					CurrentByte++;
					continue;
				}

				if (CurrentByte == MAVLINK_SYS_BYTE)
				{
					pSerialPort->MavlinkMessage_GlobalPos[3] = cRecved;
					pSerialPort->MavlinkMessage_CommandLong[3] = cRecved;
					pSerialPort->MavlinkMessage_SetMode[3] = cRecved;
					CurrentByte++;
					continue;
				}

				if (CurrentByte == 5)
				{
					pSerialPort->MavlinkMessage_GlobalPos[4] = cRecved;
					pSerialPort->MavlinkMessage_CommandLong[4] = cRecved;
					pSerialPort->MavlinkMessage_SetMode[4] = cRecved;
					CurrentByte++;
					continue;
				}

				if (CurrentByte == MAVLINK_HEADER_LENGTH)
				{
					//SET_MODE ( #11 )
					if ((unsigned char)cRecved == 0xB)
					{
						IsSetMode = true;
						pSerialPort->SetMode_flag = 1;
						pSerialPort->MavlinkMessage_SetMode[5] = cRecved;
					}
					//COMMAND_LONG ( #76 )
					else if((unsigned char)cRecved == 0x4C)
					{
						IsCommandLong = true;
						pSerialPort->CommandLong_flag = 1;
						pSerialPort->MavlinkMessage_CommandLong[5] = cRecved;
					}
					//SET_POSITION_TARGET_GLOBAL_INT ( #86 )
					else if ((unsigned char)cRecved == 0x56)
					{
						IsGlobalPos = true;
						pSerialPort->GlobalPos_flag = 1;
						pSerialPort->MavlinkMessage_GlobalPos[5] = cRecved;
					}

					CurrentByte++;
					continue;
				}

				if (CurrentByte <= MAVLINK_HEADER_LENGTH + Payload_Length && CurrentByte > MAVLINK_HEADER_LENGTH)
				{
					if (IsGlobalPos == true)
					{
						pSerialPort->MavlinkMessage_GlobalPos[CurrentByte  - 1] = cRecved;
					}
					else if (IsCommandLong == true)
					{
						pSerialPort->MavlinkMessage_CommandLong[CurrentByte  - 1] = cRecved;
					}
					else if (IsSetMode == true)
					{
						pSerialPort->MavlinkMessage_SetMode[CurrentByte  - 1] = cRecved;
					}
					CurrentByte++;
					continue;
				}


				if (CurrentByte == MAVLINK_HEADER_LENGTH + Payload_Length + 1)
				{
					if (IsGlobalPos == true)
					{
						pSerialPort->MavlinkMessage_GlobalPos_crc[CurrentByte - Payload_Length - MAVLINK_HEADER_LENGTH - 1] = cRecved;
					}
					else if (IsCommandLong == true)
					{
						pSerialPort->MavlinkMessage_CommandLong_crc[CurrentByte - Payload_Length - MAVLINK_HEADER_LENGTH - 1] = cRecved;
					}
					else if (IsSetMode == true)
					{
						pSerialPort->MavlinkMessage_SetMode_crc[CurrentByte - Payload_Length - MAVLINK_HEADER_LENGTH - 1] = cRecved;
					}
					CurrentByte++;
					continue;
				}

				if (CurrentByte == MAVLINK_HEADER_LENGTH + Payload_Length + 2)
				{
					if (IsGlobalPos == true)
					{
						pSerialPort->MavlinkMessage_GlobalPos_crc[CurrentByte - Payload_Length - MAVLINK_HEADER_LENGTH - 1] = cRecved;
					}
					else if (IsCommandLong == true)
					{
						pSerialPort->MavlinkMessage_CommandLong_crc[CurrentByte - Payload_Length - MAVLINK_HEADER_LENGTH - 1] = cRecved;
					}
					else if (IsSetMode == true)
					{
						pSerialPort->MavlinkMessage_SetMode_crc[CurrentByte - Payload_Length - MAVLINK_HEADER_LENGTH - 1] = cRecved;
					}
					pSerialPort->isGCStalk = true;
					IsGlobalPos = false;
					IsHeartbeat = false;
					IsInPacket = false;
					IsCommandLong = false;
					IsSetMode = false;
					Payload_Length = 0;
					CurrentByte = 0;
					continue;
				}

				//printf("%02x ", (unsigned char)cRecved);


			}
		} while (--BytesInQue);
	}

	return 0;
}




bool CMavLinkProcessor::ReadChar( char &cRecved )  
{  
    BOOL  bResult     = TRUE;  
    DWORD BytesRead   = 0;  
    if(m_hComm == INVALID_HANDLE_VALUE)  
    {  
        return false;  
    }  
 
    /** 临界区保护 */   
    EnterCriticalSection(&m_csCommunicationSync);  
 
    /** 从缓冲区读取一个字节的数据 */   
    bResult = ReadFile(m_hComm, &cRecved, 1, &BytesRead, NULL);  
    if ((!bResult))  
    {   
        /** 获取错误码,可以根据该错误码查出错误原因 */   
        DWORD dwError = GetLastError();  
 
        /** 清空串口缓冲区 */   
        PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);  
        LeaveCriticalSection(&m_csCommunicationSync);  
 
        return false;  
    }  
 
    /** 离开临界区 */   
    LeaveCriticalSection(&m_csCommunicationSync);  
 
    return (BytesRead == 1);  
 
}  
 
bool CMavLinkProcessor::WriteData( unsigned char* pData, unsigned int length )  
{  
    BOOL   bResult     = TRUE;  
    DWORD  BytesToSend = 0;  
    if(m_hComm == INVALID_HANDLE_VALUE)  
    {  
		printf("write data failed !!!!!\n");
        return false;  
    }  
 
    /** 临界区保护 */   
    EnterCriticalSection(&m_csCommunicationSync);  
 
    /** 向缓冲区写入指定量的数据 */   
    bResult = WriteFile(m_hComm, pData, length, &BytesToSend, NULL);  
    if (!bResult)    
    {  
		printf("write data failed !!!!!\n");
        DWORD dwError = GetLastError();  
        /** 清空串口缓冲区 */   
        PurgeComm(m_hComm, PURGE_RXCLEAR | PURGE_RXABORT);  
        LeaveCriticalSection(&m_csCommunicationSync);  
        return false;  
    }  

    /** 离开临界区 */   
	//printf("write data finished !!!!!\n");
    LeaveCriticalSection(&m_csCommunicationSync);  
    return true;  
}

bool CMavLinkProcessor::WriteFloatData(float floatData, bool displayFloat, bool displayUnsigned)
{  
	bool flag;
	unsigned char buff[4];
	//float data;
	memcpy(buff, &floatData, 4);
	//memcpy(&data,buff,4);
	//float* floatDataPoint = &floatData;
	//unsigned char *unsignedDataPoint=(unsigned char *) floatDataPoint;

	flag = WriteData(buff,4);//将float分成四字节数据发送
	cout<<int(flag)<<endl;

	if (displayFloat)//显示发送数据的float表示
	{
		cout<<"十进制：";
		cout<<buff[0]<<endl;
	}

	if (displayUnsigned)//显示发送数据的16进制表示
	{
		cout<<"十六进制：";
		for (int i = 0;i <4 ;i++)
		{
			printf("%02x ",buff[i]);
		}
		cout<<endl;
	}

	if (flag)
	{
		return true;
	} 
	else
	{
		return false;
	}


}




bool CMavLinkProcessor::checkMessage(char* message,char* crc)
{
	uint8_t buf[2];
	uint16_t checksum = CRC_gengerate((uint8_t*)message);
	buf[0] = (unsigned char)checksum & 0xFF;
	buf[1] = checksum >> 8;
	if (buf[0] == crc[0] && buf[1] == crc[1])
	{
		return true;
	}
	else
	{
		return false;
	}
}



//parse MavlinkMessage_33[] to variables Attitude[]
void CMavLinkProcessor::parseCharStream (int Attitude[ATTITUDE_DIM])
{
	uint8_t Sysid;
	uint32_t Timestamp;
	int Lat, Lon, Alt, Relative_alt;
	int16_t Vx, Vy, Vz;
	uint16_t Yaw;

	while (1)
	{

		if (checkMessage(MavlinkMessage_33, MavlinkMessage_33_crc))
		{
			break;
		}
	}

		uint8_t *ft0= reinterpret_cast<uint8_t*>(&MavlinkMessage_33[3]);	//sysid
		Sysid = *ft0;
		Attitude[0] = Sysid;

		uint32_t *ft1 = reinterpret_cast<uint32_t*>(&MavlinkMessage_33[6]);			//Timestamp
		Timestamp = *ft1;
		Attitude[1] = Timestamp;

		int *ft2 = reinterpret_cast<int*>(&MavlinkMessage_33[10]);			//纬度
		Lat = *ft2;
		Attitude[2] = Lat;

		int *ft3 = reinterpret_cast<int*>(&MavlinkMessage_33[14]);	//经度
		Lon = *ft3;
		Attitude[3] = Lon;

		int *ft4 = reinterpret_cast<int*>(&MavlinkMessage_33[18]);  //绝对高度，GPS测得
		Alt = *ft4;
		Attitude[4] = Alt;

		int *ft5 = reinterpret_cast<int*>(&MavlinkMessage_33[22]);  //相对高度，与飞控通电气压计初始化时相比
		Relative_alt = *ft5;
		Attitude[5] = Relative_alt;

		int16_t *ft6 = reinterpret_cast<int16_t *>(&MavlinkMessage_33[26]);
		Vx = *ft6;
		Attitude[6] = Vx;

		int16_t *ft7 = reinterpret_cast<int16_t *>(&MavlinkMessage_33[28]);
		Vy = *ft7;
		Attitude[7] = Vy;

		int16_t *ft8 = reinterpret_cast<int16_t *>(&MavlinkMessage_33[30]);
		Vz = *ft8;
		Attitude[8] = Vz;

		uint16_t *ft9 = reinterpret_cast<uint16_t*>(&MavlinkMessage_33[32]);  //偏航
		Yaw = *ft9;
		Attitude[9] = Yaw;

		MavlinkMessage_33[0] = 0;	//每次parse之后置零，

}


//parse MavlinkMessage_SetMode[] to variables SetMode[]
void CMavLinkProcessor::parseSetMode(int SetMode[SETMODE_DIM])
{
	uint32_t custom_mode;
	uint8_t target_system, base_mode;

	while (1)
	{
		if (checkMessage(MavlinkMessage_SetMode, MavlinkMessage_SetMode_crc))
		{
			break;
		}
	}

		uint32_t *ft1 = reinterpret_cast<uint32_t*>(&MavlinkMessage_SetMode[6]);
		custom_mode = *ft1;
		SetMode[0] = custom_mode;

		uint8_t *ft2 = reinterpret_cast<uint8_t*>(&MavlinkMessage_SetMode[10]);	
		target_system = *ft2;
		SetMode[1] = target_system;

		uint8_t *ft3 = reinterpret_cast<uint8_t*>(&MavlinkMessage_SetMode[11]);	
		base_mode = *ft3;
		SetMode[2] = base_mode;
}



//parse Payload_store_CommandLong[] to variables CommandLong[]
void CMavLinkProcessor::parseCommandLongStream(int CommandLong[COMMANDLONG_DIM])
{

	float param1, param2, param3, param4, param5, param6, param7;
	uint16_t command;
	uint8_t target_system, target_component, confirmation;

	while (1)
	{
		if (checkMessage(MavlinkMessage_CommandLong, MavlinkMessage_CommandLong_crc))
		{
			break;
		}
	}

	float *ft1 = reinterpret_cast<float*>(&MavlinkMessage_CommandLong[6]);
	param1 = *ft1;
	CommandLong[0] = param1;

	float *ft2 = reinterpret_cast<float*>(&MavlinkMessage_CommandLong[10]);
	param2 = *ft2;
	CommandLong[1] = param2;

	float *ft3 = reinterpret_cast<float*>(&MavlinkMessage_CommandLong[14]);
	param3 = *ft3;
	CommandLong[2] = param3;

	float *ft4 = reinterpret_cast<float*>(&MavlinkMessage_CommandLong[18]);
	param4 = *ft4;
	CommandLong[3] = param4;

	float *ft5 = reinterpret_cast<float*>(&MavlinkMessage_CommandLong[22]);
	param5 = *ft5;
	CommandLong[4] = param5;

	float *ft6 = reinterpret_cast<float*>(&MavlinkMessage_CommandLong[26]);
	param6 = *ft6;
	CommandLong[5] = param6;

	float *ft7 = reinterpret_cast<float*>(&MavlinkMessage_CommandLong[30]);
	param7 = *ft7;
	CommandLong[6] = param7;

	uint16_t *ft8 = reinterpret_cast<uint16_t*>(&MavlinkMessage_CommandLong[34]);
	command = *ft8;
	CommandLong[7] = command;

	uint8_t *ft9 = reinterpret_cast<uint8_t*>(&MavlinkMessage_CommandLong[36]);
	target_system = *ft9;
	CommandLong[8] = target_system;

	uint8_t *ft10 = reinterpret_cast<uint8_t*>(&MavlinkMessage_CommandLong[37]);
	target_component = *ft10;
	CommandLong[9] = target_component;

	uint8_t *ft11 = reinterpret_cast<uint8_t*>(&MavlinkMessage_CommandLong[38]);
	confirmation = *ft11;
	CommandLong[10] = confirmation;
}


//parse MavlinkMessage_GlobalPos[] to variables GlobalPos[]
void CMavLinkProcessor::parseGlobalPos(int GlobalPos[GLOBALPOS_DIM])
{

	uint32_t time_boot_ms;
	int32_t lat_int, lon_int;
	float alt, vx, vy, vz, afx, afy, afz, yaw, yaw_rate;
	uint16_t type_mask;
	uint8_t target_system, target_component, coordinate_frame;

	while (1)
	{
		if (checkMessage(MavlinkMessage_GlobalPos, MavlinkMessage_GlobalPos_crc))
		{
			break;
		}
	}

	uint32_t *ft1 = reinterpret_cast<uint32_t*>(&MavlinkMessage_GlobalPos[6]);
	time_boot_ms = *ft1;
	GlobalPos[0] = time_boot_ms;

	int32_t *ft2 = reinterpret_cast<int32_t*>(&MavlinkMessage_GlobalPos[10]);
	lat_int = *ft2;
	GlobalPos[1] = lat_int;

	int32_t *ft3 = reinterpret_cast<int32_t*>(&MavlinkMessage_GlobalPos[14]);
	lon_int = *ft3;
	GlobalPos[2] = lon_int;

	float *ft4 = reinterpret_cast<float*>(&MavlinkMessage_GlobalPos[18]);
	alt = *ft4;
	GlobalPos[3] = alt;

	float *ft5 = reinterpret_cast<float*>(&MavlinkMessage_GlobalPos[22]);
	vx = *ft5;
	GlobalPos[4] = vx;

	float *ft6 = reinterpret_cast<float*>(&MavlinkMessage_GlobalPos[26]);
	vy = *ft6;
	GlobalPos[5] = vy;

	float *ft7 = reinterpret_cast<float*>(&MavlinkMessage_GlobalPos[30]);
	vz = *ft7;
	GlobalPos[6] = vz;

	float *ft8 = reinterpret_cast<float*>(&MavlinkMessage_GlobalPos[34]);
	afx = *ft8;
	GlobalPos[7] = afx;

	float *ft9 = reinterpret_cast<float*>(&MavlinkMessage_GlobalPos[38]);
	afy = *ft9;
	GlobalPos[8] = afy;

	float *ft10 = reinterpret_cast<float*>(&MavlinkMessage_GlobalPos[42]);
	afz = *ft10;
	GlobalPos[9] = afz;

	float *ft11 = reinterpret_cast<float*>(&MavlinkMessage_GlobalPos[46]);
	yaw = *ft11;
	GlobalPos[10] = yaw;

	float *ft12 = reinterpret_cast<float*>(&MavlinkMessage_GlobalPos[50]);
	yaw_rate = *ft12;
	GlobalPos[12] = yaw_rate;

	uint16_t *ft13 = reinterpret_cast<uint16_t*>(&MavlinkMessage_GlobalPos[54]);
	type_mask = *ft13;
	GlobalPos[12] = type_mask;

	uint8_t *ft14 = reinterpret_cast<uint8_t*>(&MavlinkMessage_GlobalPos[56]);
	target_system = *ft14;
	GlobalPos[13] = target_system;

	uint8_t *ft15 = reinterpret_cast<uint8_t*>(&MavlinkMessage_GlobalPos[57]);
	target_component = *ft15;
	GlobalPos[14] = target_component;

	uint8_t *ft16 = reinterpret_cast<uint8_t*>(&MavlinkMessage_GlobalPos[58]);
	coordinate_frame = *ft16;
	GlobalPos[15] = coordinate_frame;
}

//==================begin write=====================================================================================


void CMavLinkProcessor::crc_init(uint16_t* crcAccum)
{
	*crcAccum = X25_INIT_CRC;
}

void CMavLinkProcessor::crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
	uint8_t tmp;
	tmp = data ^ (uint8_t)(*crcAccum &0xff);
	tmp ^= (tmp<<4);
	*crcAccum = (*crcAccum>>8) ^ (tmp<<8) ^ (tmp <<3) ^ (tmp>>4);
}


uint16_t CMavLinkProcessor::crc_calculate(const uint8_t* pBuffer, uint16_t length)
{
	uint16_t crcTmp;
	crc_init(&crcTmp);
	while (length--) {
		crc_accumulate(*pBuffer++, &crcTmp);
	}
	return crcTmp;
}

void CMavLinkProcessor::crc_accumulate_buffer(uint16_t *crcAccum, const uint8_t *pBuffer, uint16_t length)
{
	const uint8_t *p = (const uint8_t *)pBuffer;
	while (length--) {
		crc_accumulate(*p++, crcAccum);
	}
}

void CMavLinkProcessor::_mavlink_send_uart(const uint8_t *buf, uint8_t length)
{
	/*
	for(int temp = 0; temp<length; temp++)
	{
		printf("%02X(%d) ",buf[temp], buf[temp]);
	}
	printf("\n");
	*/
	WriteData((unsigned char *)buf, length);
}

/*
//robosence
void CMavLinkProcessor::mavlink_finalize_message(
	mavlink_message_t* msg,
	uint8_t sysid,
	uint8_t compid,
	uint8_t length,
	uint8_t crc_extra,
	uint8_t target_system_id)
{
	uint16_t checksum;
	uint8_t buf[MAVLINK_NUM_HEADER_BYTES];
	uint8_t ck[2];
	buf[0] = MAVLINK_STX;
	buf[1] = length;
	buf[2] = 0;
	buf[3] = sysid;
	buf[4] = compid;
	buf[5] = msg->msgid;
	checksum = crc_calculate((const uint8_t*)&buf[1], MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&checksum, _MAV_PAYLOAD_NON_CONST(msg), length);

	crc_accumulate(crc_extra, &checksum);

	ck[0] = (uint8_t)(checksum & 0xFF);	//取checksum的低8位
	ck[1] = (uint8_t)(checksum >> 8);	//取checksum的高8位

	uint16_t DataLength_69;
	uint8_t data_length_69[2];
	DataLength_69 = 8 + 6 + length + 2 + 2;	//68帧帧头 + Mavlink帧帧头 + mavlink payload + mavlink校验 + 68帧校验和帧尾
	data_length_69[0] = DataLength_69 >> 8;
	data_length_69[1] = DataLength_69 & 0x00ff;

	uint8_t DataLength_68[1];
	DataLength_68[0] = 6 + length + 2;	//Mavlink帧帧头 + mavlink payload + mavlink校验

	
	//-----------------------计算68帧校验-----------------------//
	uint8_t ck_68[2] = { 0 };	//CRC、帧尾0x16
	for (int i = 0; i < 7; i++)
	{
		ck_68[0] += Frame_header_68[target_system_id - 1][i];
	}//68帧前7Byte
	ck_68[0] += DataLength_68[0];	//68帧第8Byte
	//68帧数据域
	for (int i = 0; i < 6; i++)
	{
		ck_68[0] += buf[i];
	}
	for (int i = 0; i < length; i++)
	{
		ck_68[0] += msg->payload64[i];
	}
	ck_68[0] += ck[0];	//mavlink校验位
	ck_68[0] += ck[1];	//mavlink校验位
	ck_68[1] = 0x16;
	//---------------------------计算68帧校验-----------------------//

	//---------------------------计算69帧校验-----------------------//
	uint8_t ck_69[2] = { 0 };	//CRC、帧尾0x17
	for (int i = 0; i < 22; i++)
	{
		ck_69[0] += Frame_header_69[target_system_id - 1][i];
	}//69帧前22Byte
	ck_69[0] += data_length_69[0];	//69帧第23字节
	ck_69[0] += data_length_69[1];	//69帧第24字节
	//69帧数据域
	ck_69[0] += 2 * ck_68[0];
	ck_69[0] += ck_68[1];
	ck_69[1] = 0x17;
	//---------------------------计算69帧校验-----------------------//

	
	//发送69帧的前22Byte，不包括数据域长度
	_mavlink_send_uart((const uint8_t *)Frame_header_69[target_system_id - 1], 22);

	//发送69帧"数据域长度"
	_mavlink_send_uart((const uint8_t *)data_length_69, 2);

	//发送69帧数据域中的68帧前7字节
	_mavlink_send_uart((const uint8_t *)Frame_header_68[target_system_id - 1], 7);

	//发送69帧数据域中的68帧第8字节，即68帧"数据域长度"
	_mavlink_send_uart((const uint8_t *)DataLength_68, 1);

	//发送68帧数据域
	_mavlink_send_uart((const uint8_t *)buf, MAVLINK_NUM_HEADER_BYTES);
	_mavlink_send_uart(_MAV_PAYLOAD_NON_CONST(msg), length);
	_mavlink_send_uart((const uint8_t *)ck, 2);

	//发送68帧校验及帧尾0x16
	_mavlink_send_uart((const uint8_t *)ck_68, 2);

	//发送69帧校验及帧尾0x17
	_mavlink_send_uart((const uint8_t *)ck_69, 2);
}
*/


//xbee
void CMavLinkProcessor::mavlink_finalize_message(
	mavlink1_message_t* msg,
	uint8_t sysid,
	uint8_t compid,
	uint8_t length,
	uint8_t crc_extra,
	uint8_t target_system_id)
{
	uint16_t checksum;
	uint8_t buf[MAVLINK_NUM_HEADER_BYTES];
	uint8_t ck[2];
	buf[0] = MAVLINK_STX;
	buf[1] = length;
	buf[2] = 0;
	buf[3] = sysid;
	buf[4] = compid;
	buf[5] = msg->msgid;
	checksum = crc_calculate((const uint8_t*)&buf[1], MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&checksum, _MAV_PAYLOAD_NON_CONST(msg), length);

	crc_accumulate(crc_extra, &checksum);

	ck[0] = (uint8_t)(checksum & 0xFF);	//取checksum的低8位
	ck[1] = (uint8_t)(checksum >> 8);	//取checksum的高8位

	_mavlink_send_uart((const uint8_t *)buf, MAVLINK_NUM_HEADER_BYTES);
	_mavlink_send_uart(_MAV_PAYLOAD_NON_CONST(msg), length);
	_mavlink_send_uart((const uint8_t *)ck, 2);
}




void CMavLinkProcessor::mavlink_finalize_message2(
	mavlink1_message_t* msg,
	uint8_t sysid,
	uint8_t compid,
	uint8_t length,
	uint8_t crc_extra, 
	uint8_t &mavlink_sequence_number)
{
	uint16_t checksum;
	uint8_t buf[MAVLINK_NUM_HEADER_BYTES];
	uint8_t ck[2];
	buf[0] = MAVLINK_STX;
	buf[1] = length;
	mavlink_sequence_number++;
	if (mavlink_sequence_number > 255)
		mavlink_sequence_number = 0;
	buf[2] = mavlink_sequence_number;
	buf[3] = sysid;
	buf[4] = compid;
	buf[5] = msg->msgid;
	checksum = crc_calculate((const uint8_t*)&buf[1], MAVLINK_CORE_HEADER_LEN);
	crc_accumulate_buffer(&checksum, _MAV_PAYLOAD_NON_CONST(msg), length);

	crc_accumulate(crc_extra, &checksum);

	ck[0] = (uint8_t)(checksum & 0xFF);	//取checksum的低8位
	ck[1] = (uint8_t)(checksum >> 8);	//取checksum的高8位

	_mavlink_send_uart((const uint8_t *)buf, MAVLINK_NUM_HEADER_BYTES);
	_mavlink_send_uart(_MAV_PAYLOAD_NON_CONST(msg), length);
	_mavlink_send_uart((const uint8_t *)ck, 2);
}


void CMavLinkProcessor::mavlink_msg_vicon_position_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw)
{

	mavlink_vicon_position_estimate_t packet;
	//packet.usec = current_tx_seq;
	//printf("%llu\n",packet.usec);
	packet.usec = usec;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	//packet.z = current_tx_seq;
	packet.roll = roll;
	packet.pitch = pitch;
	packet.yaw = yaw;

	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_LEN);

	msg->msgid = MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_LEN, MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_CRC, 1);
	
}

void CMavLinkProcessor::deliverMessage(char* message, int length, char* crc, int length_crc)
{
	_mavlink_send_uart((const uint8_t *)message, length);
	_mavlink_send_uart((const uint8_t *)crc, length_crc);
}


void CMavLinkProcessor::mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, uint8_t target_system_id, uint8_t target_component_id, uint16_t command, uint8_t confirmation, float param[])
{

	/*

	mavlink_command_long_estimate_t packet;

	packet.param1 = param[0];
	packet.param2 = param[1];
	packet.param3 = param[2];
	packet.param4 = param[3];
	packet.param5 = param[4];
	packet.param6 = param[5];
	packet.param7 = param[6];
	packet.command = command;
	packet.target_system = target_system_id;
	packet.target_component = target_component_id;
	packet.confirmation = confirmation;

	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);
	*/
	


	memcpy(&msg->payload64[0], &param[0], 4);
	memcpy(&msg->payload64[4], &param[1], 4);
	memcpy(&msg->payload64[8], &param[2], 4);
	memcpy(&msg->payload64[12], &param[3], 4);
	memcpy(&msg->payload64[16], &param[4], 4);
	memcpy(&msg->payload64[20], &param[5], 4);
	memcpy(&msg->payload64[24], &param[6], 4);
	memcpy(&msg->payload64[28], &command, 2);
	msg->payload64[30] = target_system_id;
	msg->payload64[31] = target_component_id;	
	msg->payload64[32] = confirmation;
	

	//memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);	
	msg->msgid = MAVLINK_MSG_ID_COMMAND_LONG;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_COMMAND_LONG_LEN, MAVLINK_MSG_ID_COMMAND_LONG_CRC, target_system_id);

}

void CMavLinkProcessor::mavlink_msg_set_pos_local_ned_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, uint32_t time, uint8_t target_system_id, uint8_t target_component_id, uint8_t coordinate, uint16_t type_mask, float attitude[])
{
	memcpy(&msg->payload64[0], &time, 4);
	msg->payload64[4] = target_system_id;
	msg->payload64[5] = target_component_id;
	msg->payload64[6] = coordinate;
	memcpy(&msg->payload64[7], &type_mask, 2);
	memcpy(&msg->payload64[9], &attitude[0], 4);
	memcpy(&msg->payload64[13], &attitude[1], 4);
	memcpy(&msg->payload64[17], &attitude[2], 4);
	memcpy(&msg->payload64[21], &attitude[3], 4);
	memcpy(&msg->payload64[25], &attitude[4], 4);
	memcpy(&msg->payload64[29], &attitude[5], 4);
	memcpy(&msg->payload64[33], &attitude[6], 4);
	memcpy(&msg->payload64[37], &attitude[7], 4);
	memcpy(&msg->payload64[41], &attitude[8], 4);
	memcpy(&msg->payload64[45], &attitude[9], 4);
	memcpy(&msg->payload64[49], &attitude[10], 4);

	
	msg->msgid = MAVLINK_MSG_ID_SET_POS_LOCAL_NED;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_POS_LOCAL_NED_LEN, MAVLINK_MSG_ID_SET_POS_LOCAL_NED_CRC, target_system_id);

}

void CMavLinkProcessor::mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, uint32_t custom_mode, uint8_t mode_param[])
{

	mavlink_heartbeat_estimate_t packet;

	packet.custom_mode = custom_mode;
	packet.type = mode_param[0];
	packet.autopilot = mode_param[1];
	packet.base_mode = mode_param[2];
	packet.system_status = mode_param[3];
	packet.mavlink_version = mode_param[4];


	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_HEARTBEAT_LEN);
	
	//memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);	
	msg->msgid = MAVLINK_MSG_ID_HEARTBEAT;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC, 1);

}

void CMavLinkProcessor::mavlink_msg_set_mode_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, uint32_t custom_mode, uint8_t target_system_id, uint8_t base_mode)
{
	mavlink_set_mode_estimate_t packet;


	packet.custom_mode = custom_mode;
	packet.target_system = target_system_id;
	packet.base_mode = base_mode;

	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_MODE_LEN);

	//memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);	
	msg->msgid = MAVLINK_MSG_ID_SET_MODE;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_MODE_LEN, MAVLINK_MSG_ID_SET_MODE_CRC, target_system_id);
}

void CMavLinkProcessor::mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, float param[], uint16_t seq_command[], uint8_t target_frame_etc[])
{
	mavlink_mission_item_estimate_t packet;

	packet.param1 = param[0];
	packet.param2 = param[1];
	packet.param3 = param[2];
	packet.param4 = param[3];
	packet.x = param[4];
	packet.y = param[5];
	packet.z = param[6];
	packet.seq = seq_command[0];
	packet.command = seq_command[1];
	packet.target_system = target_frame_etc[0];
	packet.target_component = target_frame_etc[1];
	packet.frame = target_frame_etc[2];
	packet.current = target_frame_etc[3];
	packet.autocontinue = target_frame_etc[4];


	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_ITEM_LEN);

	//memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);	
	msg->msgid = MAVLINK_MSG_ID_MISSION_ITEM;

	//mavlink_finalize_message2(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_ITEM_LEN, MAVLINK_MSG_ID_MISSION_ITEM_CRC, mavlink_sequence_number);
	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_ITEM_LEN, MAVLINK_MSG_ID_MISSION_ITEM_CRC, target_frame_etc[0]);
}


void CMavLinkProcessor::mavlink_msg_mission_count_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, uint16_t count, uint8_t target_system, uint8_t target_component)
{
	mavlink_mission_count_estimate_t packet;

	packet.count = count;
	packet.target_system = target_system;
	packet.target_component = target_component;

	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_COUNT_LEN);

	//memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);	
	msg->msgid = MAVLINK_MSG_ID_MISSION_COUNT;

	mavlink_finalize_message2(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_COUNT_LEN, MAVLINK_MSG_ID_MISSION_COUNT_CRC, mavlink_sequence_number);
}


void CMavLinkProcessor::mavlink_msg_mission_ack_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, uint8_t target_system, uint8_t target_component, uint8_t type)
{
	mavlink_mission_ack_estimate_t packet;

	packet.type = type;
	packet.target_system = target_system;
	packet.target_component = target_component;

	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_ACK_LEN);

	//memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);	
	msg->msgid = MAVLINK_MSG_ID_MISSION_ACK;

	mavlink_finalize_message2(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_ACK_LEN, MAVLINK_MSG_ID_MISSION_ACK_CRC, mavlink_sequence_number);
}


void CMavLinkProcessor::mavlink_msg_set_pos_global_int_pack(
	uint8_t system_id, 
	uint8_t component_id, 
	mavlink1_message_t* msg, 
	uint32_t time, 
	int32_t lat_lng[], 
	float param[], 
	uint16_t type_mask, 
	uint8_t target_frame[])
{
	mavlink_set_pos_global_int_estimate_t packet;

	packet.time = time;
	packet.lat = lat_lng[0];
	packet.lon = lat_lng[1];
	packet.alt = param[0];
	packet.vx = param[1];
	packet.vy = param[2];
	packet.vz = param[3];
	packet.afx = param[4];
	packet.afy = param[5];
	packet.afz = param[6];
	packet.yaw = param[7];
	packet.yaw_rate = param[8];
	packet.type_mask = type_mask;
	packet.target_system = target_frame[0];
	packet.target_component = target_frame[1];
	packet.coordinate_frame = target_frame[2];


	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_POS_GLOBAL_INT_LEN);

	//memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);	
	msg->msgid = MAVLINK_MSG_ID_SET_POS_GLOBAL_INT;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_POS_GLOBAL_INT_LEN, MAVLINK_MSG_ID_SET_POS_GLOBAL_INT_CRC, target_frame[0]);
}

void CMavLinkProcessor::mavlink_msg_rc_channel_override_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, uint16_t chan_raw[], uint8_t target_comp[])
{
	mavlink_rc_channel_override_t packet;

	packet.chan1_raw = chan_raw[0];
	packet.chan2_raw = chan_raw[1];
	packet.chan3_raw = chan_raw[2];
	packet.chan4_raw = chan_raw[3];
	packet.chan5_raw = chan_raw[4];
	packet.chan6_raw = chan_raw[5];
	packet.chan7_raw = chan_raw[6];
	packet.chan8_raw = chan_raw[7];
	packet.target_system = target_comp[0];
	packet.target_component = target_comp[1];

	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RC_CHANNEL_OVERRIDE_LEN);

	//memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_COMMAND_LONG_LEN);	
	msg->msgid = MAVLINK_MSG_ID_RC_CHANNEL_OVERRIDE;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RC_CHANNEL_OVERRIDE_LEN, MAVLINK_MSG_ID_RC_CHANNEL_OVERRIDE_CRC, target_comp[0]);

}


void CMavLinkProcessor::mavlink_msg_param_set_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t * msg, float param_value, uint8_t target_system, uint8_t target_component, string param_id, uint8_t param_type)
{
	mavlink_param_set_t packet;

	packet.param_value = param_value;
	packet.target_system = target_system;
	packet.target_component = target_component;
	for (int i = 0; i < 16; i++)
	{
		packet.param_id[i] = param_id[i];
	}
	packet.param_type = param_type;

	memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAM_SET_LEN);
	msg->msgid = MAVLINK_MSG_ID_PARAM_SET;

	mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PARAM_SET_LEN, MAVLINK_MSG_ID_PARAM_SET_CRC, target_system);

}
//=========================end write======================
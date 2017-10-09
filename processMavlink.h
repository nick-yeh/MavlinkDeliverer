#pragma once

#include "SerialPort.h"
#include <string>
#include "checkcrc.h"
using namespace std;

//每次实验需要修改的参数
#define FORMATION_HEADING 0.66666*3.1415926
//#define FORMATION_HEADING 1.66666*3.1415926	//整体队形的朝向，in rad，[0, 2*PI)，正北为0，顺时针增加，暂时规定‘ZJU↑’的箭头指向为整体队形的朝向
#define Z_J 32		//字母Z中心到字母J中心地带距离 unit：m
#define Z_U 66
#define cm_per_pixel 40		//画图程序中一个像素表示飞机间隔40cm
#define UAV_NUM_G1 1		//number of UAV in group 1 (controlled by serial port 1)只有一组，所以等于UAV_NUM
#define UAV_NUM_G2 1		//number of UAV in group 2 (controlled by serial port 2)
#define UAV_NUM_G3 1		//number of UAV in group 3 (controlled by serial port 3)
#define UAV_NUM 8		//飞机的最大编号
#define UAV_AVAILABLE_NUM 1		//
#define LEADER_ID 1
#define G1_LEADER_ID 1
#define G2_LEADER_ID 5
#define G3_LEADER_ID 9
#define CENTER_ID_G1 1
#define CENTER_ID_G2 5
#define CENTER_ID_G3 9
#define COMNUM1 4
#define COMNUM2 2
#define COMNUM3 9
#define LAT_L 27
#define LAT_H 32
#define LON_L 117
#define LON_H 122
#define H1 5	//起飞高度
#define H2 1
#define H3 1
#define H4 12
#define H5 1
#define H6 1
#define H7 14
#define H8 16
#define H9 1
#define H10 11
#define H11 13
#define H12 15
#define H13 1
#define H14 1
#define H15 1
#define H16 17
#define H17 1
#define H18 1
#define H19 30
#define H20 28
#define H21 26
#define H22 24
#define H23 22
#define H24 20
#define H25 18
#define H26 16
#define H27 14

#define UAV_NUM_I 9
#define UAV_NUM_L 12
#define UAV_NUM_Z 12
#define UAV_NUM_J 10
#define UAV_NUM_U 10
#define UAV_NUM_C 10
#define UAV_NUM_S 12
#define UAV_NUM_E 11

/////////////////////////////////
#define DELAY_TIME_TROTTING 500		//走马亮灯间隔时间 ms
#define DELAY_TIME_ARM 250		//解锁间隔 ms
#define DELAY_TIME_MOVE 200		//解锁键哥 ms
#define DELAY_TIME_TAKEOFF 250	//起飞间隔 ms
#define DELAY_TIME_TURNOFF 100
#define DELAY_TIME_LIGHTUP 100

#define ATTITUDE_DIM 10	//sysid\time_boot_ms\lat\lng\alt\relative_alt\Vx\Vy\Vz\hdg
#define COMMANDLONG_DIM 11	//param1\2\3\4\5\6\7\command\target_system\target_component\confirmation
#define SETMODE_DIM 3		//custom_mode\target_system\base_mode
#define GLOBALPOS_DIM 16	//time_boot_ms\lat\lon\alt\vx\vy\vz\afx\afy\afz\yaw\yaw_rate\type_musk\target_system\target_component\coordinate_frame

#define BAUD1 57600
#define BAUD2 57600
#define BAUD3 57600

#define PAYLOAD_LENGTH_00 9
#define PAYLOAD_LENGTH_30 28
#define PAYLOAD_LENGTH_32 28
#define PAYLOAD_LENGTH_33 28
#define PAYLOAD_LENGTH_GLOBALPOS 53
#define PAYLOAD_LENGTH_COMMANDLONG 33
#define PAYLOAD_LENGTH_SETMODE 6
#define DATA_LENGTH_NODE_ADDR 2	//node_address_high、node_address_low
#define MAVLINK_HEADER_LENGTH 6
#define MAVLINK_CK_LENGTH 2
#define MAVLINK_LENGTH_BYTE 2
#define MAVLINK_SYS_BYTE 4

//===================begin write=========================================

#define MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_LEN 32
#define MAVLINK_MSG_ID_104_LEN 32
#define MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_CRC 56
#define MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE 104

#define MAVLINK_MSG_ID_COMMAND_LONG_LEN 33
#define MAVLINK_MSG_ID_76_LEN 33
#define MAVLINK_MSG_ID_COMMAND_LONG_CRC 152
#define MAVLINK_MSG_ID_COMMAND_LONG 76
#define MAV_CMD_NAV_TAKEOFF 22
#define MAV_CMD_NAV_LAND 21
#define MAV_CMD_COMPONENT_ARM_DISARM 400

#define MAVLINK_MSG_ID_HEARTBEAT_LEN 9
#define MAVLINK_MSG_ID_00_LEN 9
#define MAVLINK_MSG_ID_HEARTBEAT_CRC 50
#define MAVLINK_MSG_ID_HEARTBEAT 0

#define MAVLINK_MSG_ID_SET_MODE_LEN 6
#define MAVLINK_MSG_ID_11_LEN 6
#define MAVLINK_MSG_ID_SET_MODE_CRC 89
#define MAVLINK_MSG_ID_SET_MODE 11

#define MAVLINK_MSG_ID_SET_POS_LOCAL_NED_LEN 53
#define MAVLINK_MSG_ID_84_LEN 53
#define MAVLINK_MSG_ID_SET_POS_LOCAL_NED_CRC 143
#define MAVLINK_MSG_ID_SET_POS_LOCAL_NED 84

#define MAVLINK_MSG_ID_SET_POS_GLOBAL_INT_LEN 53
#define MAVLINK_MSG_ID_86_LEN 53
#define MAVLINK_MSG_ID_SET_POS_GLOBAL_INT_CRC 5
#define MAVLINK_MSG_ID_SET_POS_GLOBAL_INT 86

#define MAVLINK_MSG_ID_MISSION_ITEM_LEN 37
#define MAVLINK_MSG_ID_39_LEN 37
#define MAVLINK_MSG_ID_MISSION_ITEM_CRC 254
#define MAVLINK_MSG_ID_MISSION_ITEM 39

#define MAVLINK_MSG_ID_MISSION_COUNT_LEN 4
#define MAVLINK_MSG_ID_44_LEN 4
#define MAVLINK_MSG_ID_MISSION_COUNT_CRC 221
#define MAVLINK_MSG_ID_MISSION_COUNT 44

#define MAVLINK_MSG_ID_MISSION_ACK_LEN 3
#define MAVLINK_MSG_ID_47_LEN 3
#define MAVLINK_MSG_ID_MISSION_ACK_CRC 153
#define MAVLINK_MSG_ID_MISSION_ACK 47

#define MAVLINK_MSG_ID_RC_CHANNEL_OVERRIDE_LEN 18
#define MAVLINK_MSG_ID_70_LEN 18
#define MAVLINK_MSG_ID_RC_CHANNEL_OVERRIDE_CRC 124
#define MAVLINK_MSG_ID_RC_CHANNEL_OVERRIDE 70

#define MAVLINK_MSG_ID_PARAM_SET_LEN 23
#define MAVLINK_MSG_ID_23_LEN 23
#define MAVLINK_MSG_ID_PARAM_SET_CRC 168
#define MAVLINK_MSG_ID_PARAM_SET 23



#define MAVLINK_CORE_HEADER_LEN 5 ///< Length of core header (of the comm. layer): message length (1 byte) + message sequence (1 byte) + message system id (1 byte) + message component id (1 byte) + message type id (1 byte)
#define MAVLINK_NUM_HEADER_BYTES (MAVLINK_CORE_HEADER_LEN + 1) ///< Length of all header bytes, including core and checksum
#define MAVLINK_STX 254
#define X25_INIT_CRC 0xffff
#define _MAV_PAYLOAD(msg) ((const char *)(&((msg)->payload64[0])))
#define _MAV_PAYLOAD_NON_CONST(msg) ((uint8_t *)(&((msg)->payload64[0])))


#ifndef __int8_t_defined
# define __int8_t_defined
typedef signed char             int8_t; 
typedef short int               int16_t;
typedef int                     int32_t;
# if __WORDSIZE == 64
typedef long int                int64_t;
# else
//__extension__
typedef long long int           int64_t;
# endif
#endif
/* Unsigned.  */
typedef unsigned char           uint8_t;
typedef unsigned short int      uint16_t;
#ifndef __uint32_t_defined
typedef unsigned int            uint32_t;
# define __uint32_t_defined
#endif
#if __WORDSIZE == 64
typedef unsigned long int       uint64_t;
#else
//__extension__
typedef unsigned long long int  uint64_t;
#endif


typedef struct __mavlink_vicon_position_estimate_t
{
	uint64_t usec; /*< Timestamp (microseconds, synced to UNIX time or since system boot)*/
	float x; /*< Global X position*/
	float y; /*< Global Y position*/
	float z; /*< Global Z position*/
	float roll; /*< Roll angle in rad*/
	float pitch; /*< Pitch angle in rad*/ 
	float yaw; /*< Yaw angle in rad*/
} mavlink_vicon_position_estimate_t;

typedef struct __mavlink_command_long_estimate_t
{
	float param1;
	float param2;
	float param3;
	float param4;
	float param5;
	float param6;
	float param7;
	uint16_t command;
	uint8_t target_system;
	uint8_t target_component;	
	uint8_t confirmation;
	
} mavlink_command_long_estimate_t;

typedef struct __mavlink_heartbeat_estimate_t
{
	uint32_t custom_mode;
	uint8_t type;
	uint8_t autopilot;
	uint8_t base_mode;
	uint8_t system_status;
	uint8_t mavlink_version;

} mavlink_heartbeat_estimate_t;

typedef struct __mavlink_set_mode_estimate_t
{
	uint32_t custom_mode;
	uint8_t target_system;
	uint8_t base_mode;

} mavlink_set_mode_estimate_t;

typedef struct __mavlink_mission_item_estimate_t
{
	float param1;
	float param2;
	float param3;
	float param4;
	float x;
	float y;
	float z;
	uint16_t seq;
	uint16_t command;
	uint8_t target_system;
	uint8_t target_component;
	uint8_t frame;
	uint8_t current;
	uint8_t autocontinue;
} mavlink_mission_item_estimate_t;

typedef struct __mavlink_mission_count_estimate_t
{
	uint16_t count;
	uint8_t target_system;
	uint8_t target_component;
} mavlink_mission_count_estimate_t;

typedef struct __mavlink_mission_ack_estimate_t
{
	uint8_t target_system;
	uint8_t target_component;
	uint8_t type;
} mavlink_mission_ack_estimate_t;

typedef struct __mavlink_rc_channel_override_t
{
	uint16_t chan1_raw;
	uint16_t chan2_raw;
	uint16_t chan3_raw;
	uint16_t chan4_raw;
	uint16_t chan5_raw;
	uint16_t chan6_raw;
	uint16_t chan7_raw;
	uint16_t chan8_raw;
	uint8_t target_system;
	uint8_t target_component;
} mavlink_rc_channel_override_t;

typedef struct __mavlink_set_pos_global_int_estimate_t
{
	uint32_t time;
	int32_t lat;
	int32_t lon;
	float alt;
	float vx;
	float vy;
	float vz;
	float afx;
	float afy;
	float afz;
	float yaw;
	float yaw_rate;
	uint16_t type_mask;
	uint8_t target_system;
	uint8_t target_component;
	uint8_t coordinate_frame;
} mavlink_set_pos_global_int_estimate_t;

typedef struct __mavlink_param_set_t
{
	float param_value;
	uint8_t target_system;
	uint8_t target_component;
	char param_id[16];
	uint8_t param_type;
} mavlink_param_set_t;

typedef struct mavlink_message{
	uint8_t magic;
	uint8_t length;
	uint8_t seq;
	uint8_t sysid;
	uint8_t compid;
	uint8_t msgid;
	uint8_t payload64[255];
	uint8_t checksum[2];
} mavlink1_message_t;



#define MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE 104

//===================end write=======================================

class CMavLinkProcessor:public CSerialPort
{
public:
	char Payload_store_00[PAYLOAD_LENGTH_00];
	char Payload_store_30[PAYLOAD_LENGTH_30];
	char Payload_store_32[PAYLOAD_LENGTH_32];
	char Payload_store_33[PAYLOAD_LENGTH_33];
	char Payload_store_GlobalPos[PAYLOAD_LENGTH_GLOBALPOS];
	char Payload_store_SetMode[PAYLOAD_LENGTH_SETMODE];
	char Payload_store_CommandLong[PAYLOAD_LENGTH_COMMANDLONG];

	char MavlinkMessage_33[PAYLOAD_LENGTH_33 + 6];	//not include crc
	char MavlinkMessage_33_crc[2];
	char MavlinkMessage_GlobalPos[PAYLOAD_LENGTH_GLOBALPOS + 6];	//not include crc
	char MavlinkMessage_GlobalPos_crc[2];
	char MavlinkMessage_SetMode[PAYLOAD_LENGTH_SETMODE + 6];	//not include crc
	char MavlinkMessage_SetMode_crc[2];
	char MavlinkMessage_CommandLong[PAYLOAD_LENGTH_COMMANDLONG + 6];	//not include crc
	char MavlinkMessage_CommandLong_crc[2];

	bool GlobalPos_flag = 0;
	bool SetMode_flag = 0;
	bool CommandLong_flag = 0;
	bool isGCStalk = 0;

	static char Node_addr[UAV_NUM_G1][DATA_LENGTH_NODE_ADDR];
	static char Frame_header_68[UAV_NUM_G1][8];
	static char Frame_header_69[UAV_NUM_G1][24];

	static uint8_t mavlink_sequence_number;

	void parseCharStream(int Attitude[ATTITUDE_DIM]);
	void parseCommandLongStream(int CommandLong[COMMANDLONG_DIM]);
	void parseSetMode(int SetMode[SETMODE_DIM]);
	void parseGlobalPos(int GlobalPos[GLOBALPOS_DIM]);
	bool checkMessage(char* message, char* crc);

	void deliverMessage(char* message, int length, char* crc, int length_crc);

public:  
    CMavLinkProcessor(void);  
    ~CMavLinkProcessor(void);  
 
public:  
      
    /** 初始化串口函数  
     *  
     *  @param:  UINT portNo 串口编号,默认值为1,即COM1,注意,尽量不要大于9  
     *  @param:  UINT baud   波特率,默认为9600  
     *  @param:  char parity 是否进行奇偶校验,'Y'表示需要奇偶校验,'N'表示不需要奇偶校验  
     *  @param:  UINT databits 数据位的个数,默认值为8个数据位  
     *  @param:  UINT stopsbits 停止位使用格式,默认值为1  
     *  @param:  DWORD dwCommEvents 默认为EV_RXCHAR,即只要收发任意一个字符,则产生一个事件  
     *  @return: bool  初始化是否成功  
     *  @note:   在使用其他本类提供的函数前,请先调用本函数进行串口的初始化  
     *　　　　　   /n本函数提供了一些常用的串口参数设置,若需要自行设置详细的DCB参数,可使用重载函数  
     *           /n本串口类析构时会自动关闭串口,无需额外执行关闭串口  
     *  @see:      
     */ 
    bool InitPort( UINT  portNo = 1,UINT  baud = CBR_9600,char  parity = 'N',UINT  databits = 8, UINT  stopsbits = 1,DWORD dwCommEvents = EV_RXCHAR);  
 
    /** 串口初始化函数  
     *  
     *  本函数提供直接根据DCB参数设置串口参数  
     *  @param:  UINT portNo  
     *  @param:  const LPDCB & plDCB  
     *  @return: bool  初始化是否成功  
     *  @note:   本函数提供用户自定义地串口初始化参数  
     *  @see:      
     */ 
    bool InitPort( UINT  portNo ,const LPDCB& plDCB );  
 
    /** 开启监听线程1 
     *  ----yhk---只能用于打开串口1线程
     *  本监听线程完成对串口数据的监听,并将接收到的数据打印到屏幕输出  
     *  @return: bool  操作是否成功  
     *  @note:   当线程已经处于开启状态时,返回flase  
     *  @see:      
     */ 
    bool OpenListenThread1();  
	bool OpenListenThread2();

 
    /** 关闭监听线程  
     *  
     *    
     *  @return: bool  操作是否成功  
     *  @note:   调用本函数后,监听串口的线程将会被关闭  
     *  @see:      
     */ 
    bool CloseListenTread();  
 
    /** 向串口写数据  
     *  
     *  将缓冲区中的数据写入到串口  
     *  @param:  unsigned char * pData 指向需要写入串口的数据缓冲区  
     *  @param:  unsigned int length 需要写入的数据长度  
     *  @return: bool  操作是否成功  
     *  @note:   length不要大于pData所指向缓冲区的大小  
     *  @see:      
     */ 
    bool WriteData(unsigned char* pData, unsigned int length);  
 
    /** 获取串口缓冲区中的字节数  
     *  
     *    
     *  @return: UINT  操作是否成功  
     *  @note:   当串口缓冲区中无数据时,返回0  
     *  @see:      
     */ 
    UINT GetBytesInCOM();  
 
    /** 读取串口接收缓冲区中一个字节的数据  
     *  
     *    
     *  @param:  char & cRecved 存放读取数据的字符变量  
     *  @return: bool  读取是否成功  
     *  @note:     
     *  @see:      
     */ 
    bool ReadChar(char &cRecved);  
 
private:  
 
    /** 打开串口  
     *  
     *    
     *  @param:  UINT portNo 串口设备号  
     *  @return: bool  打开是否成功  
     *  @note:     
     *  @see:      
     */ 
    bool openPort( UINT  portNo );  
 
    /** 关闭串口  
     *  
     *    
     *  @return: void  操作是否成功  
     *  @note:     
     *  @see:      
     */ 
    void ClosePort();  
      
    /** 串口监听线程1
     *  ----yhk----只能用于串口1线程
     *  监听来自串口的数据和信息  
     *  @param:  void * pParam 线程参数  
     *  @return: UINT WINAPI 线程返回值  
     *  @note:     
     *  @see:      
     */ 
    static UINT WINAPI ListenThread1(void* pParam);  
	static UINT WINAPI ListenThread2(void* pParam);


private:
 
    /** 串口句柄 */   
    HANDLE  m_hComm;  
 
    /** 线程退出标志变量 */   
    static bool s_bExit;  
 
    /** 线程句柄 */   
    volatile HANDLE    m_hListenThread;  
 
    /** 同步互斥,临界区保护 */   
    CRITICAL_SECTION   m_csCommunicationSync;       //!< 互斥操作串口  
	


 
//------------------------------------------------------添加的函数与类--------------------------------------
public:
    /** 向串口写float数据（先传输高位字节）
     *  
     *  将缓冲区中的数据写入到串口  
     *  @param:  float floatData 需要发送的float类型的数据
	 *  @param:  bool displayFloat 是否显示发送的float数据，默认不显示
	 *  @param:  bool displayUnsigned 是否显示发送的数据转换成unsigned char的形式，默认不显示
     *  @return: bool  操作是否成功  
     *  @note:   
     *  @see:      
     */ 
    bool WriteFloatData(float floatData,bool displayFloat = false,bool displayUnsigned = false); 

	//=======================begin write============================================

public:
	double last_time_get_pos;
	//DWORD time_delay = GetTickCount();
	double time_delay;
	double last_timestamp;
	double alpha;
	double fh;
	double fs;
	float pos_NED_x;
	float pos_NED_y;
	float pos_NED_z;
	float pos_NED_Vx;
	float pos_NED_Vy;
	float pos_NED_Vz;
	LARGE_INTEGER litmp; 
	LONGLONG QPart1,QPart2;  
	double dfMinus, dfFreq, dfTim; 
	double time_stamp_prev;

	//   -YJ-
	float *QRoll;
	float  *QPitch;
	float *QYaw;

	struct _pos{
		float x;
		float y;
		float z;
	} last_pos;
	struct _vel{
		float v_x;
		float v_y;
		float v_z;
	} vel;




	void crc_init(uint16_t* crcAccum);

	void crc_accumulate(uint8_t data, uint16_t *crcAccum);


	uint16_t crc_calculate(const uint8_t* pBuffer, uint16_t length);

	void crc_accumulate_buffer(uint16_t * crcAccum, const uint8_t * pBuffer, uint16_t length);

	//void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length);

	void _mavlink_send_uart(const uint8_t * buf, uint8_t length);

	//void _mavlink_send_uart(const char *buf, uint8_t length);

	void mavlink_finalize_message(mavlink1_message_t* msg, uint8_t sysid, uint8_t compid, uint8_t length, uint8_t crc_extra, uint8_t target_system_id);

	void mavlink_finalize_message2(mavlink1_message_t* msg, uint8_t sysid, uint8_t compid, uint8_t length, uint8_t crc_extra, uint8_t &mavlink_sequence_number);

	void mavlink_msg_vicon_position_estimate_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, uint64_t usec, float x, float y, float z, float roll, float pitch, float yaw);

	void mavlink_msg_command_long_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, uint8_t target_system_id, uint8_t target_component_id, uint16_t command, uint8_t confirmation, float param[]);

	void mavlink_msg_set_pos_local_ned_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, uint32_t time, uint8_t target_system_id, uint8_t target_component_id, uint8_t coordinate, uint16_t type_mask, float attitude[]);

	void mavlink_msg_heartbeat_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, uint32_t custom_mode, uint8_t mode_param[]);

	void mavlink_msg_set_mode_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, uint32_t custom_mode, uint8_t target_system_id, uint8_t base_mode);

	void mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, float param[], uint16_t seq_command[], uint8_t target_frame_etc[]);

	void mavlink_msg_mission_count_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t * msg, uint16_t count, uint8_t target_system, uint8_t target_component);

	void mavlink_msg_mission_ack_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t * msg, uint8_t target_system, uint8_t target_component, uint8_t type);


	void mavlink_msg_set_pos_global_int_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, uint32_t time, int32_t lat_lng[], float param[], uint16_t type_mask, uint8_t target_frame[]);
	
	void mavlink_msg_rc_channel_override_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t* msg, uint16_t chan_raw[], uint8_t target_comp[]);

	void mavlink_msg_param_set_pack(uint8_t system_id, uint8_t component_id, mavlink1_message_t * msg, float param_value, uint8_t target_system, uint8_t target_component, string param_id, uint8_t param_type);
	//===========================end write========================



};

#include"stm32f10x.h"

#ifndef AX_H
#define AX_H

#define	OK                      0x00// 返回状态正常或该指令完成
#define	ResServoPosInfo         0x01// 请求舵机位置信息
#define	CheckError              0x02// 校验位错误
#define	InvalidPosition         0x03// 插入或删除位置出现错误
#define	NoSuchServo             0x04// 没有这个舵机号码
#define	NoSuchType              0x05// 没有这种数据包类型
#define	NoSuchInstructionType 	0x06// 没有这种指令类型
#define NoSuchFile              0x07// 没有fianlfile文件
#define HaveFile                0x08// 存在finalfile文件,但没有动作序列
#define HaveActionFrame         0x09// 文件中存在动作序列
#define ServoIDNumber           0x0a// 舵机的连通的ID号
#define UpLoadActionsNameType   0x0b// 回复下位机中已经存储的微动作的列表

void getServoConnective(void);
void disableServo(uint8_t *p, uint8_t num);
void enableServo(uint8_t *p, uint8_t num);
void getServoPosition(uint8_t *p, uint8_t num);
void moveServoPosition(uint8_t *p, uint8_t num);
void moveServoPosWithSpeed(uint8_t *p, uint8_t num);
void playMicroAction(uint8_t *p, uint16_t poolSize);
void Packet_Reply(USART_TypeDef* USARTx, unsigned char InstrType,						 //length只包含data的长度
		unsigned char * data, unsigned int length);

void TxPacketBroadSynWrite(uint8_t bInstruction, uint8_t bParameterLength);
uint8_t axSendSpeed(uint8_t bID, uint16_t target_speed);

uint8_t TxPacket(uint8_t bID, uint8_t bInstruction, uint8_t bParameterLength);
uint8_t RxPacket(uint8_t bRxPacketLength);

uint8_t axPing(uint8_t bID);
uint8_t axTorqueOn(uint8_t bID);
uint8_t axTorqueOff(uint8_t bID);

void axTorqueOffAll(void);
void axTorqueOnAll(void);
uint8_t axSendPosition(uint8_t bID, uint16_t target_pos, uint16_t target_speed);
uint16_t axReadPosition(uint8_t bID);

//--- Control Table Address ---
//EEPROM AREA
#define P_MODEL_NUMBER_L      0									  //型号代码L
#define P_MODOEL_NUMBER_H     1									  //型号代码H
#define P_VERSION             2									  //固件版本
#define P_ID                  3									  //ID特征值
#define P_BAUD_RATE           4									  //波特率
#define P_RETURN_DELAY_TIME   5									  //返回延迟时间
#define P_CW_ANGLE_LIMIT_L    6									  //最小角度L
#define P_CW_ANGLE_LIMIT_H    7									  //最小角度H
#define P_CCW_ANGLE_LIMIT_L   8									  //最大角度（L）（CCW代表逆时针）
#define P_CCW_ANGLE_LIMIT_H   9									  //最大角度（H）
#define P_SYSTEM_DATA2        10								  //（保留）
#define P_LIMIT_TEMPERATURE   11							      //最高的界限温度
#define P_DOWN_LIMIT_VOLTAGE  12								  //最低的界限电压
#define P_UP_LIMIT_VOLTAGE    13								  //最高的界限电压
#define P_MAX_TORQUE_L        14								  //最大转力矩L
#define P_MAX_TORQUE_H        15								  //最大转力矩H
#define P_RETURN_LEVEL        16								  //状态反馈程度
#define P_ALARM_LED           17								  //LED 警报 警报引导
#define P_ALARM_SHUTDOWN      18								  //撤消扭矩警报
#define P_OPERATING_MODE      19
#define P_DOWN_CALIBRATION_L  20
#define P_DOWN_CALIBRATION_H  21
#define P_UP_CALIBRATION_L    22
#define P_UP_CALIBRATION_H    23

#define P_TORQUE_ENABLE         (24)							 //激活扭矩
#define P_LED                   (25)						  	//LED
#define P_CW_COMPLIANCE_MARGIN  (26)						   //顺时针柔性边距
#define P_CCW_COMPLIANCE_MARGIN (27)						   //逆时针柔性边距
#define P_CW_COMPLIANCE_SLOPE   (28)						   //顺时针柔性斜率
#define P_CCW_COMPLIANCE_SLOPE  (29)						   //逆时针伺服斜率
#define P_GOAL_POSITION_L       (30)						   // 目标位置（低）
#define P_GOAL_POSITION_H       (31)						   // 目标位置（高）
#define P_GOAL_SPEED_L          (32)					 	  // 运动速度（低）
#define P_GOAL_SPEED_H          (33)					     // 运动速度（高）
#define P_TORQUE_LIMIT_L        (34)					    // 扭矩限制（低）
#define P_TORQUE_LIMIT_H        (35)					   //扭矩限制（高）
#define P_PRESENT_POSITION_L    (36)					  //  当前位置（低）
#define P_PRESENT_POSITION_H    (37)				     //	当前位置（高）
#define P_PRESENT_SPEED_L       (38)				    //  当前速度（低）
#define P_PRESENT_SPEED_H       (39)				   //	当前速度（高）
#define P_PRESENT_LOAD_L        (40)				  //	当前负载（低）
#define P_PRESENT_LOAD_H        (41)			     //	    当前负载（高）
#define P_PRESENT_VOLTAGE       (42)			    //	    当前电压
#define P_PRESENT_TEMPERATURE   (43)			   //       当前温度
#define P_REGISTERED_INSTRUCTION (44)
#define P_PAUSE_TIME            (45)
#define P_MOVING (46)
#define P_LOCK                  (47)
#define P_PUNCH_L               (48)
#define P_PUNCH_H               (49)

//--- Instruction ---
#define INST_PING           0x01						//连通性检测程序,无操作，用来得到状态数据包	  
#define INST_READ           0x02					   	//数据读取,读取控制表中的数值
#define INST_WRITE          0x03						//数据写入,向控制表中写入数值
#define INST_REG_WRITE      0x04						//写入寄存器,和数据写入类似，但在操作指令给出之前处于备用模式.注册表写入指令和数据写入指令类似，但是两者指令包完成的时间是不同的
#define INST_ACTION         0x05						//操作,引发写入的指令 ,即接收该指令后，执行INST_REG_WRITE的内容（避免延迟，舵机不同步），无返回状态包
#define INST_RESET          0x06						//复位,把Dynamixel舵机的控制表数值调整到出厂设置值
#define INST_DIGITAL_RESET  0x07
#define INST_SYSTEM_READ    0x0C
#define INST_SYSTEM_WRITE   0x0D
#define INST_SYNC_WRITE     0x83						// 同时写入,用于同时控制多个Dynamixel舵机
#define INST_SYNC_REG_WRITE 0x84

#define CLEAR_BUFFER gbRxBufferReadPointer = gbRxBufferWritePointer
#define DEFAULT_RETURN_PACKET_SIZE 6
#define BROADCASTING_ID 0xfe

#define  AX_TXD  GPIO_SetBits(GPIOB,GPIO_Pin_12); GPIO_ResetBits(GPIOB,GPIO_Pin_13);
#define  AX_RXD  GPIO_SetBits(GPIOB,GPIO_Pin_13); GPIO_ResetBits(GPIOB,GPIO_Pin_12);
//#define  AX_TXD  GPIO_SetBits(GPIOB,GPIO_Pin_12);
//#define  AX_RXD  GPIO_ResetBits(GPIOB,GPIO_Pin_12);

#define ACTION_COMPLETE 0
#define ACTION_ERROR 1

#define AX_MAX_NUM 10//最大舵机编号，检测舵机号只检测从0到AX_MAX_NUM-1。
#define PNULL 0

#define DEFAULTSPEED 512
extern volatile uint8_t gbpParameter[];
extern volatile uint8_t gbpRxBuffer[];
extern volatile unsigned char gbpRxInterruptBuffer[];
extern volatile unsigned char gbRxBufferWritePointer;
//volatile uint8_t gbRxBufferReadPointer;
void changeServoID(uint8_t *p, uint8_t num);

#endif


#include"stm32f10x.h"
#include "AX.h"
#include "usart2.h"
#include "led.h"
#define RX_TIMEOUT_COUNT2   500L
#define RX_TIMEOUT_COUNT1  (RX_TIMEOUT_COUNT2*10L)

volatile uint8_t gbpParameter[128];				 //数据包缓存
volatile uint8_t gbRxBufferReadPointer;
volatile uint8_t gbpRxBuffer[128];				 //经过处理的缓冲数据
volatile uint8_t gbpTxBuffer[128];				// 发送缓存
volatile uint8_t gbRxBufferWritePointer;	    //	  
volatile uint8_t gbpRxInterruptBuffer[256];	   //	中断接收的缓冲数据
volatile uint8_t Timer_count;
volatile uint8_t axOline[AX_MAX_NUM];
volatile uint8_t axOlineNum;
volatile uint8_t delayCount;
extern u8 pcneedpos[2];

uint8_t TxPacket(uint8_t bID, uint8_t bInstruction, uint8_t bParameterLength)
   {		//bParameterLength为参数PARAMETER长度，返回数据包长信息(除掉指令等)

	
	uint8_t bCount, bPacketLength;
	uint8_t bCheckSum;

	gbpTxBuffer[0] = 0xff;
	gbpTxBuffer[1] = 0xff;
	gbpTxBuffer[2] = bID;
	gbpTxBuffer[3] = bParameterLength + 2;		//Length(Paramter,Instruction,Checksum) 
	gbpTxBuffer[4] = bInstruction;
	for (bCount = 0; bCount < bParameterLength; bCount++)
	 {
		gbpTxBuffer[bCount + 5] = gbpParameter[bCount];
	 }
	bCheckSum = 0;
	bPacketLength = bParameterLength + 4 + 2;					  //bPacketLength包含了两个0XFF	,和checksum
	for (bCount = 2; bCount < bPacketLength - 1; bCount++) //except 0xff,checksum

			{
		bCheckSum += gbpTxBuffer[bCount];
	}
	gbpTxBuffer[bCount] = (uint8_t) (~bCheckSum); //Writing Checksum  with  Bit Inversion
//	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    CLEAR_BUFFER;
	AX_TXD
	for (bCount = 0; bCount < bPacketLength; bCount++) 
	{
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
		USART_SendData(USART3, gbpTxBuffer[bCount]);
	}

	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);

	AX_RXD
  //  for(i=0;i<0x8fff;i++);
	return (bPacketLength);		   //返回发送的所有数据长度
}

/*uint8_t RxPacket(uint8_t bRxPacketLength)
 * bRxPacketLength 我们准备接收的数据长度
 * return 实际返回的数据长度（包括0XFF）
 * */
uint8_t RxPacket(uint8_t bRxPacketLength)
 {

	unsigned long ulCounter;
	uint8_t bCount, bLength, bChecksum;
	uint8_t bTimeout;
	bTimeout = 0;
	/*接受循环
	 * 当接受到指定长度的数据或者超时的时候退出循环*/
	for (bCount = 0; bCount < bRxPacketLength; bCount++) 
	{
		ulCounter = 0;
		/*利用中断接受数据
		 * 主程序处于等待状态
		 * 当中断缓冲区里面没有数据gbRxBufferReadPointer == gbRxBufferWritePointer
		 * 时间计数器自增
		 * 当超时或者中断缓冲区里面有数据的时候跳出等待*/
		while (gbRxBufferReadPointer == gbRxBufferWritePointer) 
		{						   
			if (ulCounter++ > RX_TIMEOUT_COUNT1) 
			{
				bTimeout = 1;
				break;
			}
		}
		if (bTimeout) break;
		gbpRxBuffer[bCount] = gbpRxInterruptBuffer[gbRxBufferReadPointer++];
	} ///这里把数据都从中断队列中放到RxBuffer里面了

	bLength = bCount;
	bChecksum = 0;
	/*默认情况下TxPacket与RxPacket连续使用
	 * 当上一个数据包不是BROADCASTING_ID时有返回的数据*/
	if (gbpTxBuffer[2] != BROADCASTING_ID)
	 {
		if (bTimeout && (bRxPacketLength != 255)) 
		{				 //超时，且未接收完
			//TxDString("\r\n [Error:RxD Timeout]");
			CLEAR_BUFFER; //清空接收缓冲区
			return 0;
	    }

		if (bLength > 3) //checking is available.				 //接收数据长度至少要大于3位
		{
			if (gbpRxBuffer[0] != 0xff || gbpRxBuffer[1] != 0xff) 
			{
				//TxDString("\r\n [Error:Wrong Header]");
				CLEAR_BUFFER;	  //清空接收缓冲区
				return 0;
			}
			if (gbpRxBuffer[2] != gbpTxBuffer[2]) 
			{
				//TxDString("\r\n [Error:TxID != RxID]");
				CLEAR_BUFFER;
				return 0;
			}
			if (gbpRxBuffer[3] != bLength - 4) 		 //接受到的数据长度-4不等于length
			{					//默认状态包长度不大于255
				//TxDString("\r\n [Error:Wrong Length]");
				CLEAR_BUFFER;
				return 0;
			}
			for (bCount = 2; bCount < bLength; bCount++)
				bChecksum += gbpRxBuffer[bCount];
			if (bChecksum != 0xff) 
			{
				//TxDString("\r\n [Error:Wrong CheckSum]");
				CLEAR_BUFFER;
				return 0;
			}
		}
	}
//	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
	return bLength;
}


//uint8_t axPing(uint8_t bID) {
//unsigned char bTxPacketLength, bRxPacketLength;
//bTxPacketLength = TxPacket(bCount, INST_PING, 0);
//bRxPacketLength = RxPacket(255); //我们准备接收好多个数据
//if (bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE)
//	;
//}

uint8_t axTorqueOff(uint8_t bID) ///释放舵机的力矩
{
	gbpParameter[0] = P_TORQUE_ENABLE; //Address of Torque
	gbpParameter[1] = 0; //Writing Data
	TxPacket(bID, INST_WRITE, 2);
	if (RxPacket(DEFAULT_RETURN_PACKET_SIZE) == DEFAULT_RETURN_PACKET_SIZE)
		return OK;
	else
		return NoSuchServo;
}

uint8_t axTorqueOn(uint8_t bID) ///使能舵机的力矩
{
	gbpParameter[0] = P_TORQUE_ENABLE; //Address of Torque
	gbpParameter[1] = 1; //Writing Data
	TxPacket(bID, INST_WRITE, 2);
	if (RxPacket(DEFAULT_RETURN_PACKET_SIZE) == DEFAULT_RETURN_PACKET_SIZE)
		return OK;
	else
		return NoSuchServo;
}

uint8_t axSendPosition(uint8_t bID, uint16_t target_pos, uint16_t target_speed)	  //给某个舵机位置
///发送给某一个舵机一个要移动到的位置，包括三个参数 1ID 2位置 3速度
{
	gbpParameter[0] = P_GOAL_POSITION_L; //Address of Firmware Version
	gbpParameter[1] = target_pos; //Writing Data P_GOAL_POSITION_L
	gbpParameter[2] = target_pos >> 8; //Writing Data P_GOAL_POSITION_H
	gbpParameter[3] = target_speed; //Writing Data P_GOAL_SPEED_L
	gbpParameter[4] = target_speed >> 8; //Writing Data P_GOAL_SPEED_H
	TxPacket(bID, INST_WRITE, 5);
	if (RxPacket(DEFAULT_RETURN_PACKET_SIZE + 1) == DEFAULT_RETURN_PACKET_SIZE)
	{
	   return OK;
	}
	
	else
	{		
			return NoSuchServo;
	}
	
}

/*uint16_t axReadPosition(uint8_t bID)
 * parameter
 * 		bID 舵机号
 * return value
 * 		舵机位置
 * 		0xffff表示出错了
 * */

uint16_t axReadPosition(uint8_t bID) 			  //读取某个舵机的位置
{
	unsigned int Position;
	gbpParameter[0] = P_PRESENT_POSITION_L; //位置数据的起始地址 #define P_GOAL_POSITION_L (30) 参见数据手册
	gbpParameter[1] = 2; //读取长度
	TxPacket(bID, INST_READ, 2);
	if(RxPacket(DEFAULT_RETURN_PACKET_SIZE + gbpParameter[1]) != DEFAULT_RETURN_PACKET_SIZE + gbpParameter[1])
	{		 //?????????????
		Position = 0xffff;
	}
	else
	{
	    pcneedpos[0]=gbpRxBuffer[6];
		pcneedpos[1]=gbpRxBuffer[5];		//把数据存起来传给上位机
		Position = ((unsigned int) gbpRxBuffer[6]) << 8;
		Position += gbpRxBuffer[5];
	
	}
	return Position;
}

void axTorqueOffAll(void)					//释放所有舵机力矩
 {
	gbpParameter[0] = P_TORQUE_ENABLE;
	gbpParameter[1] = 0x00;
	TxPacket(BROADCASTING_ID, INST_WRITE, 2);

 }

void axTorqueOnAll(void) 				  //使能所有舵机力矩
{
	gbpParameter[0] = P_TORQUE_ENABLE;
	gbpParameter[1] = 0x01;
	TxPacket(BROADCASTING_ID, INST_WRITE, 2);
}


/*void axScan(void)
 * Return Values
 * 		在线舵机总数
 * 所能检测ID的范围0~AX_MAX_NUM-1
 * 检测连线的舵机号码保存在全局数组axOline[30]
 * 检测连线的舵机数保存在全局数组axOlineNum
 * */

void getServoConnective(void) 
{							 //PING舵机从几号开始
	uint8_t bCount;
	axOlineNum = 0;
	for (bCount = 0; bCount < AX_MAX_NUM - 1; bCount++) ///用一个循环查询每一个舵机的状态为了快速我们只扫描0x00~0x1f
	{
		TxPacket(bCount, INST_PING, 0);
		if (RxPacket(255) == DEFAULT_RETURN_PACKET_SIZE) //如果返回包的长度正确
		axOline[axOlineNum++] = bCount;
	}

	Packet_Reply(USART2, ServoIDNumber, (uint8_t *) &axOline[0], axOlineNum);

}


void moveServoPosition(uint8_t *p, uint8_t num) 
{
	uint8_t i, err = OK;
	uint16_t pos;
	for (i = 0; i < num / 3; i++) {
		pos = (*(p + (3 * i) + 2)) * 256 + (*(p + (3 * i) + 1));
		err |= axSendPosition(*p + (3 * i), pos, DEFAULTSPEED);
	}

	Packet_Reply(USART2, err, PNULL, 0);
}


void moveServoPosWithSpeed(uint8_t *p, uint8_t num) 
{
	uint8_t i, err = OK;
	uint16_t pos, speed;
	for (i = 0; i < num / 5; i++) 
	{
		pos = *(p + (5 * i) + 2) * 256 + *(p + (5* i) +1);
		speed = *(p + (5* i) + 4) * 256 + *(p + (5 * i)+3 );
		err |= axSendPosition(*(p + (5 * i)), pos, speed);
	}
	Packet_Reply(USART2, err, PNULL, 0);
}

/*GetServoPosition(uint8_t *p,uint8_t num)
 * parameter:
 * 		*p
 * 			指向有效数据数组的头指针
 * 		num
 * 			有数组有效长度
 * */
void getServoPosition(uint8_t *p, uint8_t num) 
{
	uint8_t i;
	uint8_t retdata[100];
	uint16_t tmp;
	switch (num) {
	case 1:
		if (*p == BROADCASTING_ID) { //读取全部舵机pos
			for (i = 0; i < axOlineNum; i++) {
				retdata[i*3] = axOline[i];
				tmp = axReadPosition(axOline[i]);
				retdata[i*3+1] =tmp & 0xff;
				retdata[i*3+2] =(tmp & 0xff00) >> 8; 
			}
			
		} else {
			retdata[0] = *p;
			tmp = axReadPosition(*p);
			retdata[1] = tmp & 0xff;
			retdata[2] = (tmp&0xff00) >> 8;
		}
		Packet_Reply(USART2, ResServoPosInfo, (uint8_t *) &retdata[0],
					axOlineNum * 3);
		break;
	//case 2:
	default:
		for (i = 0; i < num; i++) {
			retdata[i*3] = *p;
			tmp = axReadPosition(*p);
			p++;
			retdata[i*3+1] = tmp & 0xff;
			retdata[i*3+2] = (tmp & 0xff00) >> 8;
		}
		Packet_Reply(USART2, ResServoPosInfo, (uint8_t *) &retdata,
				num * 3);
		break;
	}		  
}

void enableServo(uint8_t *p, uint8_t num) 
{
	uint8_t err = OK;
	uint8_t i, *p0;
	p0 = p;
	switch (num) 
	{
	case 1:
		if (*p0 == BROADCASTING_ID) { //读取全部舵机pos
			axTorqueOnAll();
			err = OK;
			Packet_Reply(USART2, err, PNULL, 0);

		} else {
			err = axTorqueOn(*p0);
			Packet_Reply(USART2, err, PNULL, 0);
		}
		break;
	default:
		for (i = 0; i < num; i++)
			err |= axTorqueOn(*p0++);
		Packet_Reply(USART2, err, PNULL, 0);
		break;
	}
}

void disableServo(uint8_t *p, uint8_t num) 
{
	uint8_t err = OK;
	uint8_t i, *p0;
	p0 = p;
	switch (num) {
	case 1:
		if (*p0 == BROADCASTING_ID) { //读取全部舵机pos
			axTorqueOffAll();
			err = OK;
			Packet_Reply(USART2, err, PNULL, 0);
		} else {
			err = axTorqueOff(*p0++);
			Packet_Reply(USART2, err, PNULL, 0);
		}
		break;
	case 2:
		for (i = 0; i < num; i++)
			err |= axTorqueOff(*p0);
		Packet_Reply(USART2, err, PNULL, 0);
		break;
	}
}

void executeInstruction(uint8_t *p, uint8_t num) 
{
	uint8_t err;
	uint8_t i;
	//Send
	AX_TXD
	for (i = 0; i < num; i++) 
	{
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
			;
		USART_SendData(USART3, *p++);
	}
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
		;
	//Rec
	AX_RXD
	if (RxPacket(DEFAULT_RETURN_PACKET_SIZE) == DEFAULT_RETURN_PACKET_SIZE)
		err = OK;
	else
		err = NoSuchServo;
	Packet_Reply(USART2, err, PNULL, 0);
	
}
/***********************
*void playMicroAction(uint8_t *p,uint16_t poolSize)
 * parameter:
 * 		*p
 * 			指令池首地址指针
 * 		poolSize
 * 			指令池大小*
 * void playMicroAction(uint8_t *p, uint16_t poolSize) {
	 * FF FF FE LENGTH INST_SYNC_WRITE(0X83)
	 * 1E(1st write add) 04(write bytes)
	 * ID1 	POSL   POSH   SPEEDL   SPEEDH
	 * ID1 	POSL   POSH   SPEEDL   SPEEDH
	 * ...
	 * ...
	 * IDN 	POSL   POSH   SPEEDL   SPEEDH
	 * CHECKSUM
	 * hence all package length is LENGTH(*(P+3))+4
	 *
*	uint16_t packageLength, j, time;
	uint16_t packageNum, i;
//	TimeInterval = (*p) * 10;												 //感觉应该*10																
	packageLength = (uint16_t)(*(p+1)+(uint16_t)*(p+2)*256);		//可能有问题 （必须滴）
	packageNum = (poolSize-3)/ packageLength;  
	p = p+3;
	AX_TXD;
	for (i = 0; i < packageNum; i++) {
		//启动延时函数
		MiniActionBeginFlag = 1;
		while(NewKeyActionFlag != 1)
			;
		for (j = 0; j <packageLength; j++) {
			while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
				;
			USART_SendData(USART3, *p++);
		}
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
			;
		NewKeyActionFlag = 0;
	}
	AX_RXD;
}  
*********************/


void TxPacketBroadSynWrite(uint8_t bInstruction, uint8_t bParameterLength) {
	uint8_t bCount, bPacketLength;
	gbpTxBuffer[0] = 0xff;
	gbpTxBuffer[1] = 0xff;
	gbpTxBuffer[2] = 0xfe;
	gbpTxBuffer[3] = bParameterLength + 2;
//Length(Paramter,Instruction,Checksum) 
	gbpTxBuffer[4] = bInstruction;
	for (bCount = 0; bCount < bParameterLength; bCount++) {
		gbpTxBuffer[bCount + 5] = gbpParameter[bCount];
	}
	bPacketLength = bParameterLength + 5;											 //可能有问题

	AX_TXD
	for (bCount = 0; bCount < bPacketLength; bCount++) {
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
			;
		USART_SendData(USART3, gbpTxBuffer[bCount]);
	}
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
		;

	AX_RXD;
}

void changeServoID(uint8_t *p, uint8_t num)
{
	gbpParameter[0] = P_ID; //舵机ID地址
	gbpParameter[1] = *p;
	if(num == 1){							 //一次只能给1个制定的ID
	TxPacket(BROADCASTING_ID,INST_WRITE,2);
	getServoConnective();
	}
	else{
		Packet_Reply(USART2, CheckError, (void *)0, 0);
	}
}


uint8_t axSendSpeed(uint8_t bID, uint16_t target_speed)
///发送给某一个舵机一个要移动到的位置，包括三个参数 1ID 2位置 3速度
{
	gbpParameter[0] = P_GOAL_SPEED_L; //Address of Firmware Version
	gbpParameter[3] = target_speed; //Writing Data P_GOAL_SPEED_L
	gbpParameter[4] = target_speed >> 8; //Writing Data P_GOAL_SPEED_H
	TxPacket(bID, INST_WRITE, 3);
	if (RxPacket(DEFAULT_RETURN_PACKET_SIZE) == DEFAULT_RETURN_PACKET_SIZE)
		return OK;
	else
		return NoSuchServo;
}

//  ff Length_L	  Length_H	 0	 2	   InstrType	****	   Check_Sum
//InstrType代被回复指令类型，data代表返回数据的指针,length代表要发送的也就是我的下家返回给我的数据长度，要打包送给上位机
void Packet_Reply(USART_TypeDef* USARTx, unsigned char InstrType,unsigned char * data, unsigned int length) 						
	{			   //length只包含data的长度

	 unsigned char Length_H, Length_L, Check_Sum = 0;
	 unsigned int j = 0;
	 Length_H = (length + 6) >> 8;
	 Length_L = (length + 6);
	 
	 RS485_TX_MODE		   //单片机发送485数据	
	 while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)	 //发送数据寄存器空标志位
	 ;

	 USART_SendData(USARTx, 0XFF); //包头
	 while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
	 ;

	 while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
	 ;

	 USART_SendData(USARTx, Length_L); //长度l
	 while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
	 ;
	 Check_Sum = Check_Sum + Length_L;
	 while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
	 ;

	 USART_SendData(USARTx, Length_H); //长度2
	 while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
	 ;
	 Check_Sum = Check_Sum + Length_H;

	 while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
	 ;

	 USART_SendData(USARTx, 0); //ID =0,代表数字舵机
	 while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
	 ;
	 while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
	 ;

	 USART_SendData(USARTx, 2); //代表这是回复信息
	 while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
	 ;
	 Check_Sum = Check_Sum + 2;

	 while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
	 ;

	 USART_SendData(USARTx, InstrType); //代表指令类型
	 while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
	 ;
	 Check_Sum = Check_Sum + InstrType;

	 //发送有效数据
	 for (j = 0; j < length; j++) 
	 {
		 while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
		 ;
	
		 USART_SendData(USARTx, *(data + j));
		 Check_Sum = Check_Sum + *(data + j);
		 while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
		 ;
	 }
	 Check_Sum = ~Check_Sum; //计算校验和

	 while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
	 ;

	 USART_SendData(USARTx, Check_Sum); //代表指令类型
	 while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
	 ;

	 Check_Sum = 0;
     RS485_RX_MODE		 //接收485 
}

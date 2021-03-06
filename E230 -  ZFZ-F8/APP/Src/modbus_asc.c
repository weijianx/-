
#include "modbus_asc.h"
#include "modbus_ascii.h"
#include "type.h"
#include "para.h"
#include "string.h"

#include "timer.h"

#include "SHT3x.h"

#if ds18b20
#include "ds18b20.h"

#endif

uint8_t StartFlag = 0;
uint8_t StartCountFlag = 0;
uint8_t SendLen;
uint8_t SendBuf[100];
extern uint8_t UARTx_RXBuff[USART_MAX_DATALEN];
extern UserTypeDef UserPara;

extern uint32_t time1;
extern uint8_t uint8_tSendNum;
extern BitAction UartRecvFrameOK;
extern BitAction StartFillBufFlag;


extern uint8_t Cur_Param[USER_DEFAULT_LEN];
extern uint8_t const  User_Default_Param[USER_DEFAULT_LEN];


uint8_t const SensorSoftVersion[10] = {0x07, 'G', 'D', '1', '.', '0', '.', '0'};      //软件版本  20200413


extern uint16_t Current_PositiveTime ;  //当前正转脉冲数
extern uint16_t Current_NegativeTime ;  //当前反转脉冲数

//**************************************************************************************************
// 名称         	: MBASCII_GetSlaveAddr()
// 创建日期   	        : 2015-10-29
// 作者         	: wang
// 功能         	:获得设备地址
// 输入参数     	:接收的缓冲数据(uint8_t *uint8_tMsg)
// 输出参数     	: 设备地址
// 返回结果     	: 无
// 注意和说明   	:
// 修改内容     	:
//**************************************************************************************************

uint32_t MBASC_GetSlaveAddr(uint8_t *uint8_tMsg)
{
    return(uint8_tMsg[0]);
}


//******************************************************************************
// 名称         : MBASC_SendMsg()
// 创建日期     : 2018-06-08
// 作者         : MMX
// 功能         : 以ASCII格式发送消息
// 输入参数     : 帧缓存区(uint8_t *uint8_tMsg),帧长(uint8_t uint8_tMsgLen)
// 输出参数     : 无
// 返回结果     : 无
// 注意和说明   : 无
// 修改内容     : 无
//******************************************************************************
void MBASC_SendMsg(uint8_t *uint8_tMsg, uint8_t uint8_tMsgLen)
{
    if(MB_ADDRESS_BROADCAST != uint8_tMsg[0])
    {
      Delay_Ms(50);
        MODBUS_ASCII_SendData(uint8_tMsg, uint8_tMsgLen);
    }
}

//******************************************************************************
// 名称         : MBASC_SendMsg_NoLimit()
// 创建日期     : 2018-06-08
// 作者         : MMX
// 功能         : 无限制性的以ASCII格式发送消息
// 输入参数     : 帧缓存区(uint8_t *uint8_tMsg),帧长(uint8_t uint8_tMsgLen)
// 输出参数     : 无
// 返回结果     : 无
// 注意和说明   : 无
// 修改内容     : 无
//******************************************************************************
void MBASC_SendMsg_NoLimit(uint8_t *uint8_tMsg, uint8_t uint8_tMsgLen)
{
   Delay_Ms(50);
    MODBUS_ASCII_SendData(uint8_tMsg, uint8_tMsgLen);
}

//******************************************************************************
// 名称         : MBASC_SendErr()
// 创建日期     : 2016-06-08
// 作者         : MMX
// 功能         : 发送错误码的响应帧
// 输入参数     : 错误码(uint8_t ErrCode)
// 输出参数     : 无
// 返回结果     : 无
// 注意和说明   : 无
// 修改内容     : 无
//******************************************************************************
void MBASC_SendErr(uint8_t ErrCode)
{
    SendLen = 1;
    SendBuf[SendLen++] |= 0x80;
    SendBuf[SendLen++] = ErrCode;
    MBASC_SendMsg(SendBuf, SendLen);
}

//******************************************************************************
// 名称         : MBASC_Fun03()
// 创建日期     : 2018-06-08
// 作者         : MMX
// 功能         : 03功能码，读单个寄存器操作
// 输入参数     : 无
// 输出参数     : 无
// 返回结果     : 无
// 注意和说明   : 无
// 修改内容     : 无
//******************************************************************************
void MBASC_Fun03(void)
{
    uint8_t i;
    uint16_t Data_Buf;
   // uint32_t Data_Buf1;
    uint16_t Register_Num = 0;
    uint8_t ReadAdrH, ReadAdrL;
    
    ReadAdrH = UARTx_RXBuff[2];
    ReadAdrL = UARTx_RXBuff[3];

    Register_Num = ((uint16_t)UARTx_RXBuff[4] << 8) + UARTx_RXBuff[5];
     
    SendLen = 0;
    SendBuf[SendLen++] = UARTx_RXBuff[0] ? UserPara.SlaveAddr : 0x00;
    SendBuf[SendLen++] = 0x03;	                        //功能码
    SendBuf[SendLen++] = Register_Num * 2;		                        //数据长度
                                                                                //如果读取范围溢出
    if ((!(((ReadAdrL >= MBASC_HOLDING_REG_REGION_BGEIN) && (ReadAdrL <= MBASC_HOLDING_REG_REGION_END)
        && (ReadAdrL + Register_Num <= (MBASC_HOLDING_REG_REGION_END + 1)))&& (0 != Register_Num)))
        || ((ReadAdrH != UserPara.SlaveAddr) && (ReadAdrH != MB_ADDRESS_BROADCAST)))//广播地址放开
    {
        MBASC_SendErr(MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }

    for (uint8_t k = 0; k < Register_Num; ReadAdrL++, k++)
    {
        switch (ReadAdrL)
        {
          case 0x30:
            Data_Buf = UserPara.SlaveAddr;      //设备地址
            break;

          case 0x31:
            Data_Buf = UserPara.Baudrate;	//查询波特率：			        
            break;

          case 0x32:
            Data_Buf = 3;		//查询奇偶校验：	                        
            break;

          case 0x33:
            Data_Buf = 0;						        
            break;

          case 0x34:
            Data_Buf = 0;  //补偿使能	                        
            break;

          case 0x35:                                                          
            Data_Buf = 0;	//滤波方式	  		                                             
            break;

          case 0x36:
            Data_Buf = 0;	//自动上报方式	               
            break;

          case 0x37:            //输出修正系数K
            Data_Buf = 0;                       
            break;

          case 0x38:             //输出修正系数B
            Data_Buf = 0;                        
            break;

          case 0x39:
            Data_Buf = 0;	                                                                 
            break;

          case 0x3A:
            Data_Buf = 0;	                                                             
            break;  
            
          case 0x3B:
            Data_Buf = 0;	                                                             
            break;             
            
          case 0x3C:
            Data_Buf = 0;	                                                             
            break;
            
          case 0x3D:
            //Data_Buf = UserPara.HeightUnit;	
            break;             
            
          case 0x3E:
            Data_Buf = 0;	                                                             
            break;
            
          case 0x3F:
           // Data_Buf = UserPara.SensorRange;//;传感器长度量程                                                  
            break;
            
          case 0x40:
            Data_Buf = 0;//UserPara.DetectThr;		        
            break;

          case 0x41:
            Data_Buf = 0;
            //Data_Buf = UserPara.StandbyAlarmThr;
            break;
            
          case 0x42:
            Data_Buf = 0;//UserPara.StaVaryDuration;                                 
            break;  
            
          case 0x43:
            Data_Buf = 0;//(uint16_t)(UserPara.WorkTimeBase >> 16);                 //总工作时长基值高位                       
            break;

          case 0x44:
            Data_Buf = 0;//(uint16_t)UserPara.WorkTimeBase;                         //总工作时长基值低位                       
            break;

          case 0x45:
            Data_Buf = 0;//(uint16_t)(UserPara.StandbyTimeBase >> 16);              //总待机时长基值高位                                                            
            break;

          case 0x46:
            Data_Buf =0;//(uint16_t)UserPara.StandbyTimeBase;                       //总待机时长基值低位                                                              
            break;  
            
          case 0x47:
            Data_Buf = 0;//(uint16_t)(UserPara.StopTimeBase >> 16);                 //总停机时长基值高位  	                                                             
            break;             
            
          case 0x48:
            Data_Buf = 0;//(uint16_t)UserPara.StopTimeBase;	                        //总停机时长基值低位                                                            
            break;

          case 0x49:
            Data_Buf = 0;
            break; 

          case 0x4A:
            Data_Buf = 0;
            break;

          case 0x4B: 
            Data_Buf = 0;        
            break;   
            
          case 0x4C:
            Data_Buf = 0;  
            break; 

          case 0x4D:
            Data_Buf = 0;  
            break;

          case 0x4E: 
            Data_Buf = 0;    
            break;   
                
          case 0x4F:
            Data_Buf = 0;  
            break;            
            
          default:
            Data_Buf = 0;
            break;
        }
        
       
          for (i = 2; i > 0; i--)
          {
              SendBuf[SendLen++] = (uint8_t)(Data_Buf >> ((i - 1) * 8));
          }
        
      
    }
    MBASC_SendMsg_NoLimit(SendBuf, SendLen);                                      
}



//******************************************************************************
// 名称         : MBASC_Fun04()
// 创建日期     : 2018-06-08
// 作者         : MMX
// 功能         : 04功能码，读双个寄存器操作
// 输入参数     : 无
// 输出参数     : 无
// 返回结果     : 无
// 注意和说明   : 无
// 修改内容     : 无 
//******************************************************************************
void MBASC_Fun04(void)	                                                        //读双个寄存器
{
    uint8_t i;
    uint32_t Data_Buf;
    uint16_t Register_Num ;
    uint8_t ReadAdrH;
    uint8_t ReadAdrL;
    
    ReadAdrH = UARTx_RXBuff[2];
    ReadAdrL = UARTx_RXBuff[3];
    
    Register_Num = ((uint16_t)UARTx_RXBuff[4] <<8) + UARTx_RXBuff[5];
  
    SendLen = 0;
    SendBuf[SendLen++] = UARTx_RXBuff[0] ? UserPara.SlaveAddr : 0x00;
    SendBuf[SendLen++] = 0x04;
    SendBuf[SendLen++] = Register_Num * 2;		                        //数据长度

    if(  (!((   ( (ReadAdrL <= MBASC_INPUT_REG_REGION_END) && ((ReadAdrL + Register_Num) <= (MBASC_INPUT_REG_REGION_END + 2)))  )&& ((0 != Register_Num) && (0 == (Register_Num & 0x01)) && (0 == (ReadAdrL & 0x01)))))   ||   (ReadAdrH != UserPara.SlaveAddr))   
                         //子码地址小于等04功能码寄存器最大值       子码地址长度 +2 <= 04功能码寄存器最大值+2                              长度不为0                 长度为双数                     子码为双数                          从机地址相符
    {
        MBASC_SendErr(MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }

    for(uint8_t k = 0; k < Register_Num; ReadAdrL += 2, k += 2)
    {
        switch (ReadAdrL)
        {
          case 0x00:
             
              Data_Buf = UserPara.RotateSta;//旋转状态

              break;
          case 0x02:  
              Data_Buf = UserPara.DirSta;//旋转方向                                      
            break;
            
          case 0x04: 
              Data_Buf = UserPara.RotateSpeed;//旋转速度	                                                                     
            break;
            
          case 0x06: 
              Data_Buf = UserPara.WorkTime;//  累计运行时间                                      
            break;

          case 0x08: 
              Data_Buf = UserPara.TotalPulse;//  累计脉冲数量                                    
            break;            

          case 0x0A: 
              
              Data_Buf = UserPara.Duration;
                                 
            break; 

          case 0x0C: 
              Data_Buf = 0;//                                      
            break; 
            
          case 0x0E: 
              Data_Buf =  0;//                                   
            break;
            
          default:
            Data_Buf = 0; 
            break;
        }
        for(i = 4; i > 0; i--)
        {
            SendBuf[SendLen++] = (uint8_t)(Data_Buf >> ((i - 1) * 8));
        }
    }
    MBASC_SendMsg(SendBuf, SendLen);
}


//******************************************************************************
// 名称         : MBASC_Fun05()
// 创建日期     : 2019-12-17
// 作者         : LHL
// 功能         : 05功能码，//写单个线圈，用于标定液位空值和满值
// 输入参数     : 无
// 输出参数     : 无
// 返回结果     : 无
// 注意和说明   : 无
// 修改内容     : 无
//******************************************************************************
void MBASC_Fun05()   //写单个线圈
{
   // uint32_t index = 0;
    //uint16_t ReadAdr = 0;
  //  uint16_t Register_Num = 0;
    //uint16_t uint16_tTemp;
    uint8_t ReadAdrH, ReadAdrL;
    //uint8_t uTemp[2];

   // uint8_t uArray[12];
    BitAction bWriteParaFlag = Bit_RESET; 
    
    ReadAdrH = UARTx_RXBuff[2];
    ReadAdrL = UARTx_RXBuff[3];
        
  //  Register_Num = ((uint16_t)UARTx_RXBuff[4] << 8) + UARTx_RXBuff[5]; 
    
    SendLen = 0;
    SendBuf[SendLen++] = UARTx_RXBuff[0] ? UserPara.SlaveAddr : 0x00;
    SendBuf[SendLen++] = 0x05;
   // SendBuf[SendLen++] = Register_Num * 2;
                                                                                //如果读取范围溢出
    if ((!((((ReadAdrL >= MBASC_SINGLE_COIL_ADDR_BGEIN) && (ReadAdrL <= MBASC_SINGLE_COIL_ADDR_END)))))|| (ReadAdrH != UserPara.SlaveAddr))
    {
        MBASC_SendErr(MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }
 
    if(!(((UARTx_RXBuff[4] == 0xFF) && (UARTx_RXBuff[5] == 0x00))
              || ((UARTx_RXBuff[4] == 0x00) && (UARTx_RXBuff[5] == 0x00))))
    {
      MBASC_SendErr(MB_EX_ILLEGAL_DATA_VALUE);
      return;
    }
    
        switch (ReadAdrL)
        {
            case 0x50:   //标定低液位/高液位	
             if((UARTx_RXBuff[4] == 0x00) && (UARTx_RXBuff[5] == 0x00))//标定低液位
             {
             //  UserPara.LiquidAdMin = UserPara.AD_Value_Filter;
               // Unshort2Array(UserPara.LiquidAdMin, uTemp);
               //Flash_Write_MultiBytes(LIQUIDADMIN, uTemp, 2);                
             }
             else if((UARTx_RXBuff[4] == 0xFF) && (UARTx_RXBuff[5] == 0x00))//标定高液位
              {
             //  UserPara.LiquidAdMax = UserPara.AD_Value_Filter;
              // Unshort2Array(UserPara.LiquidAdMax, uTemp);
              // Flash_Write_MultiBytes(LIQUIDADMAX, uTemp, 2);  
              } 
               break;
         
            case 0x51:	
               if ((UARTx_RXBuff[4] == 0x00) && (UARTx_RXBuff[5] == 0x00))
                  {
                    Flash_Write_MultiBytes(RUN_ADDR_BASE, "\x02", 1);  //恢复出厂设置
                    ReadPara(); 
                    Flash_Write_MultiBytes(RUN_ADDR_BASE, "\x01", 1);  
                  }
               break;
                
            case 0x52:  //冰结设备/解冻设备
                           
                break;   
                
            case 0x53: //flash 写使能/农牧局禁止              
                break; 
                
            default:
                break;
        }
   
                                                                          //不管下发哪个基值，当前计数值清零
    if(bWriteParaFlag == Bit_SET)
    {
        bWriteParaFlag = Bit_RESET;
        //UserPara.Duration = 0;
        //UserPara.WorkTime = 0;
       // UserPara.StandbyTime = 0;
        //UserPara.StopTime = 0;
        //memset(uArray, 0x00, sizeof(uArray));
       // Flash_Write_MultiBytes(WORK_TIME, uArray, sizeof(uArray));
    }
    MBASC_SendMsg(UARTx_RXBuff, 6);
}

//******************************************************************************
// 名称         : MBASC_Fun10()
// 创建日期     : 2018-06-08
// 作者         : MMX
// 功能         : 10功能码，写多个寄存器操作，用于修改参数
// 输入参数     : 无
// 输出参数     : 无
// 返回结果     : 无
// 注意和说明   : 无
// 修改内容     : 无
//******************************************************************************
void MBASC_Fun10()
{
    uint32_t index = 0;
    //uint16_t ReadAdr = 0;
    uint16_t Register_Num = 0;
    uint16_t uint16_tTemp;
    uint8_t ReadAdrH, ReadAdrL;
    uint8_t uTemp[4];

   // uint8_t uArray[12];
    BitAction bWriteParaFlag = Bit_RESET; 
    
    ReadAdrH = UARTx_RXBuff[2];
    ReadAdrL = UARTx_RXBuff[3];
        
    Register_Num = ((uint16_t)UARTx_RXBuff[4] << 8) + UARTx_RXBuff[5]; 
    
    SendLen = 0;
    SendBuf[SendLen++] = UARTx_RXBuff[0] ? UserPara.SlaveAddr : 0x00;
    SendBuf[SendLen++] = 0x10;
    SendBuf[SendLen++] = Register_Num * 2;
                                                                                //如果读取范围溢出
    if ((!((((ReadAdrL >= MBASC_MUL_REG_REGION_BGEIN) && (ReadAdrL <= MBASC_MUL_REG_REGION_END)
            && (ReadAdrL + Register_Num <= (MBASC_MUL_REG_REGION_END + 1))))
            && ((0 != Register_Num)) && ((Register_Num * 2) == UARTx_RXBuff[6])))
            || (ReadAdrH != UserPara.SlaveAddr))
    {
        MBASC_SendErr(MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }

    for (uint8_t k = 0; k < Register_Num; ReadAdrL++, k++)
    {
        switch (ReadAdrL)
        {
            case 0x30:						                //设备地址
                UserPara.SlaveAddr = UARTx_RXBuff[7+index] * 256 + UARTx_RXBuff[8+index];
//                if((UserPara.SlaveAddr >= 0xB1) && (UserPara.SlaveAddr <= 0xBF))
                    Flash_Write_MultiBytes(SLAVE_ADDR, &UserPara.SlaveAddr, 1);
                break;
            case 0x31:                                                          //波特率
                UserPara.Baudrate = UARTx_RXBuff[7+index] * 256 + UARTx_RXBuff[8+index];
                Flash_Write_MultiBytes(BAUDRATE, &UserPara.Baudrate, 1);
//                Uart_Config_Init();
                break;
               
            case 0x32:	                                                        //奇偶校验
                UserPara.Parity = UARTx_RXBuff[7+index] * 256 + UARTx_RXBuff[8+index];
//                Flash_Write_MultiBytes(PARITY, &UserPara.Parity, 1);    
//                Uart_Config_Init();
                break;

            case 0x33: //保留                                                             
                break;

            case 0x34:  //补偿使能                                                           
                break;

            case 0x35:	//滤波系数
                //if((0x00 < UARTx_RXBuff[8+index]) && (0x04 > UARTx_RXBuff[8+index]))
                {
                 // UserPara.FilterLevel = UARTx_RXBuff[8+index];
                 // Switch_Fiter(UserPara.FilterLevel);
                }
               // Flash_Write_MultiBytes(FILTER_LEVEL, &UserPara.FilterLevel, 1);
           					                
                break;

            case 0x36:   //自动上报方式                                                           
                break;

            case 0x37:   //输出修正系数K                                                        
                break;

            case 0x38:   //输出修正系数B
                break;

            case 0x39:	//修改累计运行时间
                 uint16_tTemp =  UARTx_RXBuff[7 + index] * 256 + UARTx_RXBuff[8 + index];
                if(uint16_tTemp != 0xFFFF)                                           //发0xFFFF时不执行
                {
                    UserPara.WorkTime = uint16_tTemp;
                    long32Array(UserPara.WorkTime, uTemp);
                    Flash_Write_MultiBytes(WORK_TIME_BASE, uTemp, 4);
                }        
                break;

            case 0x3A: //修改累计脉冲数
                 uint16_tTemp =  UARTx_RXBuff[7 + index] * 256 + UARTx_RXBuff[8 + index];
                if(uint16_tTemp != 0xFFFF)                                           //发0xFFFF时不执行
                {
                    UserPara.TotalPulse = uint16_tTemp;
                    long32Array(UserPara.TotalPulse, uTemp);
                    Flash_Write_MultiBytes(PULSE_TOTAL_BASE, uTemp, 4);
                }        
                break; 

            case 0x3B:						                
                break;

            case 0x3C:						                
                break;

            case 0x3D:						             
            
                break;

            case 0x3E:						                
                break; 
           
            case 0x40:
                            
                break; 
        
        
            case 0x70:	   //车速            
                return;

            case 0x71:     //ACC状态            
                break;   
                
            case 0x72:        //0x72 0x73 总里程       
                break; 
                
            default:
                break;
        }
        index += 2;
    }                                                                           //不管下发哪个基值，当前计数值清零
    if(bWriteParaFlag == Bit_SET)
    {
        bWriteParaFlag = Bit_RESET;
        //UserPara.Duration = 0;
        //UserPara.WorkTime = 0;
       // UserPara.StandbyTime = 0;
        //UserPara.StopTime = 0;
        //memset(uArray, 0x00, sizeof(uArray));
       // Flash_Write_MultiBytes(WORK_TIME, uArray, sizeof(uArray));
    }
    MBASC_SendMsg(UARTx_RXBuff, 6);
}


//**************************************************************************************************
// 名称         : MBASC_Fun2B()
// 创建日期     : 2016-09-19
// 作者         : 梅梦醒
// 功能         : 用于读取设备序列号信息
// 输入参数     : 无
// 输出参数     : 无
// 返回结果     : 无
// 注意和说明   : 一起读5个对象时数据量很大，注意SendBuf的长度
// 修改内容     :
//**************************************************************************************************
void MBASC_Fun2B(void)
{
    uint16_t Object_Num = 0, ReadAdr = 0;
    uint8_t CompanyNameLen = 0;//ProductionCodeLen = 0, HardwareVersionLen = 0;
    uint8_t SoftwareVersionLen = 0;//DeviceIdLen = 0, CustomerCodeLen = 0;
    
    if(UserPara.SlaveAddr != UARTx_RXBuff[2])                                   //当从机地址改为非0x45时，也可以读写寄存器
    {
        ReadAdr = UARTx_RXBuff[2] * 256 + UARTx_RXBuff[3];
    }
    else
    {
        ReadAdr = 0x51 * 256 + UARTx_RXBuff[3];
    }
    Object_Num = UARTx_RXBuff[4] * 256 + UARTx_RXBuff[5];                       //读对象个数   
    
    SendLen = 0;
    SendBuf[SendLen++] = (MBASC_GetSlaveAddr(UARTx_RXBuff)) ? UserPara.SlaveAddr : 0x00;
    SendBuf[SendLen++] = 0x2B;
    SendBuf[SendLen++] = UARTx_RXBuff[4];
    SendBuf[SendLen++] = UARTx_RXBuff[5];                      
    
    if (!(((ReadAdr >= 0x51E0) && (ReadAdr <= 0x51E5) && (ReadAdr + Object_Num) <= (0x71E5 + 1))
        && (0 != Object_Num)))
    {
        MBASC_SendErr(MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }   
    
    for (uint8_t k = 0; k < Object_Num; ReadAdr++, k++)
    {
        switch (ReadAdr)
        {
        case 0x51E0:
            SendBuf[SendLen++] = 0xE0;
            //CompanyNameLen = I2C1_ReadByte(EEPROM_ADDRESS, COMPANY);            //读取第一个位置的数据的长度
            if((0 == CompanyNameLen) || (0x32 < CompanyNameLen))                //长度不正确
            {
                SendBuf[SendLen++] = 0x01;                                      //应答一个字节 0x00                             
                SendBuf[SendLen++] = 0x00;
            }
            else
            {
                SendBuf[SendLen++] = CompanyNameLen;
//                for (uint8_t i = 0; i < CompanyNameLen; i++)
//                {                             
//                    SendBuf[SendLen++] = I2C1_ReadByte(EEPROM_ADDRESS, COMPANY + 1 + i);
//                }
            }
            break;
        
        case 0x51E1:
            SendBuf[SendLen++] = 0xE1;
           // ProductionCodeLen = I2C1_ReadByte(EEPROM_ADDRESS, DEV_ENCODING);
//            if((0 == ProductionCodeLen) || (0x32 < ProductionCodeLen))        
//            {
//                SendBuf[SendLen++] = 0x01;                                      //应答一个字节 0x00                             
//                SendBuf[SendLen++] = 0x00;
//            }
//            else
//            {
//                SendBuf[SendLen++] = ProductionCodeLen;
//                for (uint8_t i = 0; i < ProductionCodeLen; i++)
//                {                             
//                    SendBuf[SendLen++] = I2C1_ReadByte(EEPROM_ADDRESS, DEV_ENCODING + 1 + i);
//                }
//            }
            break;
            
        case 0x51E2:
            SendBuf[SendLen++] = 0xE2;
//            HardwareVersionLen = I2C1_ReadByte(EEPROM_ADDRESS, HWVERSION);
//            if((0 == HardwareVersionLen) || (0x32 < HardwareVersionLen))      
//            {
//                SendBuf[SendLen++] = 0x01;                                      //应答一个字节 0x00                             
//                SendBuf[SendLen++] = 0x00;
//            }
//            else
//            {
//                SendBuf[SendLen++] = HardwareVersionLen;
//                for (uint8_t i = 0; i < HardwareVersionLen; i++)
//                {                             
//                    SendBuf[SendLen++] = I2C1_ReadByte(EEPROM_ADDRESS, HWVERSION + 1 + i);
//                }
//            }
            break;
            
        case 0x51E3:
            SendBuf[SendLen++] = 0xE3;  //查询软件版本
            SoftwareVersionLen = 8;//I2C1_ReadByte(EEPROM_ADDRESS, SFVERSION);
            if((0 == SoftwareVersionLen) || (0x32 < SoftwareVersionLen))       
            {
                SendBuf[SendLen++] = 0x01;                                      //应答一个字节 0x00                             
                SendBuf[SendLen++] = 0x00;
            } 
            else
            {
                SendBuf[SendLen++] = SoftwareVersionLen;
                for (uint8_t i = 0; i < SoftwareVersionLen; i++)
                {                             
                    SendBuf[SendLen++] = SensorSoftVersion[i+1];//(EEPROM_ADDRESS, SFVERSION + 1 + i);
                }
            }
            break;
            
        case 0x51E4:
            SendBuf[SendLen++] = 0xE4;
//            CustomerCodeLen = I2C1_ReadByte(EEPROM_ADDRESS, DEV_ID);
//            if((0 == CustomerCodeLen) || (0x32 < CustomerCodeLen))            //没有写过而去读则返回错误码
//            {
//                SendBuf[SendLen++] = 0x01;                                      //应答一个字节 0x00                             
//                SendBuf[SendLen++] = 0x00;
//            } 
//            else
//            {
//                SendBuf[SendLen++] = CustomerCodeLen;
//                for (uint8_t i = 0; i < CustomerCodeLen; i++)
//                {                             
//                    SendBuf[SendLen++] = I2C1_ReadByte(EEPROM_ADDRESS, DEV_ID + 1 + i);
//                }
//            }
            break;
            
        case 0x51E5:
            SendBuf[SendLen++] = 0xE5;
//            DeviceIdLen = I2C1_ReadByte(EEPROM_ADDRESS, CUSTOMERCODE);
//            if((0 == DeviceIdLen) || (0x32 < DeviceIdLen))                     //没有写过而去读则直接返回
//            {
//                SendBuf[SendLen++] = 0x01;                                      //应答一个字节 0x00                             
//                SendBuf[SendLen++] = 0x00;
//            } 
//            else
//            {
//                SendBuf[SendLen++] = DeviceIdLen;
//                for (uint8_t i = 0; i < DeviceIdLen; i++)
//                {                             
//                    SendBuf[SendLen++] = I2C1_ReadByte(EEPROM_ADDRESS, CUSTOMERCODE + 1 + i);
//                }
//            }            
           break;            
            
        default:
            break; 
        }
    }
    MBASC_SendMsg(SendBuf, SendLen);
}

                
//**************************************************************************************************
// ??         : MBASC_Fun41()
// ????     : 2016-09-19
// ??         : ???
// ??         : ????(?boot????)
// ????     : ?
// ????     : ?
// ????     : ?
// ?????   : 
// ????     :
//**************************************************************************************************
void Delay_1Ms(uint32_t cnt)
{
    cnt = cnt * 940;

    while (cnt--);
   
  
}

void MBASC_Fun41(void)
{
	uint8_t buf[1]={0xFF};
    uint16_t ReadAdr = 0;
    uint16_t DataLength = 0;
    uint8_t ReadData;
      
    ReadAdr = ((uint16_t)(UARTx_RXBuff[2]<< 8)) + UARTx_RXBuff[3];
    DataLength = ((uint16_t)(UARTx_RXBuff[4]<< 8)) + UARTx_RXBuff[5];
        
    SendLen = 0;
//    SendBuf[SendLen++] = UARTx_RXBuff[0] ? UserPara.SlaveAddr : MB_ADDRESS_BROADCAST;
    SendBuf[SendLen++] = MBASC_GetSlaveAddr(UARTx_RXBuff);
	SendBuf[SendLen++] = 0x41;
    for(uint8_t i = 2; i < 6; i++)
    {
        SendBuf[SendLen++] = UARTx_RXBuff[i];
    }
                                                    //???????????boot
    if((0x0001 != ReadAdr) || (0 != DataLength) || (UARTx_RXBuff[0] == MB_ADDRESS_BROADCAST))
    {
        return;
    }
    else 
    {
	//	Flash_Write_OneByte(SOWAYFLASH_SRCPARA_ADDR,0xFF);
		Flash_Write_MultiBytes(RUN_ADDR_BASE, &buf, 1);
        SendBuf[SendLen++] = 0x00;
        MBASC_SendMsg(SendBuf, SendLen);
		Delay_1Ms(20);
        NVIC_SystemReset();

    }  
}     
     

//**************************************************************************************************
// ??         : MBASC_Function()
// ????     : 2016-09-05
// ??         : ???
// ??         : modbus ascii????
// ????     : ?
// ????     : ?
// ????     : ?
// ?????   : 
// ????     :  
//                                         
//**************************************************************************************************
void MBASC_Function(void)
{
    uint16_t RecvLen = 0;
    if(UartRecvFrameOK == SET)
    {
        if(2 == MODBUS_ASCII_RecvData(UARTx_RXBuff, &RecvLen))//????
        {
            return;
        }  

        if(RecvLen <= 0)
        {
            return;                                                                 //???????,???
        }

        else if((UserPara.SlaveAddr != UARTx_RXBuff[0])&& (MB_ADDRESS_BROADCAST != UARTx_RXBuff[0]))
        {
            return;                                                                 //??????????,???
        }

        else
        {
            switch (UARTx_RXBuff[1])
            {
              case 0x03:
                MBASC_Fun03();	                                                //??????(?????)
                break;

              case 0x04:
                MBASC_Fun04();	                                                //??????(????)
                break;
                
              case 0x10:
                MBASC_Fun10();                                                  //??????
                break;    

              case 0x2B:
                MBASC_Fun2B();                                                  //???????????
                break;    
     
              case 0x41:
                MBASC_Fun41();
                break; 
                
              default:
                SendLen = 0;
                SendBuf[SendLen++] = UARTx_RXBuff[0];
                SendBuf[SendLen++] = 0x80 | UARTx_RXBuff[1];
                SendBuf[SendLen++] = MB_EX_ILLEGAL_FUNCTION;
                MBASC_SendMsg(SendBuf, SendLen);
                break;
             }
         }
    }
//	else if(AutoUpLoad_flag)
//	{
//		
//		AutoUpLoad_flag = 0;
//		usart_disable(USART1);
	
//		usart_enable(USART1);
//	}

}


void MBASC_AutoUpLoadFrame(void)
{
    uint8_t     i;
    uint8_t     j;
    uint32_t    Data_Buf[3];
    
    SendLen = 0;
    

//	Data_Buf[0] = UserPara.Temp; 


//    Data_Buf[1] = UserPara.Duration;                                     

//    Data_Buf[2] = UserPara.AlarmSta;   
    
    for(j = 0; j < sizeof(Data_Buf) / sizeof(Data_Buf[0]); j++)
    {
        for(i = 4; i > 0; i--)
        {
            SendBuf[SendLen++] = (uint8_t)(Data_Buf[j] >> ((i - 1) * 8));
        }
    }
    MBASC_SendMsg_NoLimit(SendBuf, SendLen);
}




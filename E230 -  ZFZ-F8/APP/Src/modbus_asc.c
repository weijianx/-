
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


uint8_t const SensorSoftVersion[10] = {0x07, 'G', 'D', '1', '.', '0', '.', '0'};      //����汾  20200413


extern uint16_t Current_PositiveTime ;  //��ǰ��ת������
extern uint16_t Current_NegativeTime ;  //��ǰ��ת������

//**************************************************************************************************
// ����         	: MBASCII_GetSlaveAddr()
// ��������   	        : 2015-10-29
// ����         	: wang
// ����         	:����豸��ַ
// �������     	:���յĻ�������(uint8_t *uint8_tMsg)
// �������     	: �豸��ַ
// ���ؽ��     	: ��
// ע���˵��   	:
// �޸�����     	:
//**************************************************************************************************

uint32_t MBASC_GetSlaveAddr(uint8_t *uint8_tMsg)
{
    return(uint8_tMsg[0]);
}


//******************************************************************************
// ����         : MBASC_SendMsg()
// ��������     : 2018-06-08
// ����         : MMX
// ����         : ��ASCII��ʽ������Ϣ
// �������     : ֡������(uint8_t *uint8_tMsg),֡��(uint8_t uint8_tMsgLen)
// �������     : ��
// ���ؽ��     : ��
// ע���˵��   : ��
// �޸�����     : ��
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
// ����         : MBASC_SendMsg_NoLimit()
// ��������     : 2018-06-08
// ����         : MMX
// ����         : �������Ե���ASCII��ʽ������Ϣ
// �������     : ֡������(uint8_t *uint8_tMsg),֡��(uint8_t uint8_tMsgLen)
// �������     : ��
// ���ؽ��     : ��
// ע���˵��   : ��
// �޸�����     : ��
//******************************************************************************
void MBASC_SendMsg_NoLimit(uint8_t *uint8_tMsg, uint8_t uint8_tMsgLen)
{
   Delay_Ms(50);
    MODBUS_ASCII_SendData(uint8_tMsg, uint8_tMsgLen);
}

//******************************************************************************
// ����         : MBASC_SendErr()
// ��������     : 2016-06-08
// ����         : MMX
// ����         : ���ʹ��������Ӧ֡
// �������     : ������(uint8_t ErrCode)
// �������     : ��
// ���ؽ��     : ��
// ע���˵��   : ��
// �޸�����     : ��
//******************************************************************************
void MBASC_SendErr(uint8_t ErrCode)
{
    SendLen = 1;
    SendBuf[SendLen++] |= 0x80;
    SendBuf[SendLen++] = ErrCode;
    MBASC_SendMsg(SendBuf, SendLen);
}

//******************************************************************************
// ����         : MBASC_Fun03()
// ��������     : 2018-06-08
// ����         : MMX
// ����         : 03�����룬�������Ĵ�������
// �������     : ��
// �������     : ��
// ���ؽ��     : ��
// ע���˵��   : ��
// �޸�����     : ��
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
    SendBuf[SendLen++] = 0x03;	                        //������
    SendBuf[SendLen++] = Register_Num * 2;		                        //���ݳ���
                                                                                //�����ȡ��Χ���
    if ((!(((ReadAdrL >= MBASC_HOLDING_REG_REGION_BGEIN) && (ReadAdrL <= MBASC_HOLDING_REG_REGION_END)
        && (ReadAdrL + Register_Num <= (MBASC_HOLDING_REG_REGION_END + 1)))&& (0 != Register_Num)))
        || ((ReadAdrH != UserPara.SlaveAddr) && (ReadAdrH != MB_ADDRESS_BROADCAST)))//�㲥��ַ�ſ�
    {
        MBASC_SendErr(MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }

    for (uint8_t k = 0; k < Register_Num; ReadAdrL++, k++)
    {
        switch (ReadAdrL)
        {
          case 0x30:
            Data_Buf = UserPara.SlaveAddr;      //�豸��ַ
            break;

          case 0x31:
            Data_Buf = UserPara.Baudrate;	//��ѯ�����ʣ�			        
            break;

          case 0x32:
            Data_Buf = 3;		//��ѯ��żУ�飺	                        
            break;

          case 0x33:
            Data_Buf = 0;						        
            break;

          case 0x34:
            Data_Buf = 0;  //����ʹ��	                        
            break;

          case 0x35:                                                          
            Data_Buf = 0;	//�˲���ʽ	  		                                             
            break;

          case 0x36:
            Data_Buf = 0;	//�Զ��ϱ���ʽ	               
            break;

          case 0x37:            //�������ϵ��K
            Data_Buf = 0;                       
            break;

          case 0x38:             //�������ϵ��B
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
           // Data_Buf = UserPara.SensorRange;//;��������������                                                  
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
            Data_Buf = 0;//(uint16_t)(UserPara.WorkTimeBase >> 16);                 //�ܹ���ʱ����ֵ��λ                       
            break;

          case 0x44:
            Data_Buf = 0;//(uint16_t)UserPara.WorkTimeBase;                         //�ܹ���ʱ����ֵ��λ                       
            break;

          case 0x45:
            Data_Buf = 0;//(uint16_t)(UserPara.StandbyTimeBase >> 16);              //�ܴ���ʱ����ֵ��λ                                                            
            break;

          case 0x46:
            Data_Buf =0;//(uint16_t)UserPara.StandbyTimeBase;                       //�ܴ���ʱ����ֵ��λ                                                              
            break;  
            
          case 0x47:
            Data_Buf = 0;//(uint16_t)(UserPara.StopTimeBase >> 16);                 //��ͣ��ʱ����ֵ��λ  	                                                             
            break;             
            
          case 0x48:
            Data_Buf = 0;//(uint16_t)UserPara.StopTimeBase;	                        //��ͣ��ʱ����ֵ��λ                                                            
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
// ����         : MBASC_Fun04()
// ��������     : 2018-06-08
// ����         : MMX
// ����         : 04�����룬��˫���Ĵ�������
// �������     : ��
// �������     : ��
// ���ؽ��     : ��
// ע���˵��   : ��
// �޸�����     : �� 
//******************************************************************************
void MBASC_Fun04(void)	                                                        //��˫���Ĵ���
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
    SendBuf[SendLen++] = Register_Num * 2;		                        //���ݳ���

    if(  (!((   ( (ReadAdrL <= MBASC_INPUT_REG_REGION_END) && ((ReadAdrL + Register_Num) <= (MBASC_INPUT_REG_REGION_END + 2)))  )&& ((0 != Register_Num) && (0 == (Register_Num & 0x01)) && (0 == (ReadAdrL & 0x01)))))   ||   (ReadAdrH != UserPara.SlaveAddr))   
                         //�����ַС�ڵ�04������Ĵ������ֵ       �����ַ���� +2 <= 04������Ĵ������ֵ+2                              ���Ȳ�Ϊ0                 ����Ϊ˫��                     ����Ϊ˫��                          �ӻ���ַ���
    {
        MBASC_SendErr(MB_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }

    for(uint8_t k = 0; k < Register_Num; ReadAdrL += 2, k += 2)
    {
        switch (ReadAdrL)
        {
          case 0x00:
             
              Data_Buf = UserPara.RotateSta;//��ת״̬

              break;
          case 0x02:  
              Data_Buf = UserPara.DirSta;//��ת����                                      
            break;
            
          case 0x04: 
              Data_Buf = UserPara.RotateSpeed;//��ת�ٶ�	                                                                     
            break;
            
          case 0x06: 
              Data_Buf = UserPara.WorkTime;//  �ۼ�����ʱ��                                      
            break;

          case 0x08: 
              Data_Buf = UserPara.TotalPulse;//  �ۼ���������                                    
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
// ����         : MBASC_Fun05()
// ��������     : 2019-12-17
// ����         : LHL
// ����         : 05�����룬//д������Ȧ�����ڱ궨Һλ��ֵ����ֵ
// �������     : ��
// �������     : ��
// ���ؽ��     : ��
// ע���˵��   : ��
// �޸�����     : ��
//******************************************************************************
void MBASC_Fun05()   //д������Ȧ
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
                                                                                //�����ȡ��Χ���
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
            case 0x50:   //�궨��Һλ/��Һλ	
             if((UARTx_RXBuff[4] == 0x00) && (UARTx_RXBuff[5] == 0x00))//�궨��Һλ
             {
             //  UserPara.LiquidAdMin = UserPara.AD_Value_Filter;
               // Unshort2Array(UserPara.LiquidAdMin, uTemp);
               //Flash_Write_MultiBytes(LIQUIDADMIN, uTemp, 2);                
             }
             else if((UARTx_RXBuff[4] == 0xFF) && (UARTx_RXBuff[5] == 0x00))//�궨��Һλ
              {
             //  UserPara.LiquidAdMax = UserPara.AD_Value_Filter;
              // Unshort2Array(UserPara.LiquidAdMax, uTemp);
              // Flash_Write_MultiBytes(LIQUIDADMAX, uTemp, 2);  
              } 
               break;
         
            case 0x51:	
               if ((UARTx_RXBuff[4] == 0x00) && (UARTx_RXBuff[5] == 0x00))
                  {
                    Flash_Write_MultiBytes(RUN_ADDR_BASE, "\x02", 1);  //�ָ���������
                    ReadPara(); 
                    Flash_Write_MultiBytes(RUN_ADDR_BASE, "\x01", 1);  
                  }
               break;
                
            case 0x52:  //�����豸/�ⶳ�豸
                           
                break;   
                
            case 0x53: //flash дʹ��/ũ���ֽ�ֹ              
                break; 
                
            default:
                break;
        }
   
                                                                          //�����·��ĸ���ֵ����ǰ����ֵ����
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
// ����         : MBASC_Fun10()
// ��������     : 2018-06-08
// ����         : MMX
// ����         : 10�����룬д����Ĵ��������������޸Ĳ���
// �������     : ��
// �������     : ��
// ���ؽ��     : ��
// ע���˵��   : ��
// �޸�����     : ��
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
                                                                                //�����ȡ��Χ���
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
            case 0x30:						                //�豸��ַ
                UserPara.SlaveAddr = UARTx_RXBuff[7+index] * 256 + UARTx_RXBuff[8+index];
//                if((UserPara.SlaveAddr >= 0xB1) && (UserPara.SlaveAddr <= 0xBF))
                    Flash_Write_MultiBytes(SLAVE_ADDR, &UserPara.SlaveAddr, 1);
                break;
            case 0x31:                                                          //������
                UserPara.Baudrate = UARTx_RXBuff[7+index] * 256 + UARTx_RXBuff[8+index];
                Flash_Write_MultiBytes(BAUDRATE, &UserPara.Baudrate, 1);
//                Uart_Config_Init();
                break;
               
            case 0x32:	                                                        //��żУ��
                UserPara.Parity = UARTx_RXBuff[7+index] * 256 + UARTx_RXBuff[8+index];
//                Flash_Write_MultiBytes(PARITY, &UserPara.Parity, 1);    
//                Uart_Config_Init();
                break;

            case 0x33: //����                                                             
                break;

            case 0x34:  //����ʹ��                                                           
                break;

            case 0x35:	//�˲�ϵ��
                //if((0x00 < UARTx_RXBuff[8+index]) && (0x04 > UARTx_RXBuff[8+index]))
                {
                 // UserPara.FilterLevel = UARTx_RXBuff[8+index];
                 // Switch_Fiter(UserPara.FilterLevel);
                }
               // Flash_Write_MultiBytes(FILTER_LEVEL, &UserPara.FilterLevel, 1);
           					                
                break;

            case 0x36:   //�Զ��ϱ���ʽ                                                           
                break;

            case 0x37:   //�������ϵ��K                                                        
                break;

            case 0x38:   //�������ϵ��B
                break;

            case 0x39:	//�޸��ۼ�����ʱ��
                 uint16_tTemp =  UARTx_RXBuff[7 + index] * 256 + UARTx_RXBuff[8 + index];
                if(uint16_tTemp != 0xFFFF)                                           //��0xFFFFʱ��ִ��
                {
                    UserPara.WorkTime = uint16_tTemp;
                    long32Array(UserPara.WorkTime, uTemp);
                    Flash_Write_MultiBytes(WORK_TIME_BASE, uTemp, 4);
                }        
                break;

            case 0x3A: //�޸��ۼ�������
                 uint16_tTemp =  UARTx_RXBuff[7 + index] * 256 + UARTx_RXBuff[8 + index];
                if(uint16_tTemp != 0xFFFF)                                           //��0xFFFFʱ��ִ��
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
        
        
            case 0x70:	   //����            
                return;

            case 0x71:     //ACC״̬            
                break;   
                
            case 0x72:        //0x72 0x73 �����       
                break; 
                
            default:
                break;
        }
        index += 2;
    }                                                                           //�����·��ĸ���ֵ����ǰ����ֵ����
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
// ����         : MBASC_Fun2B()
// ��������     : 2016-09-19
// ����         : ÷����
// ����         : ���ڶ�ȡ�豸���к���Ϣ
// �������     : ��
// �������     : ��
// ���ؽ��     : ��
// ע���˵��   : һ���5������ʱ�������ܴ�ע��SendBuf�ĳ���
// �޸�����     :
//**************************************************************************************************
void MBASC_Fun2B(void)
{
    uint16_t Object_Num = 0, ReadAdr = 0;
    uint8_t CompanyNameLen = 0;//ProductionCodeLen = 0, HardwareVersionLen = 0;
    uint8_t SoftwareVersionLen = 0;//DeviceIdLen = 0, CustomerCodeLen = 0;
    
    if(UserPara.SlaveAddr != UARTx_RXBuff[2])                                   //���ӻ���ַ��Ϊ��0x45ʱ��Ҳ���Զ�д�Ĵ���
    {
        ReadAdr = UARTx_RXBuff[2] * 256 + UARTx_RXBuff[3];
    }
    else
    {
        ReadAdr = 0x51 * 256 + UARTx_RXBuff[3];
    }
    Object_Num = UARTx_RXBuff[4] * 256 + UARTx_RXBuff[5];                       //���������   
    
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
            //CompanyNameLen = I2C1_ReadByte(EEPROM_ADDRESS, COMPANY);            //��ȡ��һ��λ�õ����ݵĳ���
            if((0 == CompanyNameLen) || (0x32 < CompanyNameLen))                //���Ȳ���ȷ
            {
                SendBuf[SendLen++] = 0x01;                                      //Ӧ��һ���ֽ� 0x00                             
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
//                SendBuf[SendLen++] = 0x01;                                      //Ӧ��һ���ֽ� 0x00                             
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
//                SendBuf[SendLen++] = 0x01;                                      //Ӧ��һ���ֽ� 0x00                             
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
            SendBuf[SendLen++] = 0xE3;  //��ѯ����汾
            SoftwareVersionLen = 8;//I2C1_ReadByte(EEPROM_ADDRESS, SFVERSION);
            if((0 == SoftwareVersionLen) || (0x32 < SoftwareVersionLen))       
            {
                SendBuf[SendLen++] = 0x01;                                      //Ӧ��һ���ֽ� 0x00                             
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
//            if((0 == CustomerCodeLen) || (0x32 < CustomerCodeLen))            //û��д����ȥ���򷵻ش�����
//            {
//                SendBuf[SendLen++] = 0x01;                                      //Ӧ��һ���ֽ� 0x00                             
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
//            if((0 == DeviceIdLen) || (0x32 < DeviceIdLen))                     //û��д����ȥ����ֱ�ӷ���
//            {
//                SendBuf[SendLen++] = 0x01;                                      //Ӧ��һ���ֽ� 0x00                             
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




#include "SHT3x.h"
#include "TH_IIC.h"
//#include "usart.h"
#include "string.h"

#if 0
float gTemBuf[TEM_CH_NUM][TEM_FIL_NUM];                                        //4路温度滤波缓冲
float gHumBuf[HUM_CH_NUM][HUM_FIL_NUM];                                        //4路湿度滤波缓冲 
SensorSta_Typedef  SensorSta[2*SENSOR_NUM] = {NExist};                          //4路温度4路湿度的状态（共4对）  
SHT3_Typedef T_H;


//检查传感器的状态，在modbus中调用，用来检查传感器是否掉线
SensorSta_Typedef Check_Status(uint8_t SlaveAddr)
{
    if((SlaveAddr >= 0x21) && (SlaveAddr <= 0x28))
    {
        return  SensorSta[SlaveAddr - 0x21];            
    }
    return NExist;
}



//获得超阀值持续时间，4个字节，0x00010000表示高温(高湿度)报警；0x00000001表示低温(低湿度)报警
void Get_Duration(void)
{  
    static uint32_t TemCountCnt1 = 0,TemCountCnt2 = 0;                                          //必须初始化为0
    uint8_t  i,k;
    
    for(i = 0; i < SENSOR_NUM; i++)
    {
        if(T_H.Temp[i] > T_H.UpThreshold[i])                                   //高于最高值
        {
            if(!(T_H.AlarmSta[i] & 0x00010000))                                   //从正常状态跳转到高于高值
            {
                if(TemCountCnt1++ >= T_H.DurThreshold[i] - 1)                            //如果持续超过阀值时间
                {         
                    TemCountCnt1 = 0;
                    T_H.AlarmSta[i] |= 0x00010000;                                //报警
                    T_H.AlarmSta[i] &= 0x00010000;
                }
            }
            else                                                                   //持续时间+1s
            {
                TemCountCnt1 = 0;                                                    //如果在阈值附近抖动，计数值重新开始
                T_H.Duration[i] += 1;
            }
        }
        else if(T_H.Temp[i] < T_H.DoThreshold[i])                               //低于最低值
        {
            if(!(T_H.AlarmSta[i] & 0x00000001))                                   //从正常状态跳转到低于低值
            {
                if(TemCountCnt1++ >= T_H.DurThreshold[i] - 1)                            //如果持续超过阀值时间
                {
                    TemCountCnt1 = 0;
                    T_H.AlarmSta[i] |= 0x00000001;                                //报警
                    T_H.AlarmSta[i] &= 0x00000001;
                }
            }
            else                         
            {
                TemCountCnt1 = 0;                                                    //如果在阈值附近抖动，计数值重新开始
                T_H.Duration[i] += 1;                                             //持续时间+1s
            }
        }
        else                                                                        //在阈值范围内
        {
            if(T_H.AlarmSta[i] != 0)                                              //从报警状态切换到正常状态
            {
                T_H.Duration[i] += 1;                                             //先要继续+1s
                if(TemCountCnt1++ >= T_H.DurThreshold[i] - 1)                            //如果持续超过阀值时间
                {
                    TemCountCnt1 = 0;
                    T_H.AlarmSta[i] = 0;
                    T_H.Duration[i] = 0;
                }
            }
            else
            {   
                TemCountCnt1 = 0;                                                    //如果在阈值附近抖动，计数值重新开始
                T_H.AlarmSta[i] = 0;
                T_H.Duration[i] = 0;
            }
        } 
    }

    
     for(i = SENSOR_NUM; i < 2 * SENSOR_NUM; i++)
    {
       k = i - SENSOR_NUM;
      if(T_H.Humi[k]  > T_H.UpThreshold[i])                                   //高于最高值
        {
            if(!(T_H.AlarmSta[i] & 0x00010000))                                   //从正常状态跳转到高于高值
            {
                if(TemCountCnt2++ >= T_H.DurThreshold[i] - 1)                            //如果持续超过阀值时间
                {         
                    TemCountCnt2 = 0;
                    T_H.AlarmSta[i] |= 0x00010000;                                //报警
                    T_H.AlarmSta[i] &= 0x00010000;
                }
            }
            else                                                                   //持续时间+1s
            {
                TemCountCnt2 = 0;                                                    //如果在阈值附近抖动，计数值重新开始
                T_H.Duration[i] += 1;
            }
        }
        else if(T_H.Humi[k]  < T_H.DoThreshold[i])                               //低于最低值
        {
            if(!(T_H.AlarmSta[i] & 0x00000001))                                   //从正常状态跳转到低于低值
            {
                if(TemCountCnt2++ >= T_H.DurThreshold[i] - 1)                            //如果持续超过阀值时间
                {
                    TemCountCnt2 = 0;
                    T_H.AlarmSta[i] |= 0x00000001;                                //报警
                    T_H.AlarmSta[i] &= 0x00000001;
                }
            }
            else                         
            {
                TemCountCnt2 = 0;                                                    //如果在阈值附近抖动，计数值重新开始
                T_H.Duration[i] += 1;                                             //持续时间+1s
            }
        }
        else                                                                        //在阈值范围内
        {
            if(T_H.AlarmSta[i] != 0)                                              //从报警状态切换到正常状态
            {
                T_H.Duration[i] += 1;                                             //先要继续+1s
                if(TemCountCnt2++ >= T_H.DurThreshold[i] - 1)                            //如果持续超过阀值时间
                {
                    TemCountCnt2 = 0;
                    T_H.AlarmSta[i] = 0;
                    T_H.Duration[i] = 0;
                }
            }
            else
            {              
                TemCountCnt2 = 0;                                                    //如果在阈值附近抖动，计数值重新开始
                T_H.AlarmSta[i] = 0;
                T_H.Duration[i] = 0;
            }
        } 
      
      
    }
}


//初始化滤波数组以及设置SHT3的转换速率
void SHT3_Init(void)
{
    uint8_t i, j, sta;
    for(i = 0; i < SENSOR_NUM; i++)
    {
        for(j = 0; j < FILTER_NUM; j++)
        {
            gTemBuf[i][j] = 25.0f;                                              //温度缓存初始设置为25度
            gHumBuf[i][j] = 50.0f;                                              //湿度缓存初始设置为50%
        }
    }
    for(i = 0; i < SENSOR_NUM; i++)
    {
        sta = Set_Sht3_Periodi(i, SPERIODIC_MODE_SET_4_0_LOW);                  //设置转换率高一点，否则如果读快了会没应答
        if(sta)
        {
            SensorSta[i + SENSOR_NUM] = SensorSta[i] = NExist;                  //IIC状态错误
        }
        else
        {
            SensorSta[i + SENSOR_NUM] = SensorSta[i] = Exist;                   //IIC状态正常
        }
    }       
}



//**************************************************************************************************
// 名称         : SHT3x_WriteData()
// 创建日期     : 2017-09-21
// 作者         : MMX
// 功能         : 向SHT3x写数据，即发送指令
// 输入参数     : uint8_t DriverAddr      IIC器件的地址
//                uint16_t Command        发送的指令     
// 输出参数     : 无
// 返回结果     : 传感器应答状态
// 注意和说明   : 
// 修改内容     : 
//**************************************************************************************************
uint8_t SHT3x_WriteData(uint8_t DriverAddr, uint16_t Command)
{
    uint8_t sta = 0;
   // CPU_SR_ALLOC();
   // OS_CRITICAL_ENTER();
    TH_IIC_Start();                                                                //起始信号
    TH_IIC_Send_Byte(DriverAddr);	                                                //发送写指令
    sta += TH_IIC_Wait_Ack();                                                      //应答
    TH_IIC_Send_Byte((Command >> 8) & 0xff);                                       //发送指令高字节
    sta += TH_IIC_Wait_Ack();                                                      //应答
    TH_IIC_Send_Byte(Command & 0xff);                                              //发送指令低字节
    sta += TH_IIC_Wait_Ack();                                                      //应答
    TH_IIC_Stop();                                                                 //停止信号
   // OS_CRITICAL_EXIT();
    return sta;
}



//**************************************************************************************************
// 名称         : SHT3x_ReadData()
// 创建日期     : 2017-09-21
// 作者         : MMX
// 功能         : 从SHT3x读数据
// 输入参数     : uint8_t DriverAddr      IIC器件的地址
//                uint16_t Command        发送的指令 
//                uint8_t* DataBuff       数据缓存 
//                uint8_t DataLen         接收数据长度    
// 输出参数     : 无
// 返回结果     : 传感器应答状态
// 注意和说明   : 
// 修改内容     : 
//**************************************************************************************************
uint8_t SHT3x_ReadData(uint8_t DriverAddr, uint16_t Command, uint8_t* DataBuff, uint8_t DataLen)
{
    uint8_t i;
    uint8_t sta = 0;
   // CPU_SR_ALLOC();
    //OS_CRITICAL_ENTER();
    TH_IIC_Start();                                                             //起始信号
    TH_IIC_Send_Byte(DriverAddr);	                                            //发送写指令
    sta += TH_IIC_Wait_Ack();                                                   //应答
    TH_IIC_Send_Byte((Command >> 8) & 0xff);                                    //发送指令高字节
    sta += TH_IIC_Wait_Ack();                                                   //应答
    TH_IIC_Send_Byte(Command & 0xff);                                           //发送指令低字节
    sta += TH_IIC_Wait_Ack();                                                   //应答
	

    TH_IIC_Start();                                                             //起始信号
    TH_IIC_Send_Byte(DriverAddr + 1);	                                        //发送读指令
    sta += TH_IIC_Wait_Ack();                                                   //应答

    for(i = 0; i < DataLen - 1; i++)                                            //开始读取数据
    {
        *(DataBuff + i) = TH_IIC_Read_Byte(1);                                  //前几个字节需要应答                                           
    }
    *(DataBuff + i)  = TH_IIC_Read_Byte(0);                                     //最后一个字节不需要应答
    TH_IIC_Stop();
    //OS_CRITICAL_EXIT();
    return sta;
}


uint8_t Set_Sht3_Periodi(uint8_t sChannal, uint16_t sPeriodi)
{
    uint8_t sStatus;
    Get_Channal_Pin(sChannal);
    sStatus = SHT3x_WriteData(SHT3X_ADDR, SPERIODIC_MODE_SET_4_0_LOW);
    return sStatus;
}


uint8_t Read_Sht3_TH(uint8_t sChannal, uint16_t sReadCmd, uint8_t *sReadBuf, uint16_t sReadLen)
{
    uint8_t sStatus;
    Get_Channal_Pin(sChannal);
    sStatus = SHT3x_ReadData(SHT3X_ADDR, sReadCmd, sReadBuf, sReadLen);
    return sStatus;    
}


//******************************************************************************
// 名称         : Filter()
// 创建日期     : 2017-09-022
// 作者         : 梅梦醒
// 功能         : 排序并求均值
// 输入参数     : float NewData             新产生的值
//                float* FilterBuf          二维数组的行地址 
//                uint16_t FilterNum             滤波数量    
//                uint16_t DelHeadNum            需要删除的最小的元素个数  
//                uint16_t DelTailNum            需要删除的最大的元素个数 
// 输出参数     : 无
// 返回结果     : 平均值
// 注意和说明   : 
// 修改内容     :
//******************************************************************************
float TH_Filter(float NewData, float* FilterBuf, uint16_t FilterNum, uint16_t DelHeadNum, uint16_t DelTailNum)
{
    int i, j;
    float filter_temp;
    float filter_sum = 0;
    float FilterArray[TEM_FIL_NUM];

    if(FilterNum != TEM_FIL_NUM)
        return 999.0f;
    if(DelHeadNum + DelTailNum >= FilterNum)
        return 888.0f;
                                                                                      
    for(i = 0; i < FilterNum - 1 ; i ++)                                        //FIFO把最新的数据放在数组最后一个元素中           
    {
        FilterBuf[i] = FilterBuf[i + 1];
    }
    FilterBuf[i] = NewData;
    
    memcpy(FilterArray, FilterBuf, sizeof(FilterArray));                       //接收数据(这个是必须的，不能直接对FilterBuf进行排序，这样会丢采样值)
                                                                                //复制的个数必须用sizeof
    for (i = 0; i < DelTailNum; i ++)                                           //把最大的N个值移到最后面                                    
    {
        for (j = 0; j < FilterNum - 1 - i; j ++)                                
        {
            if(FilterArray[j] > FilterArray[j + 1])
            {
                filter_temp = FilterArray[j];
                FilterArray[j] = FilterArray[j + 1];
                FilterArray[j + 1] = filter_temp;
            }
        }
    }
           
    for(i = 0; i < DelHeadNum; i ++)                                            //把最小的M个值移到最前面                                      
    {
        for(j = FilterNum - DelTailNum - 1 ; j > i; j --)
        {
            if(FilterArray[j - 1] > FilterArray[j])                             //这样的好处是只需要排序N+M次，不需对整个数组排序
            {
                filter_temp = FilterArray[j - 1];
                FilterArray[j - 1] = FilterArray[j];
                FilterArray[j] = filter_temp;
            }
        }
    }
     
    for(i = DelHeadNum; i < FilterNum - DelTailNum; i ++)                      //去掉最小和最大的值求均值
        filter_sum += FilterArray[i];
    
    filter_sum /= (FilterNum - (DelHeadNum + DelTailNum));
    
    memset(FilterArray, 0x00, sizeof(FilterArray));                             //最好是清0一下，清零的数据个数必须用sizeof
    
    return filter_sum;

}


void ReadTH(void)
{
   // OS_ERR err;
    uint8_t i;
    uint8_t status;
    uint8_t sDataBuff[6];
    uint16_t sTH[2];
    float sTem[SENSOR_NUM], sHum[SENSOR_NUM];

   // TH_IIC_Init();
   // SHT3_Init();
    while(1)
    {
     //   OSTimeDlyHMSM(0u, 0u, 1u, 0u, OS_OPT_TIME_HMSM_STRICT, &err);
       // if(err == OS_ERR_NONE)
       // {
            for(i = 0; i < SENSOR_NUM; i++)
            {
                status = Read_Sht3_TH(i, SPERIODIC_MODE_FETCH_DATA_CMD, sDataBuff, sizeof(sDataBuff));
                
                if(status)
                {
                    SensorSta[i] = NExist;
                    SensorSta[i + SENSOR_NUM] = SensorSta[i];
                }
                else
                {
                    SensorSta[i] = Exist;
                    SensorSta[i + SENSOR_NUM] = SensorSta[i];
                    
                    sTH[0] = ((uint16_t)sDataBuff[0] << 8) | sDataBuff[1];      //读出的温度数组
                    sTH[1] = ((uint16_t)sDataBuff[3] << 8) | sDataBuff[4];      //读出的湿度数组
                    sTem[i] = -45.0f + 175.0f * sTH[0] / 65535.0f;              //计算成摄氏度
                    sHum[i] = 100.0f * sTH[1] / 65535.0f;                       //计算成百分比
                    sTem[i] = TH_Filter(sTem[i], gTemBuf[i], TEM_FIL_NUM, DEL_HEAD_NUM, DEL_TAIL_NUM);//温度滤波
                    sHum[i] = TH_Filter(sHum[i], gHumBuf[i], HUM_FIL_NUM, DEL_HEAD_NUM, DEL_TAIL_NUM);//湿度滤波
                    T_H.Temp[i] = (uint16_t)((sTem[i] + 273.1f) * 10.0f + 0.5f);
                    T_H.Humi[i] = (uint16_t)(sHum[i] * 10.0f + 0.5f);
                }
            }
            Get_Duration();
            break;
            //Re_Connect_Handler();
       // }
    }
}
#endif


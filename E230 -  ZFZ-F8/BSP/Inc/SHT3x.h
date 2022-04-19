#ifndef _SHT3X_H
#define _SHT3X_H

#include "gd32e230.h"
//#include "stm32l0xx.h"

#define SHT3X_ADDR                                              0x88            //默认地址为0x44，前7个bits为地址，最后一个bit为读/写，所以要左移7位 

#define SPERIODIC_MODE_FETCH_DATA_CMD                           0xE000          //周期模式下获取数据指令 
#define SPERIODIC_MODE_ART_CMD                                  0x2B32          //周期模式加速响应时间指令        
#define SPERIODIC_MODE_BREAK_CMD                                0x3093          //停止/中断周期模式指令
#define SOFTWARE_RESET_CMD                                      0x30A2          //芯片软件重启指令,发指令重启
#define HARDWARE_RESET_CMD                                      0x0006          //硬件重启，拉低nReset引脚进行重启(关闭电源是另一种形式的硬件重启)       
#define HEATER_ENABLE_CMD                                       0x306D          //内部加热器使能
#define HEATER_DISABLE_CMD                                      0x3066          //内部加热器失能
#define READ_REG_STATUS_CMD                                     0xF32D          //读寄存器状态
#define CLEAR_REG_STATUS_CMD                                    0x3041          //清除寄存器状态

//模式选择指令，两个字节，单次突发模式，即发一次指令SHT3x转换一次，需要转换时间
#define SINGLE_MODE_CLOCK_ENABLE_HIGHT                          0x2C06          //时钟高等级别延伸使能低字节
#define SINGLE_MODE_CLOCK_ENABLE_MEDIUM                         0x2C0D          //时钟中等级别延伸使能低字节
#define SINGLE_MODE_CLOCK_ENABLE_LOW                            0x2C10          //时钟低等级别延伸使能低字节
 
#define SINGLE_MODE_CLOCK_DISABLE_HIGHT                         0x2400          //时钟高等级别延伸失能低字节
#define SINGLE_MODE_CLOCK_DISABLE_MEDIUM                        0x240B          //时钟中等级别延伸失能低字节
#define SINGLE_MODE_CLOCK_DISABLE_LOW                           0x2416          //时钟低等级别延伸失能低字节


//模式选择指令，两个字节，周期转换模式，即发一次指令SHT3x不停转换，转换完等待读取
#define SPERIODIC_MODE_SET_0_5_HIGHT                            0x2032          //周期为0.5mps高等级别低字节
#define SPERIODIC_MODE_SET_0_5_MEDIUM                           0x2024          //周期为0.5mps中等级别低字节
#define SPERIODIC_MODE_SET_0_5_LOW                              0x202F          //周期为0.5mps低等级别低字节

#define SPERIODIC_MODE_SET_1_0_HIGHT                            0x2130          //周期为1mps高等级别低字节
#define SPERIODIC_MODE_SET_1_0_MEDIUM                           0x2126          //周期为1mps中等级别低字节
#define SPERIODIC_MODE_SET_1_0_LOW                              0x212D          //周期为1mps低等级别低字节

#define SPERIODIC_MODE_SET_2_0_HIGHT                            0x2236          //周期为2mps高等级别低字节
#define SPERIODIC_MODE_SET_2_0_MEDIUM                           0x2220          //周期为2mps中等级别低字节
#define SPERIODIC_MODE_SET_2_0_LOW                              0x222B          //周期为2mps低等级别低字节

#define SPERIODIC_MODE_SET_4_0_HIGHT                            0x2334          //周期为4mps高等级别低字节
#define SPERIODIC_MODE_SET_4_0_MEDIUM                           0x2322          //周期为4mps中等级别低字节
#define SPERIODIC_MODE_SET_4_0_LOW                              0x2329          //周期为4mps低等级别低字节

#define SPERIODIC_MODE_SET_10_0_HIGHT                           0x2737          //周期为10mps高等级别低字节
#define SPERIODIC_MODE_SET_10_0_MEDIUM                          0x2721          //周期为10mps中等级别低字节
#define SPERIODIC_MODE_SET_10_0_LOW                             0x272A          //周期为10mps低等级别低字节

     

#define SENSOR_NUM      1
#define FILTER_NUM      10
#define TEM_CH_NUM      SENSOR_NUM                                              //温度通道数
#define HUM_CH_NUM      SENSOR_NUM                                              //湿度通道数
#define TEM_FIL_NUM     FILTER_NUM                                              //温度滤波数        
#define HUM_FIL_NUM     FILTER_NUM                                              //湿度滤波数
#define DEL_HEAD_NUM    3                                                       //去掉头部数据个数
#define DEL_TAIL_NUM    3                                                       //去掉尾部数据个数


typedef enum {NExist = 0x00, Exist}SensorSta_Typedef;                          //传感器状态


typedef struct
{
    uint16_t Temp[SENSOR_NUM];                                                  //温度（K氏）
    uint16_t Humi[SENSOR_NUM];                                                  //湿度（百分比）
    uint16_t UpThreshold[2*SENSOR_NUM];                                         //温湿度上阀值(0~3温度，4~7湿度)
    uint16_t DoThreshold[2*SENSOR_NUM];                                         //温湿度下阀值
    uint16_t DurThreshold[2*SENSOR_NUM];                                            //温湿度超阀值持续时间
    uint32_t Duration[2*SENSOR_NUM];
    
	   
    uint8_t  Upload_persist[2*SENSOR_NUM];                                      //??????
    uint8_t  dev_num;
    uint32_t AlarmSta[2*SENSOR_NUM];                                            //警报状态 
}SHT3_Typedef;  


void SHT3_Init(void);
void ReadTH(void);
void Get_Duration(void);
void APP_ReadTH(void);
SensorSta_Typedef Check_Status(uint8_t SlaveAddr);

uint8_t SHT3x_WriteData(uint8_t DriverAddr, uint16_t Command);
uint8_t SHT3x_ReadData(uint8_t DriverAddr, uint16_t Command, uint8_t* DataBuff, uint8_t DataLen);
uint8_t Set_Sht3_Periodi(uint8_t sChannal, uint16_t sPeriodi);
uint8_t Read_Sht3_TH(uint8_t sChannal, uint16_t sReadCmd, uint8_t *sReadBuf, uint16_t sReadLen);

#endif
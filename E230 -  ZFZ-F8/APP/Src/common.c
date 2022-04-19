#include "common.h"

void Delay_Ms(uint32_t cnt)
{
    cnt = cnt * 3200;
   

    while (cnt--);
}

void Delay_Us(uint32_t cnt)
{
    cnt = cnt * 8;
    //cnt = cnt * 2;
    while (cnt--);
}


uint8_t signs = 0,initial = 0;
//滤波
uint16_t Get_filter(uint16_t PCap_buf)
{
	uint8_t i=0,j=0;	
	uint32_t buf1 = 0,bufz[7]={0};
	static uint32_t buf_PCap[9]={0},buf2;
        uint32_t buf3;
//	static uint16_t buf3;
//	uint8_t signs = 0,initial = 0;
	
	if(signs >= 9)
		signs = 0;
	
	if(initial < 8)
	{
		buf2 = 0;
		initial++;
		buf_PCap[signs]=PCap_buf;
		for(i = 0;i <= signs;)
		{
			buf2+=buf_PCap[i++];
		}
		signs++;
        return PCap_buf;        
	}
	
			
		buf_PCap[signs]=PCap_buf;			//有效值
	
	j = signs++;
	for(i = 0;i<7;i++)
	{
		bufz[i] = buf_PCap[j];
		if(j <= 0)
			j = 9;
		j--;
	}
	
	for(i=0; i<7; i++)
	{
		for(j=0; j<7-i; j++)
		{
			if(bufz[j] > bufz[j+1])
			{
				buf1 = bufz[j];
				bufz[j] = bufz[j+1];
				bufz[j+1] = buf1;
			}	
		}
	}
	
//	buf2 = 0;

     buf3 = bufz[1]+bufz[2]+bufz[3]+bufz[4]+bufz[5];
//     buf3 = (uint32_t)(buf2 /4);

	return buf3;
}

//**************************************************************************************************
// 名称         : Unshort2Array()
// 创建日期     : 2018-06-08
// 作者         : MMX
// 功能         : unsigned short类型数据转换为数组
// 输入参数     : uint16_t SourceData       要转换的数据
// 输出参数     : uint8_t* Array            转换的数组
// 返回结果     : 无
// 注意和说明   : 无
// 修改内容     : 无 
//**************************************************************************************************
void Unshort2Array(uint16_t SourceData, uint8_t* Array)
{
    *Array = *((uint8_t*)&SourceData + 1);
    *(Array + 1) = *(uint8_t*)&SourceData;
}



//**************************************************************************************************
// 名称         : long32Array()
// 创建日期     : 2018-06-08
// 作者         : MMX
// 功能         : long类型数据转换为数组
// 输入参数     : uint32_t SourceData       要转换的数据
// 输出参数     : uint8_t* Array            转换的数组
// 返回结果     : 无
// 注意和说明   : 无
// 修改内容     : 无 
//**************************************************************************************************
void long32Array(uint32_t SourceData, uint8_t* Array)
{
  //  *Array = *((uint8_t*)&SourceData + 1);
   // *(Array + 1) = *(uint8_t*)&SourceData;
    
   *Array = *((uint8_t*)&SourceData + 3);
   *(Array + 1) = *((uint8_t*)&SourceData + 2);
   *(Array + 2) = *((uint8_t*)&SourceData + 1);
   *(Array + 3) = *(uint8_t*)&SourceData;

}


//**************************************************************************************************
// 名称         : Unlong2Array()
// 创建日期     : 2018-06-08
// 作者         : MMX
// 功能         : uint32_t类型数据转换为数组
// 输入参数     : uint32_t SourceData       要转换的数据
// 输出参数     : uint8_t* Array            转换的数组
// 返回结果     : 无
// 注意和说明   : 无
// 修改内容     : 无 
//**************************************************************************************************
void Unlong2Array(uint32_t SourceData, uint8_t* Array)
{
    uint8_t i;
    for(i = 0; i < 4; i++)
    {
        *(Array + i) = *((uint8_t*)&SourceData + 3 - i);
    }
}



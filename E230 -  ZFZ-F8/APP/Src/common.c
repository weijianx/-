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
//�˲�
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
	
			
		buf_PCap[signs]=PCap_buf;			//��Чֵ
	
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
// ����         : Unshort2Array()
// ��������     : 2018-06-08
// ����         : MMX
// ����         : unsigned short��������ת��Ϊ����
// �������     : uint16_t SourceData       Ҫת��������
// �������     : uint8_t* Array            ת��������
// ���ؽ��     : ��
// ע���˵��   : ��
// �޸�����     : �� 
//**************************************************************************************************
void Unshort2Array(uint16_t SourceData, uint8_t* Array)
{
    *Array = *((uint8_t*)&SourceData + 1);
    *(Array + 1) = *(uint8_t*)&SourceData;
}



//**************************************************************************************************
// ����         : long32Array()
// ��������     : 2018-06-08
// ����         : MMX
// ����         : long��������ת��Ϊ����
// �������     : uint32_t SourceData       Ҫת��������
// �������     : uint8_t* Array            ת��������
// ���ؽ��     : ��
// ע���˵��   : ��
// �޸�����     : �� 
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
// ����         : Unlong2Array()
// ��������     : 2018-06-08
// ����         : MMX
// ����         : uint32_t��������ת��Ϊ����
// �������     : uint32_t SourceData       Ҫת��������
// �������     : uint8_t* Array            ת��������
// ���ؽ��     : ��
// ע���˵��   : ��
// �޸�����     : �� 
//**************************************************************************************************
void Unlong2Array(uint32_t SourceData, uint8_t* Array)
{
    uint8_t i;
    for(i = 0; i < 4; i++)
    {
        *(Array + i) = *((uint8_t*)&SourceData + 3 - i);
    }
}



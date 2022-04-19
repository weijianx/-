#include "algorithm.h"

//******************************************************************************
// ����         : SortArrayExtreme()
// ��������     : 2016-09-05
// ����         : ׯ��Ⱥ
// ����         : �������ڵ�Ԫ�ذ���С�����˳������
// �������     : u16 Array[]               ����
//                const u32 ArraySize       ����ĳ���
//                const u32 SortHeadSize    ����ͷ��Ԫ�ظ���   
//                const u32 SortTailSize    ����β��Ԫ�ظ���
// �������     : ��
// ���ؽ��     : ��
// ע���˵��   : 
// �޸�����     :
//******************************************************************************
static void SortArrayExtreme(int Array[], const uint32_t ArraySize,
                      const uint32_t SortHeadSize, const uint32_t SortTailSize)
{
    uint32_t i, j;
    int temp;

    for (i = 0; i < SortTailSize; i++)
    {
        for (j = 0; j < ArraySize - i - 1; j++)
        {
            if (Array[j] > Array[j+1])
            {
                temp = Array[j];
                Array[j] = Array[j+1];
                Array[j+1] = temp;
            }
        }
    }

    for (i = 0; i < SortHeadSize; i++)
    {
        for (j = ArraySize - SortTailSize - 1 ; j > i; j--)
        {
            if (Array[j - 1] > Array[j])
            {
                temp = Array[j - 1];
                Array[j - 1] = Array[j];
                Array[j] = temp;
            }
        }
    }
}



//******************************************************************************
// ����         : GetAverage()
// ��������     : 2016-09-05
// ����         : ׯ��Ⱥ
// ����         : ȥ��������С���ֵ
// �������     : u16 Array[]               ����
//                const u32 ArraySize       ����ĳ���
//                const u32 SortHeadSize    ����ͷ��Ԫ�ظ���   
//                const u32 SortTailSize    ����β��Ԫ�ظ���
// �������     : ��
// ���ؽ��     : ƽ��ֵ
// ע���˵��   : 
// �޸�����     :
//******************************************************************************
static int GetAverage(int Array[], const uint32_t ArraySize,
               const uint32_t DelHeadSize, const uint32_t DelTailSize)
{
    long long int sum = 0;

    if ((DelHeadSize + DelTailSize) >= ArraySize)
    {
        return 0;
    }

    for (long i = DelHeadSize; i < ArraySize - DelTailSize; i++)
    {
        sum += Array[i];
    }

    return(sum / (ArraySize - DelHeadSize - DelTailSize));
}



//******************************************************************************
// ����         : GetDelExtremeAndAverage()
// ��������     : 2016-09-05
// ����         : ׯ��Ⱥ
// ����         : �������ֵ
// �������     : u16 Array[]               ����
//                const u32 ArraySize       ����ĳ���
//                const u32 SortHeadSize    ����ͷ��Ԫ�ظ���   
//                const u32 SortTailSize    ����β��Ԫ�ظ���
// �������     : ��
// ���ؽ��     : ƽ��ֵ
// ע���˵��   : 
// �޸�����     :
//******************************************************************************
//int GetDelExtremeAndAverage(int Array[], const uint32_t ArraySize,
//                            const uint32_t SortHeadSize, const uint32_t SortTailSize)
//{
//    SortArrayExtreme(Array, ArraySize, SortHeadSize, SortTailSize);
//    return(GetAverage(Array, ArraySize, SortHeadSize, SortTailSize));
//}


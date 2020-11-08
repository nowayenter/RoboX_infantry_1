#ifndef USER_LIB_H
#define USER_LIB_H
#include "main.h"

typedef __packed struct
{
    fp32 input;        //��������
    fp32 out;          //�������
    fp32 min_value;    //�޷���Сֵ
    fp32 max_value;    //�޷����ֵ
    fp32 frame_period; //ʱ����
} ramp_function_source_t;

//б��������ʼ��
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 max, fp32 min);

//б����������
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);

#endif

//
// Created by lr on 2021/5/9.
//

#include "data_process.h"

//电机初始位置  单位:度
//float start_pos[13] = {
//        0,
//
//        -6.353     ,  //侧摆关节1
//        -1.608 + 70,  //髋关节1
//        -0.090 + 10,  //膝关节1
//
//         0.995     ,  //侧摆关节2
//         0.272 + 70,  //髋关节2
//         0.038 + 10,  //膝关节2
//
//         0.860     ,  //侧摆关节3
//         1.487 - 70,  //髋关节3
//        -0.038 - 10,  //膝关节3
//
//         1.173     ,  //侧摆关节4
//        -1.612 - 70,  //髋关节4
//         9.783 - 10   //膝关节4
//};

float start_pos[13] = {
        0,

        0,    //侧摆关节1
        80,   //髋关节1
        80,   //膝关节1

        0,    //侧摆关节2
        80,   //髋关节2
        80,   //膝关节2

        0,    //侧摆关节3
        -80,  //髋关节3
        -80,  //膝关节3

        0,    //侧摆关节4
        -80,  //髋关节4
        -80   //膝关节4
};

PidTypeDef PositionPID[13];

int32_t tar_torque[13] = {0};
float   tar_pos[13]    = {0};

bool limit_err[13] = {false,false,false,false,false,false,
                      false,false,false,false,false,false,false};
//   1  0
//   3  2
//选择某条腿(0/1/2/3)进行调试
const int num = 1;
char start_flag = 0;

////motors enable
//bool motors_enable(bool need_enable_motor)
//{
//    if(enabled_count < 1 && need_enable_motor)
//    {
//        switch(enabled_count) {
//            case 0:
//                enabl_num = 1;
//            break;
//            case 1:
////                enabl_num = 3;
//                break;
//            case 2:
////                enabl_num = 5;
//                break;
//            case 3:
////                enabl_num = 6;
//                break;
//            case 4:
////                enabl_num = 8;
//                break;
//            case 5:
////                enabl_num = 9;
//                break;
//            case 6:
////                enabl_num = 11;
//                break;
//            case 7:
////                enabl_num = 12;
//                break;
//            default:
//                break;
//        }
//        printf("motor %d enable now!!!\n", enabl_num);
//        motor_enable(true, target[enabl_num], actual[enabl_num]);
//        return false;
//    }
//    return true;
//}

bool motors_enable(bool need_enable_motor)
{
    if(enabled_count < 12 && need_enable_motor)
    {
        printf("motor %d enable now!!!\n", enabled_count+1);
        motor_enable(true, target[enabled_count+1], actual[enabled_count+1]);
        return false;
    }
    return true;
}

void data_process()
{
    pidcontrol();
}

//相关参数初始化
void parameters_init()
{

    /******************************************** leg1 ********************************************/
    //slave1 PID parameters init
    float PosPidparameter_1_1[3] = {POS_PID_KP_1, POS_PID_KI_1, POS_PID_KD_1};
    PID_Init(&PositionPID[1], PID_POSITION, PosPidparameter_1_1, POS_MAXOUT_1, POS_MAXIOUT_1);

    //slave2 PID parameters init
    float PosPidparameter_1_2[3] = {POS_PID_KP_2, POS_PID_KI_2, POS_PID_KD_2};
    PID_Init(&PositionPID[2], PID_POSITION, PosPidparameter_1_2, POS_MAXOUT_2, POS_MAXIOUT_2);

    //slave3 PID parameters init
    float PosPidparameter_1_3[3] = {POS_PID_KP_3, POS_PID_KI_3, POS_PID_KD_3};
    PID_Init(&PositionPID[3], PID_POSITION, PosPidparameter_1_3, POS_MAXOUT_3, POS_MAXIOUT_3);
    /******************************************** leg1 ********************************************/

    /******************************************** leg2 ********************************************/
    //slave4 PID parameters init
    float PosPidparameter_2_1[3] = {POS_PID_KP_4, POS_PID_KI_4, POS_PID_KD_4};
    PID_Init(&PositionPID[4], PID_POSITION, PosPidparameter_2_1, POS_MAXOUT_4, POS_MAXIOUT_4);

    //slave5 PID parameters init
    float PosPidparameter_2_2[3] = {POS_PID_KP_5, POS_PID_KI_5, POS_PID_KD_5};
    PID_Init(&PositionPID[5], PID_POSITION, PosPidparameter_2_2, POS_MAXOUT_5, POS_MAXIOUT_5);

    //slave6 PID parameters init
    float PosPidparameter_2_3[3] = {POS_PID_KP_6, POS_PID_KI_6, POS_PID_KD_6};
    PID_Init(&PositionPID[6], PID_POSITION, PosPidparameter_2_3, POS_MAXOUT_6, POS_MAXIOUT_6);
    /******************************************** leg2 ********************************************/

    /******************************************** leg3 ********************************************/
    //slave7 PID parameters init
    float PosPidparameter_3_1[3] = {POS_PID_KP_7, POS_PID_KI_7, POS_PID_KD_7};
    PID_Init(&PositionPID[7], PID_POSITION, PosPidparameter_3_1, POS_MAXOUT_7, POS_MAXIOUT_7);

    //slave8 PID parameters init
    float PosPidparameter_3_2[3] = {POS_PID_KP_8, POS_PID_KI_8, POS_PID_KD_8};
    PID_Init(&PositionPID[8], PID_POSITION, PosPidparameter_3_2, POS_MAXOUT_8, POS_MAXIOUT_8);

    //slave9 PID parameters init
    float PosPidparameter_3_3[3] = {POS_PID_KP_9, POS_PID_KI_9, POS_PID_KD_9};
    PID_Init(&PositionPID[9], PID_POSITION, PosPidparameter_3_3, POS_MAXOUT_9, POS_MAXIOUT_9);
    /******************************************** leg3 ********************************************/

    /******************************************** leg4 ********************************************/
    //slave10 PID parameters init
    float PosPidparameter_4_1[3] = {POS_PID_KP_10, POS_PID_KI_10, POS_PID_KD_10};
    PID_Init(&PositionPID[10], PID_POSITION, PosPidparameter_4_1, POS_MAXOUT_10, POS_MAXIOUT_10);

    //slave11 PID parameters init
    float PosPidparameter_4_2[3] = {POS_PID_KP_11, POS_PID_KI_11, POS_PID_KD_11};
    PID_Init(&PositionPID[11], PID_POSITION, PosPidparameter_4_2, POS_MAXOUT_11, POS_MAXIOUT_11);

    //slave12 PID parameters init
    float PosPidparameter_4_3[3] = {POS_PID_KP_12, POS_PID_KI_12, POS_PID_KD_12};
    PID_Init(&PositionPID[12], PID_POSITION, PosPidparameter_4_3, POS_MAXOUT_12, POS_MAXIOUT_12);
    /******************************************** leg4 ********************************************/
}

//PID运算
uint8_t pidcontrol()
{
    for(int cnt = 1; cnt <= slave_count; cnt++)
    {
        //位置环
        tar_torque[cnt] = PID_Calc(&PositionPID[cnt], actual[cnt].act_position, tar_pos[cnt]);
        //电流环由驱动器完成
        target[cnt].tar_torque = 0;
//        target[cnt].tar_torque = (int16)tar_torque[cnt];
    }

    position_limit();
    return 1;
}

//在初始位置基础上限位正负15度
void position_limit()
{
    for(int cnt = 1; cnt <= slave_count; cnt++)
    {
//        if(actual[cnt].act_position < (start_pos[cnt]-10.0) && target[cnt].tar_torque < 0 ||
//            actual[cnt].act_position > (start_pos[cnt]+10.0) && target[cnt].tar_torque > 0)
        if(actual[cnt].act_position < (start_pos[cnt] - 35.0) ||
        actual[cnt].act_position > (start_pos[cnt] + 35.0) ||
        actual[cnt].act_torque > 350 || actual[cnt].act_torque < -350)
        {
            limit_err[cnt] = true;
            target[cnt].tar_torque = 0;
        }
    }
}

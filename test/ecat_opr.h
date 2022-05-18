#ifndef ECAT_OPR_HPP
#define ECAT_OPR_HPP

#include <cstdio>
#include <cstring>
#include <ctime>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <curses.h>
#include <cstdlib>

#include "ethercattype.h"
#include "../oshw/linux/nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

using namespace std;

//1度(单腿电机编码器    膝关节和髋关节)
#define ENCOLDER_TRANSFORM_AB (float)(1456.35555555 * 41.0)

//1度(单腿电机编码器    侧摆关节)
#define ENCOLDER_TRANSFORM_C  (float)(1456.35555555 * 81.0)

extern int    slave_count;
extern int    enabled_count;
extern int    enabl_num;
extern long   CycleCounter;
extern struct Data_send target[13];
extern struct Data_rec  actual[13];

struct Data_send {
    uint16  control;
    uint8   operation_mode;
    int32   tar_position;
    int32   tar_velocity;
    int16   tar_torque;
};
struct Data_rec {
    uint16  status;
    int8    act_operation_mode;
    float   act_position;
    float   act_velocity;
    int16   act_torque;
    int16   act_current;
};

Data_rec recieve_data(int num);
void writeOutputs(Data_send data_send, int num);
void Pdo_map(int cnt);
void soem_config(const char* ifname);
void motor_enable(bool exec, struct Data_send &data_send, struct Data_rec data_rec);
void ecat_communic();
void *ecatcheck(void *data);

#endif

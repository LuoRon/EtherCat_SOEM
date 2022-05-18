//
// Created by lr on 2021/5/9.
//
/*
 * slave 1 膝关节
 * slave 2 髋关节
 * slave 3 侧摆关节
 */

#ifndef SOEM_TEST_MAIN_H
#define SOEM_TEST_MAIN_H

#include <climits>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <string>

#include "kbd.h"
#include "data_process.h"
#include <lcm/lcm-cpp.hpp>
#include "../lcm-types/cpp/leg_data.hpp"
#include <cassert>

//ECAT主站网口号
//#define ECAT_PORT_NUM "enp3s0"
#define ECAT_PORT_NUM "eno1"


//是否需要使能电机
#define NEED_ENABLE_MOTOR 1
//是否跑正弦轨迹
#define SINE_TRAJECTORY   0

#define PI 3.141592653589793

#define TRIG_STEP 5000

float trig_knee[TRIG_STEP] = {0};
float trig_hip[TRIG_STEP] = {0};

int lcm_pub_flag = 0;

lcm::LCM leg_lcm;
leg_data legData;

void * ecatcommunic(void *data);
void key_detect();
bool Ecat_Init(const char *ecat_port_num);
std::string getLcmUrl(int64_t ttl);
void lcm_update();

#endif //SOEM_TEST_MAIN_H

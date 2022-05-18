#include "main.h"

int main(int argc, char *argv[]) {
    //三角函数值离散化
    for(int i = 0; i < TRIG_STEP; i++)
    {
        trig_knee[i] = 5.0 * sin(2.0 / TRIG_STEP * PI * (i + 1));
        trig_hip[i] = trig_knee[i];
    }

    Ecat_Init(ECAT_PORT_NUM);

//    leg_lcm = getLcmUrl(255);
//
//    if(!leg_lcm.good()) return -1;

    while(1)
    {
        usleep(200000);
        key_detect();
    }
    return 0;
}

void *ecatcommunic(void*)
{
    int i = 1000;
    int j = 0;
    int k = 2000;
    bool first_run = true;
    bool motor_dead_flag = false;
    int motor_dead_cnt = 60;

    parameters_init();

    while (1) {
        if(motor_dead_flag) {
            if(motor_dead_cnt--) {}
            else {
                throw std::runtime_error("\n****** ERR: one or more motors have dead! ******\n");
            }
        }
        osal_usleep(500);

        //ecat 数据收发
        ecat_communic();

        //部分参数初始化
        if(k) {
            --k;
            for(int cnt = 1; cnt <= slave_count; cnt++)
            {
                start_pos[cnt] = actual[cnt].act_position;
                tar_pos[cnt] = start_pos[cnt];
                target[cnt].tar_torque = 0;
            }
            continue;
        }

        if(!motors_enable(NEED_ENABLE_MOTOR)) continue;

        //等待电机回到初始位置
        if(start_flag < slave_count)
        {
            for(int cnt = 1; cnt <= slave_count; cnt++)
                if(fabs(actual[cnt].act_position - start_pos[cnt]) < 1.0) start_flag++;
            if(start_flag < slave_count) start_flag = 0;
        }

        //正弦(cst模式)
        if(SINE_TRAJECTORY && start_flag == slave_count)
        {
            for(int cnt = 1; cnt <= slave_count; cnt++)
            {
                if(cnt%3 == 2)
                {
                    tar_pos[cnt] = start_pos[cnt] + trig_knee[j];
                }
                if(cnt%3 == 3)
                {
                    tar_pos[cnt] = start_pos[cnt] + trig_hip[j];
                }
            }
            if(++j >= TRIG_STEP) j = 0;
        }

        //PID
        data_process();

        if(lcm_pub_flag > 0) {
            lcm_update();
            --lcm_pub_flag;
            if(lcm_pub_flag == 0) {
                printf("\nstop publish lcm!!!\n");
            }
        }

//        if(--i) continue;
//        i = 1000;
//        printf(    "\n*************** %lu ***************\n", CycleCounter);
        if(first_run) {
            if(!start_flag && NEED_ENABLE_MOTOR) printf("Wait for all motors moving to the start position!!!\n");
            else {
                first_run = false;
                printf("All motors have moved to the start position!!!\n");
            }
        }

        for(int cnt = 1; cnt <= slave_count; cnt++)
        {
            if((actual[cnt].status & 0x007f) != 0x37)
            {
                printf("\n********* motor %d has been disabled! *********\n", cnt);
                printf("The status word is:0x%x\n",actual[cnt].status);
                motor_dead_flag = true;
//                enabled_count = 0;
            }

//            printf("\n------------- slave %d -------------\n",cnt);
//            if(limit_err[cnt])
//            {
//                printf("Errer: motor %d position limit!!!\n", cnt);
//                limit_err[cnt] = false;
//            }
//            printf("The opr_mode   : %d\n"   , actual[cnt].act_operation_mode);
//            printf("The status     : 0x%x\n" , actual[cnt].status);
//            printf("The tar_pos    : %3.3f\n", tar_pos[cnt]);
//            printf("The fb_pos     : %3.3f\n", actual[cnt].act_position);
//            printf("The fb_vel     : %4.3f\n", actual[cnt].act_velocity);
//            printf("The tar_torque : %d\n"   , target[cnt].tar_torque);
//            printf("The fb_torque  : %d\n"   , actual[cnt].act_torque);
//            printf("The fb_current : %d\n"   , actual[cnt].act_current);
        }
    }
    return 0;
}

//键盘按键检测
void key_detect()
{
//    //如果运行正弦轨迹,就禁止使用键盘
//    if(SINE_TRAJECTORY || !start_flag)
//    {
//        return;
//    }
    static long key;
    key = kbd_read();
    switch(key)
    {
        case 's':
            lcm_pub_flag = 100;
            printf("\nstart publish lcm!!!\n");
            break;
        case 'o':
            lcm_pub_flag = false;
            printf("\nstop publish lcm!!!\n");
            break;
        case '0':
            break;
        case '1':
            tar_pos[num*3 + 1] += 0.5;  //侧
            break;
        case '3':
            tar_pos[num*3 + 1] -= 1;  //侧
            break;
        case '4':
            tar_pos[num*3 + 2] += 1;  //髋
            break;
        case '6':
            tar_pos[num*3 + 2] -= 1;  //髋
            break;
        case '7':
            tar_pos[num*3 + 3] += 1;  //膝
            break;
        case '9':
            tar_pos[num*3 + 3] -= 1;  //膝
            break;
        default:
            break;
    }
}

static OSAL_THREAD_HANDLE check_thread;
static OSAL_THREAD_HANDLE commoic_thread;

bool Ecat_Init(const char *ecat_port_num)
{
    //create ecatcheck thread
    osal_thread_create((void*)&check_thread, 128000, (void*)&ecatcheck, (void*) &ctime);

    soem_config(ecat_port_num);

    //create data change thread
    osal_thread_create((void*)&commoic_thread, 256000, (void*)&ecatcommunic, (void*) &ctime);

    return true;
}

void lcm_update() {
    for (int cnt = 0; cnt < 12; ++cnt) {
        legData.q[cnt] = actual[cnt].act_position;
        legData.qd[cnt] = actual[cnt].act_velocity;
        legData.tau_fb[cnt] = actual[cnt].act_torque;
        legData.tau_cmd[cnt] = target[cnt].tar_torque;
    }
    leg_lcm.publish("leg_data", &legData);
}

std::string getLcmUrl(int64_t ttl) {
    assert(ttl >= 0 && ttl <= 255);
    //数值转换字符串
    return "udpm://239.255.76.67:7667?ttl=255";
}

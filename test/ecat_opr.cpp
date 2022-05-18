#include "ecat_opr.h"

#define EC_TIMEOUTMON 500
#define IN_NUM  6
#define OUT_NUM  5
int32 in_pdo_map[IN_NUM+1] = {
        0,
        0x60410010,  //statusword
        0x60610008,  //Modes of operation display
        0x60640020,  //Position actual value
        0x606C0020,  //Velocity actual value
        0x60770010,  //Torque value
        0x60780010   //Current actual value
};
int32 out_pdo_map[OUT_NUM+1] = {
        0,
        0x60400010,  //Controlword
        0x60600008,  //Modes of operation
        0x607A0020,  //Target position
        0x60FF0020,  //Target velocity
        0x60710010   //Target torque
};

char    IOmap[8192];
int     expectedWKC;
boolean needlf;
boolean inOP;
uint8   currentgroup = 0;
int     slave_count;
int     wkc = 0;
struct  Data_send target[13];
struct  Data_rec  actual[13];

int     enabled_count = 0;
int     enabl_num = 0;
long    CycleCounter = 0;

struct Data_rec recieve_data(int num){
    struct Data_rec data_recieved{};
    int8_t map[16];
    for(unsigned int i=0;i<16;i++)
    {
        map[i] = ec_slave[num].inputs[i];
    }
    data_recieved.status              = *(uint16  *)(map + 0 );
    data_recieved.act_operation_mode  = *(int8    *)(map + 2 );
    data_recieved.act_position        = *(int32   *)(map + 3 );
    data_recieved.act_velocity        = *(int32   *)(map + 7 );
    data_recieved.act_torque          = *(int16   *)(map + 11);
    data_recieved.act_current         = *(int16   *)(map + 13);

    return data_recieved;
}

void writeOutputs(const struct Data_send data_send, int num){
    int8_t map[13] = {0};
    map[0]  = (data_send.control             ) & 0x00ff;
    map[1]  = (data_send.control        >> 8 ) & 0x00ff;

    map[2]  = (data_send.operation_mode      ) & 0x00ff;

    map[3]  = (data_send.tar_position        ) & 0x00ff;
    map[4]  = (data_send.tar_position   >> 8 ) & 0x00ff;
    map[5]  = (data_send.tar_position   >> 16) & 0x00ff;
    map[6]  = (data_send.tar_position   >> 24) & 0x00ff;

    map[7]  = (data_send.tar_velocity        ) & 0x00ff;
    map[8]  = (data_send.tar_velocity   >> 8 ) & 0x00ff;
    map[9]  = (data_send.tar_velocity   >> 16) & 0x00ff;
    map[10] = (data_send.tar_velocity   >> 24) & 0x00ff;

    map[11] = (data_send.tar_torque          ) & 0x00ff;
    map[12] = (data_send.tar_torque     >> 8 ) & 0x00ff;

    for(unsigned int i=0;i<13;i++)
    {
        ec_slave[num].outputs[i] = map[i];
    }
}

void Pdo_map(int cnt)
{
    static int8 num_pdo;

    //First setting 0 to both sync manager
    wkc +=ec_SDOwrite(cnt,0x1C12,0x00, FALSE, sizeof(num_pdo),&num_pdo,EC_TIMEOUTRXM);
    wkc +=ec_SDOwrite(cnt,0x1C13,0x00, FALSE, sizeof(num_pdo),&num_pdo,EC_TIMEOUTRXM);

    //setting TPDO
    for(num_pdo = 0; num_pdo <= IN_NUM; num_pdo++)
    {
        wkc += ec_SDOwrite(cnt,0x1A08,num_pdo, FALSE, sizeof(in_pdo_map[num_pdo]),&in_pdo_map[num_pdo],EC_TIMEOUTRXM);
    }
    num_pdo--;
    wkc += ec_SDOwrite(cnt,0x1A08,0x00, FALSE, sizeof(num_pdo),&num_pdo,EC_TIMEOUTRXM);

    //setting RPDO
    for(num_pdo = 0; num_pdo <= OUT_NUM; num_pdo++)
    {
        wkc += ec_SDOwrite(cnt,0x1608,num_pdo, FALSE, sizeof(out_pdo_map[num_pdo]),&out_pdo_map[num_pdo],EC_TIMEOUTRXM);
    }
    num_pdo--;
    wkc += ec_SDOwrite(cnt,0x1608,0x00, FALSE, sizeof(num_pdo),&num_pdo,EC_TIMEOUTRXM);

    //Finally setting the mapping
    uint16 map_1c12[2] = {0x0001, 0x1608};
    uint16 map_1c13[2] = {0x0001, 0x1A08};
    wkc += ec_SDOwrite(cnt, 0x1c12, 0x00, TRUE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTRXM);
    wkc += ec_SDOwrite(cnt, 0x1c13, 0x00, TRUE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTRXM);
}

void soem_config(const char* ifname)
{
    needlf = FALSE;
    inOP = FALSE;

    printf("Starting simple test\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */

        /** network discovery */
        if ( ec_config_init(FALSE) > 0 )
        {
            printf("%d slaves found and configured.\n",ec_slavecount);

            slave_count = ec_slavecount;

            for(int cnt = 1; cnt <= ec_slavecount; cnt++)
            {
                printf("Slave %d has CA? %s\n", cnt ,ec_slave[cnt].CoEdetails & ECT_COEDET_SDOCA ? "true":"false");

                /** CompleteAccess disabled for Elmo driver */
                ec_slave[cnt].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            /* wait for all slaves to reach PRE_OP state */
            do{
                ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);
            }
            while ((ec_slave[0].state != EC_STATE_PRE_OP));

            /** set PDO mapping */
            for(int cnt = 1; cnt <= ec_slavecount; cnt++)
            {
                Pdo_map(cnt);
            }

            /** if CA disable => automapping works */
            if(ec_config_map(&IOmap) != ec_slavecount*28)
            {
                printf("Pdo map err!!! The target IOmapSize is %d\n",ec_slavecount*28);
                exit(-1);
            }

            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            for(int cnt = 1; cnt <= ec_slavecount; cnt++)
            {
                printf("\nSlave:%d\n Name:%s\n Output size: %dbytes\n Input size: %dbytes\n State: 0x%x\n Delay: %d[ns]\n Has DC: %d\n",
                       cnt, ec_slave[cnt].name, ec_slave[cnt].Obytes, ec_slave[cnt].Ibytes,
                       ec_slave[cnt].state, ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
            }

            // locates dc slaves
            if(ec_configdc()){
                printf("configure DC success.\n");
            }else{
                printf("configure DC error. \n");
            }

            printf("Slaves mapped, state to SAFE_OP.\n");

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            /** going operational */
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            /* request OP state for all slaves */
            ec_writestate(0);

            int8 chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
                printf("Operational state reached for all slaves.\n");
                inOP = TRUE;

                for(int cnt = 1; cnt <= ec_slavecount; cnt++)
                {
//                    target[cnt].operation_mode = 0x08;  //csp位置模式
                    target[cnt].operation_mode = 0x0A;  //cst力矩模式
                    target[cnt].control        = 0x0080;
                }

                return;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(int cnt = 1; cnt <= ec_slavecount ; cnt++)
                {
                    if(ec_slave[cnt].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               cnt, ec_slave[cnt].state, ec_slave[cnt].ALstatuscode, ec_ALstatuscode2string(ec_slave[cnt].ALstatuscode));
                    }
                }
            }

            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

void* ecatcheck( void *ptr )
{
    int slave;

    while(1)
    {
        usleep(10000);

        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if(ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n",slave);
                        }
                    }
                    else if(!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n",slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if(!ec_slave[slave].state)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n",slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n",slave);
                    }
                }
            }
            if(!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
    }
}

void motor_enable(bool exec, struct Data_send &data_send, struct Data_rec data_rec)
{
    static bool error;
    static int  step = 0;
    static int  cyc_temp;

    if(error) {
        throw std::runtime_error("\n motor enable error!!! \n");
    }

    switch (step) {
        case 0:
            if(exec)
            {
                cyc_temp = (int)CycleCounter;
                step++;
            }
            break;
        case 1:
            printf("********* motor %d step 1 *********\n", enabled_count+1);
            data_send.control = 0x80;
            if ((data_rec.status & 0x004f) == 0x40 || (data_rec.status & 0x004f) == 0x41) {
                cyc_temp = (int)CycleCounter;
                step++;
            }
            break;
        case 2:
            printf("********* motor %d step 2 *********\n", enabled_count+1);
            data_send.control = 0x06;
            if ((data_rec.status & 0x006f) == 0x21) {
                cyc_temp = (int)CycleCounter;
                step++;
            }
            else if((int)CycleCounter - cyc_temp > 2000)
                error = true;
            break;
        case 3:
            printf("********* motor %d step 3 *********\n", enabled_count+1);
            data_send.control = 0x07;
            if ((data_rec.status & 0x006f) == 0x23) {
                cyc_temp = (int)CycleCounter;
                step++;
            }
            else if((int)CycleCounter - cyc_temp > 2000)
                error = true;
            break;
        case 4:
            printf("********* motor %d step 4 *********\n", enabled_count+1);
            data_send.control = 0x0f;
            if ((data_rec.status & 0x006f) == 0x27) {
                cyc_temp = (int)CycleCounter;
                step++;
            }
            else if((int)CycleCounter - cyc_temp > 2000)
                error = true;
            break;
        case 5:
            printf("********* motor %d step 5 *********\n", enabled_count+1);
            data_send.control = 0x1f;
            if ((data_rec.status & 0x007f) == 0x37)
            {
                cyc_temp = (int)CycleCounter;
                step++;
            }
            else if((int)CycleCounter - cyc_temp > 2000)
                error = true;
            break;
        case 6:
            printf("********* motor %d step 6 *********\n", enabled_count+1);
            enabled_count++;
            step = 0;
            if(!exec)
            {
                data_send.control = 0x80;
                step = 0;
                error = false;
            }
            break;
        default:
            break;
    }
}

void ecat_communic()
{
    CycleCounter++;
    float ratio = 0;

    //send data refresh
    for(int cnt = 1; cnt <= ec_slavecount; cnt++)
    {
        //将关节的目标角度及角速度值转换为编码器 cnts
        if(cnt%3) ratio = ENCOLDER_TRANSFORM_AB;  //膝关节和髋关节
        else ratio = ENCOLDER_TRANSFORM_C;        //侧摆关节
        target[cnt].tar_position = target[cnt].tar_position * (int32)ratio;
        target[cnt].tar_velocity = target[cnt].tar_velocity * (int32)ratio;

        writeOutputs(target[cnt], cnt);
    }

    /** send and receive */
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET);

    //data refresh
    for(int cnt = 1; cnt <= ec_slavecount; cnt++)
    {
        actual[cnt] = recieve_data(cnt);

        //将编码器 cnts 转换为关节角度及角速度值
        if(cnt%3 == 0 || cnt%3 == 2) ratio = ENCOLDER_TRANSFORM_AB;  //膝关节和髋关节
        else if(cnt%3 == 1) ratio = ENCOLDER_TRANSFORM_C;  //侧摆关节
        actual[cnt].act_position = (actual[cnt].act_position / ratio);
        actual[cnt].act_velocity = (actual[cnt].act_velocity / ratio);
    }

    //wkc检测
    if(wkc >= expectedWKC)
    {
        needlf = TRUE;
    }
    else
    {
        printf("wkc err: expectedWKC = %d, actualWKC = %d\n", expectedWKC, wkc);
    }
}

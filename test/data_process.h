//
// Created by lr on 2021/5/9.
//

#ifndef SOEM_TEST_DATA_PROCESS_H
#define SOEM_TEST_DATA_PROCESS_H

#include "ecat_opr.h"
#include "pid.h"

/*********** FR ***********/
//POSITION (侧摆关节)
#define POS_PID_KP_1  (400   )
#define POS_PID_KI_1  (0     )
#define POS_PID_KD_1  (5000  )
#define POS_MAXOUT_1  (300   )
#define POS_MAXIOUT_1 (0     )

//POSITION (髋关节)
#define POS_PID_KP_2  (300   )
#define POS_PID_KI_2  (0     )
#define POS_PID_KD_2  (5000  )
#define POS_MAXOUT_2  (200   )
#define POS_MAXIOUT_2 (0     )

//POSITION (膝关节)
#define POS_PID_KP_3  (250   )
#define POS_PID_KI_3  (0     )
#define POS_PID_KD_3  (3000  )
#define POS_MAXOUT_3  (200   )
#define POS_MAXIOUT_3 (0     )
/*********** FR ***********/

/*********** FL ***********/
//POSITION (侧摆关节)
#define POS_PID_KP_4  (300   )
#define POS_PID_KI_4  (0     )
#define POS_PID_KD_4  (5000  )
#define POS_MAXOUT_4  (300   )
#define POS_MAXIOUT_4 (0     )

//POSITION (髋关节)
#define POS_PID_KP_5  (300   )
#define POS_PID_KI_5  (0     )
#define POS_PID_KD_5  (5000  )
#define POS_MAXOUT_5  (300   )
#define POS_MAXIOUT_5 (0     )

//POSITION (膝关节)
#define POS_PID_KP_6  (250   )
#define POS_PID_KI_6  (0     )
#define POS_PID_KD_6  (3000  )
#define POS_MAXOUT_6  (200   )
#define POS_MAXIOUT_6 (0     )
/*********** FL ***********/

/*********** BR ***********/
//POSITION (侧摆关节)
#define POS_PID_KP_7  (300   )
#define POS_PID_KI_7  (0     )
#define POS_PID_KD_7  (5000  )
#define POS_MAXOUT_7  (300   )
#define POS_MAXIOUT_7 (0     )

//POSITION (髋关节)
#define POS_PID_KP_8  (300   )
#define POS_PID_KI_8  (0     )
#define POS_PID_KD_8  (5000  )
#define POS_MAXOUT_8  (150   )
#define POS_MAXIOUT_8 (0     )

//POSITION (膝关节)
#define POS_PID_KP_9  (250   )
#define POS_PID_KI_9  (0     )
#define POS_PID_KD_9  (3000  )
#define POS_MAXOUT_9  (200   )
#define POS_MAXIOUT_9 (0     )
/*********** BR ***********/

/*********** BL ***********/
//POSITION (侧摆关节)
#define POS_PID_KP_10  (300   )
#define POS_PID_KI_10  (0     )
#define POS_PID_KD_10  (5000  )
#define POS_MAXOUT_10  (300   )
#define POS_MAXIOUT_10 (0     )

//POSITION (髋关节)
#define POS_PID_KP_11  (300   )
#define POS_PID_KI_11  (0     )
#define POS_PID_KD_11  (5000  )
#define POS_MAXOUT_11  (150   )
#define POS_MAXIOUT_11 (0     )

//POSITION (膝关节)
#define POS_PID_KP_12  (250   )
#define POS_PID_KI_12  (0     )
#define POS_PID_KD_12  (3000  )
#define POS_MAXOUT_12  (200   )
#define POS_MAXIOUT_12 (0     )
/*********** BL ***********/

extern const int num;
extern char      start_flag;
extern int32_t   tar_torque[13];
extern float     tar_pos[13];
extern float     start_pos[13];
extern bool      limit_err[13];

bool    motors_enable(bool need_enable_motor);
void    data_process();
void    parameters_init();
uint8_t pidcontrol();
void    position_limit();

#endif //SOEM_TEST_DATA_PROCESS_H

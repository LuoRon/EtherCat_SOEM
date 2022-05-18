cmake_minimum_required(VERSION 3.5)

project(soem_test)

include_directories(
        soem
        osal
        oshw
        test
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set(
        SOURCE_FILES
        main.cpp
)

add_executable(
        soem_test
        test/main.cpp
        test/main.h
        osal/linux/osal.c
        osal/osal.h
        oshw/linux/oshw.c
        oshw/linux/oshw.h
        oshw/linux/nicdrv.c
        oshw/linux/nicdrv.h
        soem/ethercattype.h
        soem/ethercatbase.c
        soem/ethercatbase.h
        soem/ethercatmain.c
        soem/ethercatmain.h
        soem/ethercatdc.c
        soem/ethercatdc.h
        soem/ethercatcoe.c
        soem/ethercatcoe.h
        soem/ethercatfoe.c
        soem/ethercatfoe.h
        soem/ethercatconfig.h
        soem/ethercatconfig.c
        soem/ethercatprint.h
        soem/ethercatprint.c
        soem/ethercatsoe.c
        soem/ethercatsoe.h
#        lcm-types/cpp/leg_data.hpp

        test/kbd.h
        test/kbd_lnx.cpp
        test/pid.h
        test/pid.cpp
        test/ecat_opr.h
        test/ecat_opr.cpp
        test/data_process.h
        test/data_process.cpp
)

TARGET_LINK_LIBRARIES(
        soem_test
        lcm
)

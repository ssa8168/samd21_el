/*
 * data_config.h
 *
 * Created: 2019-12-05 오후 5:54:13
 *
 */ 


#ifndef DATA_CONFIG_H_
#define DATA_CONFIG_H_

#include <compiler.h>
#include <touch_api_SAMD.h>

// TEST_TOUCH_NUMBER  ( 0 ~ 14 )
#define TEST_TOUCH_NUMBER           10

// DELTA_AVERAGE    ( -32767 ~ 32767 )
#define DELTA_AVERAGE       161, 184, 222, 244, 258, 224, 201, 199, 220, 257  //,248, 189, 159, 255

// DELTA_THRESHOLD  ( -32767 ~ 32767 )
#define DELTA_THRESHOLD     50, 50, 50, 50, 50, 50, 50, 50, 50, 50  //, 100, 100, 100, 255


// DEF_SELFCAP_GAIN_PER_NODE  ( GAIN_1 ~ GAIN_32 : GAIN_1, GAIN_2, GAIN_4, GAIN_8, GAIN_16, GAIN_32 )
#define DEF_SELFCAP_GAIN_PER_NODE   GAIN_1, GAIN_1, GAIN_1, GAIN_1, GAIN_1, GAIN_1, GAIN_1, GAIN_1, GAIN_1, GAIN_1   //, GAIN_1, GAIN_1, GAIN_1, GAIN_1


// NOT_AVAIL_DELTA  ( 0 ~  ), +, - Boundary
#define OPEN_DELTA_VALUE    20

// LOG_SPEED   ( 1 ~ 50 )
#define LOG_SPEED   10   // Duration   1sec/LOG_SPEED -> log report time per sec


#endif /* DATA_CONFIG_H_ */
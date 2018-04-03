#ifndef global_vars_h
#define global_vars_h

/*

  PINS FOR ARDUINO MEGA

*/

/*                            */
/*    Pins for bottom motor   */
/*                            */

// 8 bit decoder
#define bit0_7 37
#define bit0_6 36
#define bit0_5 35
#define bit0_4 34
#define bit0_3 33
#define bit0_2 32
#define bit0_1 31
#define bit0_0 30

#define sel1Pin_0 26
#define OEPin_0 27

#define reset0 4

// Slot detector
#define enablePinA_0 8
#define dirPinA1_0 11
#define dirPinA2_0 10

// PID Limits
#define PID_UPPER_LIMIT 255
#define PID_LOWER_LIMIT -255
#define MOTOR_LOWER_LIMIT 170
#define MOTOR_UPPER_LIMIT 255

/*                            */
/*     Pins for top motor     */
/*                            */ 

// 8 bit decoder
#define bit1_7 53
#define bit1_6 52
#define bit1_5 51
#define bit1_4 50
#define bit1_3 49
#define bit1_2 48
#define bit1_1 47
#define bit1_0 46

// Sel and enable for 8 bit decoder
#define sel1Pin_1 42
#define OEPin_1 43

#define reset1 3

#define enablePinA_1 7
#define dirPinA1_1 6
#define dirPinA2_1 5

#define PID_UPPER_LIMIT_1 255
#define PID_LOWER_LIMIT_1 -255
#define MOTOR_LOWER_LIMIT_1 200 //215
#define MOTOR_UPPER_LIMIT_1 255

#endif



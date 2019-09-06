/*******************************************************************
************************DoRoBo32 SOURCE CODE************************
*******************************************************************/

#include <stdlib.h>       //Required for motor, FFT
#include <stdio.h>        //Required for tracing module
#include <stdint.h>       //Required for digital I/O, motor
#include "stdarg.h"       //Required for tracing module
#include "stm32f0xx_hal.h"//Required for digital & analog I/O, motor, trace module, FFT
#include "mxconstants.h"  //Required for digital I/O
#include "dorobo32.h"     //Required for FFT
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"   //Required for creating tasks
#include "task.h"

#include "trace.h"      //Required for tracing module
#include "string.h"     //Required for tracing module
#include "digital.h"    //Required for digital I/O, FFT
#include "adc.h"        //Required for analog I/O
#include "motor.h"      //Required for the motor
#include "fft.h"        //Required for FFT

#define _10MS       10/portTICK_PERIOD_MS	//Constant values to use
#define _20MS       20/portTICK_PERIOD_MS	//along the whole code
#define _50MS       50/portTICK_PERIOD_MS
#define _100MS      100/portTICK_PERIOD_MS
#define _200MS      200/portTICK_PERIOD_MS
#define _300MS      300/portTICK_PERIOD_MS
#define _500MS      500/portTICK_PERIOD_MS
#define _1000MS     1000/portTICK_PERIOD_MS
#define _2000MS     2000/portTICK_PERIOD_MS
#define DIST_RANGE  2500
#define MAXspeed    80
#define STEPspeed   MAXspeed/10   //I want to reach MAXspeed in 10 iterations
#define IR_RANGE    600

static void inputs(void *pvParameters);		//Function constructors
static void state_machine(void *pvParameters);
static void outputs(void *pvParameters);
void motorFORWARD();
void motorBACKWARD();
void motorRIGHT();
void motorLEFT();
void motorAROUND_RIGHT();
void motorAROUND_LEFT();
void motorSTOP();

bool switch_right = 1;		//Both mechanical switches work with negative logic,
bool switch_left = 1;		//so they must start with 1
uint32_t distance_right = 0;
uint32_t distance_left = 0;
uint16_t IR_right = 0;
uint16_t IR_left = 0;
int IR_detection = 0;
bool read_IR_sensor = 1;  //It must be 1 to read IR sensors
bool rightIR = 0;         //We start checking the left IR sensor

short estado = 0;	  //Definition of all possible states of the robot
enum STATES {
  STAND_BY,           // 0
  OBSTACLE,           // 1
  OBST_RIGHT_SWITCH,  // 2
  OBST_LEFT_SWITCH,   // 3
  OBST_RIGHT_DIST,    // 4
  OBST_LEFT_DIST,     // 5
  FORWARD,            // 6
  BACKWARD_RIGHT,     // 7
  BACKWARD_LEFT,      // 8
  TURN_RIGHT,         // 9
  TURN_LEFT,          // 10
};

int8_t forSpeed = 0;		//Variables for motor speed in different directions
int8_t backSpeed = 0;		//(they are global in order to be used in various tasks)
int8_t aroundRightSpeed = 0;
int8_t aroundLeftSpeed = 0;
int8_t turnRightSpeed = 0;
int8_t turnLeftSpeed = 0;

static void inputs(void *pvParameters) {	//Task to just refresh input sensors data
  while (1) {
    switch_right = digital_get_pin(DD_PIN_PD14);    //Mechanical switches are updated
    switch_left = digital_get_pin(DD_PIN_PC8);

    if (adc_get_value(DA_ADC_CHANNEL0) >= DIST_RANGE) {distance_right = 1;} //2500 aprox 12cm
    else {distance_right = 0;}   //Distance sensors updated with a pair of boolean variables
    if (adc_get_value(DA_ADC_CHANNEL1) >= DIST_RANGE) {distance_left = 1;}
    else {distance_left = 0;}

    if (read_IR_sensor) {   //Left and right IR sensors are read one after another
      if (rightIR) {ft_start_sampling(DD_PIN_PA8);} //Time to update the right IR remote sensor
      else {ft_start_sampling(DD_PIN_PC13);}
      read_IR_sensor = 0;
    }
    else if (ft_is_sampling_finished()) {		//Time to update the left IR remote sensor
      if (rightIR) {
        IR_right = ft_get_transform(DFT_FREQ100);
        rightIR = 0;  //Time for the left sensor
      }
      else {
        IR_left = ft_get_transform(DFT_FREQ100);
        rightIR = 1;  //Time for the right sensor
      }
      read_IR_sensor = 1;
      if (abs(IR_left-IR_right)<IR_RANGE) {IR_detection = 0;} //Objective is forward
      else if ((IR_left-IR_right)<0) {IR_detection = 1;}      //Objective is on the left
      else {IR_detection = -1;}                               //Objective is on the right
    }

    vTaskDelay(_10MS);   //Little delay to allow the other tasks to work

    /*tracef("Right: %i \n\r", IR_right);
    tracef("Left: %i \n\r", IR_left);
    tracef("Direction: %i \n\r", IR_detection);*/
  }
}
static void state_machine(void *pvParameters) {
  /*if !dip_1 --> STAND_BY			//State machine behaviour defined

    STAND_BY
      if dip_1 --> FORWARD
    OBSTACLE
      if (!switch_left) --> BACKWARD_RIGHT (either only left or both sensors, turn right)
      else if (switch_left && !switch_right) --> BACKWARD_LEFT
      else if (distance_left) --> OBST_LEFT_DIST (either only left or both sensors, turn right)
      else if (!distance_left && distance_right) --> OBST_RIGHT_DIST
      else if (previousState == BACKWARD_RIGHT) --> OBST_LEFT_SWITCH
      else if (previousState == BACKWARD_LEFT) --> OBST_RIGHT_SWITCH
      else --> FORWARD
    OBST_RIGHT_SWITCH
      After 300ms --> OBSTACLE
    OBST_LEFT_SWITCH
      After 300ms --> OBSTACLE
    OBST_RIGHT_DIST
      if (!switch_right || !switch_left || (!distance_left && !distance_right)) --> OBSTACLE
    OBST_LEFT_DIST
      if (!switch_right || !switch_left || (!distance_left && !distance_right)) --> OBSTACLE
    FORWARD
      if (!switch_right || !switch_left || distance_left || distance_right) --> OBSTACLE
      else if (IR_detection == 1) --> TURN_RIGHT
      else if (IR_detection == -1) --> TURN_LEFT
    BACKWARD_RIGHT
      After 500ms --> OBSTACLE
    BACKWARD_LEFT
      After 500ms --> OBSTACLE
    TURN_RIGHT
      if (!switch_right || !switch_left || distance_left || distance_right) --> OBSTACLE
      else if (IR_detection == 0) --> FORWARD
    TURN_LEFT
      if (!switch_right || !switch_left || distance_left || distance_right) --> OBSTACLE
      else if (IR_detection == 0) --> FORWARD
  */

  int previousState = 0;		//Current and previous robot states are checked
  int currentState = 0;

  while (1) {
    if (!digital_get_dip(DD_DIP1)) {	//While dip switch 1 deactivated, nothing happens
      estado = STAND_BY;
      //continue;
    }
    tracef("Current state %i \n\r", estado);

    previousState = currentState;
    currentState = estado;

    switch (estado) {
      default:
      case STAND_BY:
        if (digital_get_dip(DD_DIP1)) {	//Once dip switch 1 activated, robot starts moving
          estado = FORWARD;
        }
        break;
      case OBSTACLE:
        if (!switch_left)                         {estado = BACKWARD_RIGHT;}	//If switch left detects (either with or without switch right)
        else if (switch_left && !switch_right)    {estado = BACKWARD_LEFT;}		//If ONLY switch right detects, turn to the left
        else if (distance_left)                   {estado = OBST_LEFT_DIST;}	//If distance left detects (either with or without distance right)
        else if (!distance_left && distance_right){estado = OBST_RIGHT_DIST;}	//If ONLY distance right detects, turn to the left
        else if (previousState == BACKWARD_RIGHT) {estado = OBST_LEFT_SWITCH;}
        else if (previousState == BACKWARD_LEFT)  {estado = OBST_RIGHT_SWITCH;}
        else {estado = FORWARD;}   //If no sensor is detecting, I move forward
        break;
      case OBST_RIGHT_SWITCH:
        vTaskDelay(_300MS);
        estado = OBSTACLE;    //Robot goes back until for some time, and then sensors are checked again
        break;
      case OBST_LEFT_SWITCH:
        vTaskDelay(_300MS);
        estado = OBSTACLE;    //Robot goes back until for some time, and then sensors are checked again
        break;
      case OBST_RIGHT_DIST:
        if (!switch_right || !switch_left || (!distance_left && !distance_right))  {
          estado = OBSTACLE;  //Robot turns left until no distance sensors (or any switch) detects
        }
        break;
      case OBST_LEFT_DIST:
        if (!switch_right || !switch_left || (!distance_left && !distance_right))  {
          estado = OBSTACLE;  //Robot turns right until no distance sensors (or any switch) detects
        }
        break;
      case FORWARD:
        if (!switch_right || !switch_left || distance_left || distance_right) {
          estado = OBSTACLE;    //If any switch or distance sensor detects, it seems there is an obstacle
        }
        else if (IR_detection==1) {estado = TURN_RIGHT;}  //If ONLY IR right detects, turn to the right
        else if (IR_detection==-1) {estado = TURN_LEFT;}  //If ONLY IR left detects, turn to the left
        break;
      case BACKWARD_RIGHT:
        vTaskDelay(_500MS);
        estado = OBSTACLE;    //Robot goes back until for some time, and then sensors are checked again
        break;
      case BACKWARD_LEFT:
        vTaskDelay(_500MS);
        estado = OBSTACLE;    //Robot goes back until for some time, and then sensors are checked again
        break;
      case TURN_RIGHT:
        if (!switch_right || !switch_left || distance_left || distance_right) {
          estado = OBSTACLE;    //If any switch or distance sensor detects, it seems there is an obstacle
        }
        else if (IR_detection == 0) {estado = FORWARD;}  //If range between IR sensors is not big enough, just go forward
        break;
      case TURN_LEFT:
        if (!switch_right || !switch_left || distance_left || distance_right) {
          estado = OBSTACLE;    //If any switch or distance sensor detects, it seems there is an obstacle
        }
        else if (IR_detection == 0) {estado = FORWARD;}  //If range between IR sensors is not big enough, just go forward
        break;
    }
  vTaskDelay(_10MS);     //Little delay to allow the other tasks to work
  }
}
static void outputs(void *pvParameters) {	//According to the current state (defined in the previous task),
  while (1) {								//the proper output (motor) combination is activated
    switch (estado) {
      default:
      case STAND_BY:
      case OBSTACLE:
        motorSTOP();
        break;
      case OBST_RIGHT_SWITCH:
      case OBST_RIGHT_DIST:
        motorAROUND_LEFT();
        break;
      case OBST_LEFT_SWITCH:
      case OBST_LEFT_DIST:
        motorAROUND_RIGHT();
        break;
      case FORWARD:
        motorFORWARD();
        break;
      case BACKWARD_RIGHT:
      case BACKWARD_LEFT:
        motorBACKWARD();
        break;
      case TURN_RIGHT:
        motorRIGHT();
        break;
      case TURN_LEFT:
        motorLEFT();
        break;
    }
    vTaskDelay(_10MS);   //Little delay to allow the other tasks to work
  }
}
void motorFORWARD() {			//Goes forward with increasingly speed
  turnRightSpeed = 0;
  turnLeftSpeed = 0;
  motor_set(DM_MOTOR0, 0);
  if (forSpeed < MAXspeed) {
    forSpeed = forSpeed + STEPspeed;
    motor_set(DM_MOTOR1, -forSpeed);
    motor_set(DM_MOTOR2, forSpeed);
    vTaskDelay(_20MS);
  }
}
void motorBACKWARD() {		//Goes backward with increasingly speed
  motor_set(DM_MOTOR0, 0);
  if (backSpeed < MAXspeed) {
    backSpeed = backSpeed + STEPspeed;
    motor_set(DM_MOTOR1, backSpeed);
    motor_set(DM_MOTOR2, -backSpeed);
    vTaskDelay(_20MS);
  }
}
void motorAROUND_RIGHT() {	//(All wheels) Turn right with increasingly speed
  if (aroundRightSpeed < MAXspeed) {
    aroundRightSpeed = aroundRightSpeed + STEPspeed;
    motor_set(DM_MOTOR0, aroundRightSpeed);
    motor_set(DM_MOTOR1, aroundRightSpeed);
    motor_set(DM_MOTOR2, aroundRightSpeed);
    vTaskDelay(_20MS);
  }
}
void motorAROUND_LEFT() {	//(All wheels) Turn left with increasingly speed
  if (aroundLeftSpeed < MAXspeed) {
    aroundLeftSpeed = aroundLeftSpeed + STEPspeed;
    motor_set(DM_MOTOR0, -aroundLeftSpeed);
    motor_set(DM_MOTOR1, -aroundLeftSpeed);
    motor_set(DM_MOTOR2, -aroundLeftSpeed);
    vTaskDelay(_20MS);
  }
}
void motorRIGHT() {			//(Back wheel) Turn right with increasingly speed
  if (turnRightSpeed < MAXspeed/2) {   //50%
    turnRightSpeed = turnRightSpeed + STEPspeed;
    motor_set(DM_MOTOR0, turnRightSpeed);
    vTaskDelay(_20MS);
  }
}
void motorLEFT() {			//(Back wheel) Turn left with increasingly speed
  if (turnLeftSpeed < MAXspeed/2) {   //50%
    turnLeftSpeed = turnLeftSpeed + STEPspeed;
    motor_set(DM_MOTOR0, -turnLeftSpeed);
    vTaskDelay(_20MS);
  }
}
void motorSTOP() {				//All wheels stop
  forSpeed = 0;
  backSpeed = 0;
  aroundRightSpeed = 0;
  aroundLeftSpeed = 0;
  turnRightSpeed = 0;
  turnLeftSpeed = 0;

  motor_set(DM_MOTOR0, 0);
  motor_set(DM_MOTOR1, 0);
  motor_set(DM_MOTOR2, 0);
  vTaskDelay(_20MS);
}
int main() {
  dorobo_init();    //Call dorobo_init() function to initialize HAL, Clocks, Timers etc.
  trace_init();     //Enable the tracing module.
  digital_init();   //Enable the digital I/O
  adc_init();       //Enable the analog I/O
  motor_init();     //Enable the motor
  ft_init();        //FFT initialization. Call this routine once before using the FFT module.

  digital_configure_pin(DD_PIN_PC8, DD_CFG_INPUT_PULLUP);   //PC8 configured as digital input (with pull_up resistor). Micro switch 1
  digital_configure_pin(DD_PIN_PD14, DD_CFG_INPUT_PULLUP);  //PD14 configured as digital input (with pull_up resistor). Micro switch 2
  digital_configure_pin(DD_PIN_PC13, DD_CFG_INPUT_NOPULL);  //PC13 configured as digital input (without pull_up/down resistor). IR remote sensor 1
  digital_configure_pin(DD_PIN_PA8, DD_CFG_INPUT_NOPULL);   //PA8 configured as digital input (without pull_up/down resistor). IR remote sensor 2

  //Task to execute / task name / MB to allocate in the RAM / variable to send to task / priority / task reference
  xTaskCreate(inputs, "INPUTS", 128, NULL, 1, NULL);          //Task to check the inputs
  xTaskCreate(state_machine, "STATES", 256, NULL, 1, NULL);   //Task to decide the DoRoBo possible states
  xTaskCreate(outputs, "OUTPUTS", 128, NULL, 1, NULL);        //Task to activate/deactivate the outputs (motor)

  vTaskStartScheduler();    //Start the real time FREERTOS scheduler.

  return 0;   // Will not get here unless there is insufficient RAM.
}
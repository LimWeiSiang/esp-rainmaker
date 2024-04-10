/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define DEFAULT_SWITCH_POWER        true
// #define DEFAULT_LIGHT_POWER         true
// #define DEFAULT_LIGHT_BRIGHTNESS    25
// #define DEFAULT_FAN_POWER           false
// #define DEFAULT_FAN_SPEED           3
// #define DEFAULT_TEMPERATURE         25.0
// #define REPORTING_PERIOD            60 /* Seconds */
#define REPORTING_PERIOD            5 /* Seconds */


//------------Rain Sensor---------------//
#define DEFAULT_RAIN_SENSOR false
//------------Rain Sensor---------------//


//------------Limit Switch---------------//
#define DEFAULT_LIMIT_SWITCH false
//------------Limit Switch---------------//

//------------Servo Switch---------------//
#define DEFAULT_SERVO_SWITCH false
//------------Servo Switch---------------//


extern esp_rmaker_device_t *switch_device;
// extern esp_rmaker_device_t *light__device;
// extern esp_rmaker_device_t *fan_device;
// extern esp_rmaker_device_t *temp_sensor_device;

//------------Rain Sensor---------------//
extern esp_rmaker_device_t *rain_sensor_device;
//------------Rain Sensor---------------//

//------------Limit Switch---------------//
extern esp_rmaker_device_t *limit_switch_device;
//------------Limit Switch---------------//

//------------Servo Switch---------------//
extern esp_rmaker_device_t *servo_switch_device;
//------------Servo Switch---------------//

// //-----------Servo Movement init-------------//
// void servo_init(void);
// //-----------Servo Movement init-------------//

void app_driver_init(void);
int app_driver_set_state(bool state);
bool app_driver_get_state(void);
// float app_get_current_temperature();

//------------Rain Sensor---------------//
bool app_get_current_rain_sensor();
//------------Rain Sensor---------------//

//------------Limit Switch---------------//
bool app_get_current_limit_switch();
//------------Limit Switch---------------//





//-----------Servo Switch Check Move Servo-------------//
static void servo_switch_event(void *arg);
void servo_check_move(bool servo_switch_state);

int app_driver_set_servo_switch_state(bool state);
//-----------Servo Switch Check Move Servo-------------//


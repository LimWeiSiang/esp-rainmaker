/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <sdkconfig.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

#include <iot_button.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>

#include <app_reset.h>
#include <ws2812_led.h>
#include "app_priv.h"

////------------Rain Sensor Library---------------////
#include "driver/gpio.h"
////------------Rain Sensor Library---------------////






/* This is the button that is used for toggling the power */
#define BUTTON_GPIO          CONFIG_EXAMPLE_BOARD_BUTTON_GPIO
#define BUTTON_ACTIVE_LEVEL  0
/* This is the GPIO on which the power will be set */
#define OUTPUT_GPIO    CONFIG_EXAMPLE_OUTPUT_GPIO

/* These values correspoind to H,S,V = 120,100,10 */
#define DEFAULT_RED     0
#define DEFAULT_GREEN   25
#define DEFAULT_BLUE    0

#define WIFI_RESET_BUTTON_TIMEOUT       3
#define FACTORY_RESET_BUTTON_TIMEOUT    10

////------------Rain Sensor PIN Declaration---------------////
#define RAIN_SENSOR_PIN     22
////------------Rain Sensor PIN Declaration---------------////



//-----------Limit Switch PIN Declaration--------------//
#define LIMIT_SWITCH_PIN     15
//-----------Limit Switch PIN Declaration--------------//




static bool g_power_state = DEFAULT_SWITCH_POWER;

//------------Rain Sensor Variables declaration---------------//
static bool rain_sensor_state = DEFAULT_RAIN_SENSOR;
static bool rain_notification_flag=false;
static int rain_notification_counter=0;
//------------Rain Sensor Variables declaration---------------//


//-----------Limit Switch Variables declaration--------------//
static bool limit_switch_state = DEFAULT_LIMIT_SWITCH;
//-----------Limit Switch Variables declaration--------------//




// static float g_temperature = DEFAULT_TEMPERATURE;
static TimerHandle_t sensor_timer;

// static void app_sensor_update(TimerHandle_t handle)
// {
//     static float delta = 0.5;
//     g_temperature += delta;
//     if (g_temperature > 99) {
//         delta = -0.5;
//     } else if (g_temperature < 1) {
//         delta = 0.5;
//     }
//     esp_rmaker_param_update_and_report(
//                 esp_rmaker_device_get_param_by_type(temp_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),
//                 esp_rmaker_float(g_temperature));
// }

// float app_get_current_temperature()
// {
//     return g_temperature;
// }

//-----------Limit Switch -------------//












static void app_indicator_set(bool state)
{
    if (state) {
        ws2812_led_set_rgb(DEFAULT_RED, DEFAULT_GREEN, DEFAULT_BLUE);
    } else {
        ws2812_led_clear();
    }
}

// static void set_limit_power_state(bool target)
// {
//     gpio_set_level(OUTPUT_GPIO, target);
//     app_indicator_set(target);
// }

// int IRAM_ATTR app_driver_set_limit_state(bool state)
// {
//     if(limit_switch_state != state) {
//         limit_switch_state = state;
//         set_limit_power_state(limit_switch_state);
//     }
//     return ESP_OK;
// }



static void limit_swith_press_event(void *arg)
{
    limit_switch_state=true;
    // if possible use a interupt for the motor here//


    esp_rmaker_param_update_and_report(
                esp_rmaker_device_get_param_by_type(limit_switch_device, ESP_RMAKER_PARAM_TEMPERATURE),
                esp_rmaker_float(limit_switch_state)); //esp_rmaker_param: New param value type not same as the existing one. if change to bool
}


static void limit_swith_release_event(void *arg)
{
    limit_switch_state=false;
    esp_rmaker_param_update_and_report(
                esp_rmaker_device_get_param_by_type(limit_switch_device, ESP_RMAKER_PARAM_TEMPERATURE),
                esp_rmaker_float(limit_switch_state)); //esp_rmaker_param: New param value type not same as the existing one. if change to bool
}



//-----------Limit Switch -------------//



//------------Rain Sensor Update---------------//

static void app_rain_sensor_update(TimerHandle_t handle)
{
    rain_sensor_state = (bool) gpio_get_level(RAIN_SENSOR_PIN);

    //----rain notification flag------//
    if(rain_sensor_state==true)
    {
        rain_notification_counter+=1;//increase count
        if(rain_notification_counter>=3 && rain_notification_flag==false)
        {
            esp_rmaker_raise_alert("🌧 Its Raininig!!! 🌧"); //----send notification-----//
            rain_notification_flag=true;

        }
    }
    else //if rain_sensor_state is false means no rain reset variables
    {
        rain_notification_flag=false;
        rain_notification_counter=0;

    }
    //----rain notification flag------//

    esp_rmaker_param_update_and_report(
                esp_rmaker_device_get_param_by_type(rain_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),
                esp_rmaker_float(rain_sensor_state)); //esp_rmaker_param: New param value type not same as the existing one. if change to bool
}
//------------Rain Sensor Update---------------//

//------------Rain Sensor Return State---------------//
bool app_get_current_rain_sensor()
{
    return rain_sensor_state;
}
//------------Rain Sensor Return State---------------//




//------------Rain Sensor Timer init---------------//

esp_err_t app_sensor_init(void)
{
    rain_sensor_state = DEFAULT_RAIN_SENSOR;
    sensor_timer = xTimerCreate("app_sensor_update_rs", (REPORTING_PERIOD * 1000) / portTICK_PERIOD_MS,
                            pdTRUE, NULL, app_rain_sensor_update);
    if (sensor_timer) {
        xTimerStart(sensor_timer, 0);
        return ESP_OK;
    }
    return ESP_FAIL;
}

//------------Rain Sensor Timer init---------------//















static void app_indicator_init(void)
{
    ws2812_led_init();
    app_indicator_set(g_power_state);
}

static void push_btn_cb(void *arg)
{
    bool new_state = !g_power_state;
    app_driver_set_state(new_state);
    esp_rmaker_param_update_and_report(
                esp_rmaker_device_get_param_by_type(switch_device, ESP_RMAKER_PARAM_POWER),
                esp_rmaker_bool(new_state));
}

static void set_power_state(bool target)
{
    gpio_set_level(OUTPUT_GPIO, target);
    app_indicator_set(target);
}

void app_driver_init()
{
    
    button_handle_t btn_handle = iot_button_create(BUTTON_GPIO, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        /* Register a callback for a button tap (short press) event */
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_TAP, push_btn_cb, NULL);
        /* Register Wi-Fi reset and factory reset functionality on same button */
        app_reset_button_register(btn_handle, WIFI_RESET_BUTTON_TIMEOUT, FACTORY_RESET_BUTTON_TIMEOUT);
    }

    /* Configure power */
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 1,
    };
    io_conf.pin_bit_mask = ((uint64_t)1 << OUTPUT_GPIO);
    /* Configure the GPIO */
    gpio_config(&io_conf);
    app_indicator_init();
    app_sensor_init();



    // // ------------Rain Sensor---------------//
    // button_handle_t rain_sensor_handle = iot_button_create(RAIN_SENSOR_PIN, RAIN_SENSOR_ACTIVE_LEVEL);
    // if (rain_sensor_handle) {
    //     /* Register a callback for a button tap (short press) event */
    //     iot_button_set_evt_cb(rain_sensor_handle, BUTTON_CB_TAP, rain_sensor_event, "TAP");
    // }
    // // ------------Rain Sensor---------------//



    //-----------Limit Switch --------------//

    button_handle_t limit_switch_handle = iot_button_create(LIMIT_SWITCH_PIN, BUTTON_ACTIVE_LEVEL);
    //-----Two seperate events one for when limit switch is depressed another for release-----//
    iot_button_set_evt_cb(limit_switch_handle, BUTTON_CB_PUSH, limit_swith_press_event, NULL);
    iot_button_set_evt_cb(limit_switch_handle, BUTTON_CB_RELEASE, limit_swith_release_event, NULL);

    


    //-----------Limit Switch --------------//

}

int IRAM_ATTR app_driver_set_state(bool state)
{
    if(g_power_state != state) {
        g_power_state = state;
        set_power_state(g_power_state);
    }
    return ESP_OK;
}

bool app_driver_get_state(void)
{
    return g_power_state;
}




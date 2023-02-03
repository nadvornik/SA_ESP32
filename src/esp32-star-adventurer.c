/* This sketch is a extension/expansion/rework of the 'official' ESP32 Camera example
 *  sketch from Expressif:
 *  https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera/CameraWebServer
 *  and
 *  https://github.com/easytarget/esp32-cam-webserver
 *
 * note: Make sure that you have either selected ESP32 AI Thinker,
 *       or another board which has PSRAM enabled to use high resolution camera modes
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_timer.h"

#include "motor.h"

const gpio_num_t MOTOR1_GPIO = (gpio_num_t)12;
const gpio_num_t MOTOR2_GPIO = (gpio_num_t)13;
const gpio_num_t ENC1_GPIO = (gpio_num_t)14;
const gpio_num_t ENC2_GPIO = (gpio_num_t)15;

TaskHandle_t task_to_notify = NULL;



const int32_t PWM_PERIOD = 50000;
const int32_t PWM_CB_AT = 48000;
const int32_t PWM_RESOLUTION_HZ = 1000000;


dc_motor_context_t dc_motor_context = {
    .target_speed = 5,
    .Kp = 0.0030,
    .Ki = 0.0003,
    .Kd = 0.0010,
};

static int32_t pid(dc_motor_context_t *dc_motor_context, double error)
{
    dc_motor_context->pid_output += error * dc_motor_context->Ki + 
                  (error - dc_motor_context->prev_error) * dc_motor_context->Kp +
                  (error - 2 * dc_motor_context->prev_error + dc_motor_context->prev_error2) * dc_motor_context->Kd;
                  
    dc_motor_context->prev_error2 = dc_motor_context->prev_error;
    dc_motor_context->prev_error = error;


    if (dc_motor_context->pid_output < 0) dc_motor_context->pid_output = 0;
    if (dc_motor_context->pid_output > 1) dc_motor_context->pid_output = 1;
    return dc_motor_context->pid_output * PWM_PERIOD;
}

static bool pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    dc_motor_context_t *dc_motor_context = (dc_motor_context_t *)user_ctx;
    dc_motor_context->accumu_count += edata->watch_point_value;
    return false;
}
 

static bool pwm_callback(mcpwm_cmpr_handle_t comp2, const mcpwm_compare_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;
    dc_motor_context_t *dc_motor_context = (dc_motor_context_t *)user_ctx;
    
    int pulse_count_hw;
    ESP_ERROR_CHECK(pcnt_unit_get_count(dc_motor_context->pcnt_unit, &pulse_count_hw));
    int32_t pulse_count_new = dc_motor_context->accumu_count + pulse_count_hw;
    int32_t pulse_new = pulse_count_new - dc_motor_context->pulse_count;
    if (dc_motor_context->direction) pulse_new = -pulse_new;
    dc_motor_context->pulse_count = pulse_count_new;

    dc_motor_context->dif = dc_motor_context->target_speed - pulse_new;
    dc_motor_context->idif += dc_motor_context->dif;

    dc_motor_context->comp_value = pid(dc_motor_context, dc_motor_context->idif);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(dc_motor_context->comparator, dc_motor_context->comp_value));

    xTaskNotifyFromISR(task_to_notify, 0, eSetValueWithOverwrite, &high_task_wakeup);

    return high_task_wakeup;
}


void dc_motor_init(dc_motor_context_t *dc_motor_context)
{
    pcnt_unit_config_t unit_config = {
        .high_limit = 30000,
        .low_limit = -30000,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &dc_motor_context->pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 2000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(dc_motor_context->pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENC1_GPIO,
        .level_gpio_num = ENC2_GPIO,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(dc_motor_context->pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENC2_GPIO,
        .level_gpio_num = ENC1_GPIO,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(dc_motor_context->pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(dc_motor_context->pcnt_unit, 30000));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(dc_motor_context->pcnt_unit, -30000));
    pcnt_event_callbacks_t pcnt_cbs = {
        .on_reach = pcnt_on_reach, // accumulate the overflow in the callback
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(dc_motor_context->pcnt_unit, &pcnt_cbs, dc_motor_context));


    ESP_ERROR_CHECK(pcnt_unit_enable(dc_motor_context->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(dc_motor_context->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(dc_motor_context->pcnt_unit));



    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_RESOLUTION_HZ,
        .period_ticks = PWM_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &dc_motor_context->timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, dc_motor_context->timer));

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &dc_motor_context->comparator));

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = MOTOR1_GPIO,
        .flags.invert_pwm = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &dc_motor_context->generator1));


    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(dc_motor_context->generator1,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW),
                    MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(dc_motor_context->generator1,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, dc_motor_context->comparator, MCPWM_GEN_ACTION_LOW),
                    MCPWM_GEN_COMPARE_EVENT_ACTION_END()));


    generator_config.gen_gpio_num = MOTOR2_GPIO;

    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &dc_motor_context->generator2));
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(dc_motor_context->generator2,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW),
                    MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(dc_motor_context->generator2,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, dc_motor_context->comparator, MCPWM_GEN_ACTION_LOW),
                    MCPWM_GEN_COMPARE_EVENT_ACTION_END()));


    mcpwm_cmpr_handle_t comparator2 = NULL;
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator2));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, PWM_CB_AT));
    
    mcpwm_comparator_event_callbacks_t comparator_callbacks = {
        .on_reach = pwm_callback,
    };
    ESP_ERROR_CHECK(mcpwm_comparator_register_event_callbacks(comparator2, &comparator_callbacks, dc_motor_context));

/*
    esp_rom_gpio_pad_select_gpio(MOTOR2_GPIO);
    gpio_set_direction(MOTOR2_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR2_GPIO, 1);
*/

    ESP_ERROR_CHECK(mcpwm_timer_enable(dc_motor_context->timer));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(dc_motor_context->comparator, 0));

}

void dc_motor_start(dc_motor_context_t *dc_motor_context)
{
    dc_motor_context->idif = 0;

    if (dc_motor_context->direction) {
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(dc_motor_context->generator1,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW),
                    MCPWM_GEN_TIMER_EVENT_ACTION_END()));
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(dc_motor_context->generator2,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
                    MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    }
    else {
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(dc_motor_context->generator1,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
                    MCPWM_GEN_TIMER_EVENT_ACTION_END()));
        ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(dc_motor_context->generator2,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW),
                    MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    }

    ESP_ERROR_CHECK(mcpwm_timer_start_stop(dc_motor_context->timer, MCPWM_TIMER_START_NO_STOP));
    dc_motor_context->running = true;
}

void dc_motor_stop(dc_motor_context_t *dc_motor_context)
{
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(dc_motor_context->timer, MCPWM_TIMER_STOP_FULL));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(dc_motor_context->generator1,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW),
                    MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(dc_motor_context->generator2,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW),
                    MCPWM_GEN_TIMER_EVENT_ACTION_END()));

    dc_motor_context->running = false;
}

void dc_motor_set_direction(dc_motor_context_t *dc_motor_context, bool direction)
{
    dc_motor_context->direction = direction;
}

void dc_motor_set_stop_at_target(dc_motor_context_t *dc_motor_context, bool stop_at_target)
{
    dc_motor_context->stop_at_target = stop_at_target;
}

void dc_motor_set_init(dc_motor_context_t *dc_motor_context, bool init)
{
    dc_motor_context->init = init;
}

void dc_motor_set_position(dc_motor_context_t *dc_motor_context, int32_t position)
{
    ESP_ERROR_CHECK(pcnt_unit_clear_count(dc_motor_context->pcnt_unit));
    dc_motor_context->accumu_count = position;
    dc_motor_context->pulse_count = position;
    dc_motor_context->idif = 0;
}

void dc_motor_set_speed(dc_motor_context_t *dc_motor_context, double speed)
{
    dc_motor_context->target_speed = speed;
}

void dc_motor_set_target(dc_motor_context_t *dc_motor_context, int32_t target)
{
    dc_motor_context->target = target;
}

bool dc_motor_get_direction(dc_motor_context_t *dc_motor_context)
{
    return dc_motor_context->direction;
}

bool dc_motor_get_running(dc_motor_context_t *dc_motor_context)
{
    return dc_motor_context->running;
}

bool dc_motor_get_stop_at_target(dc_motor_context_t *dc_motor_context)
{
    return dc_motor_context->stop_at_target;
}

bool dc_motor_get_init(dc_motor_context_t *dc_motor_context)
{
    return dc_motor_context->init;
}

int32_t dc_motor_get_position(dc_motor_context_t *dc_motor_context)
{
    return dc_motor_context->pulse_count;
}

int32_t dc_motor_get_target(dc_motor_context_t *dc_motor_context)
{
    return dc_motor_context->target;
}

double dc_motor_get_speed(dc_motor_context_t *dc_motor_context)
{
    return dc_motor_context->target_speed;
}



void setup()
{

    task_to_notify = xTaskGetCurrentTaskHandle();

    dc_motor_init(&dc_motor_context);
    
//    dc_motor_context.direction = true;
//    dc_motor_start(&dc_motor_context);
//    esp_rom_gpio_pad_select_gpio(ENC1_GPIO);
//    gpio_set_direction(ENC1_GPIO, GPIO_MODE_INPUT);


}


int handle_command(char *cmd, char *resp);

char cmd[256];
int  cmdpos = 0;
char resp[30];

void loop()
{

/*
    uint32_t source;
    if (xTaskNotifyWait(0x00, ULONG_MAX, &source, pdMS_TO_TICKS(1000)) == pdTRUE) {
       printf("%8ld %8ld   %1.4f %1.4f  %1.4f %1.4f %1.4f\n", dc_motor_context.pulse_count, dc_motor_context.idif, 
               dc_motor_context.pid_output, dc_motor_context.prev_error, dc_motor_context.Kp, dc_motor_context.Ki, dc_motor_context.Kd);
    }
    
    int ch = fgetc(stdin);
    if (ch == 'q') dc_motor_context.Kp += 0.0001;
    if (ch == 'a') dc_motor_context.Kp -= 0.0001;
    if (ch == 'w') dc_motor_context.Ki += 0.0001;
    if (ch == 's') dc_motor_context.Ki -= 0.0001;
    if (ch == 'e') dc_motor_context.Kd += 0.0001;
    if (ch == 'd') dc_motor_context.Kd -= 0.0001;
  */  
//    if (pdTICKS_TO_MS(xTaskGetTickCount()) % 60000 < 30000) target = 3000000; else target = 6000000;


    int ch = fgetc(stdin);
    if (ch > 0) {
        //printf("%d %d\n", ch, cmdpos);
        if (ch == ':') {
            cmd[0] = ch;
            cmdpos = 1;
        }
        else if ((cmdpos > 0) && (cmdpos < 255)) {
            cmd[cmdpos++] = ch;
            if ((ch == 0x0d) || (ch == 0x0a)) {
                //printf("%d\n", cmd[3]);
                handle_command(cmd, resp);
                printf("%s", resp);
                cmdpos = 0;
            }
        }
    }
}

TaskHandle_t loopTaskHandle = NULL;

void loopTask(void *pvParameters)
{
    setup();
    loop();
    
//    esp_ota_mark_app_valid_cancel_rollback();
    
    while(1) {
        loop();
    }
}

//extern "C" 
void app_main()
{
    xTaskCreatePinnedToCore(loopTask, "loopTask", 8192, NULL, 0, &loopTaskHandle, 1);
}

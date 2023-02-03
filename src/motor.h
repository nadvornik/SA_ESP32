#include "driver/pulse_cnt.h"

#include "driver/mcpwm_prelude.h"
//#include "soc/mcpwm_periph.h"

typedef struct {
    mcpwm_timer_handle_t timer;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator1;
    mcpwm_gen_handle_t generator2;
    pcnt_unit_handle_t pcnt_unit;

    double target_speed;
    int32_t comp_value;
    
    double pid_output;
    double prev_error;
    double prev_error2;

    double Kp;
    double Ki;
    double Kd;

    double dif;
    double idif;

    int32_t pulse_count;
    int32_t accumu_count;
    
    int32_t target;
    
    bool direction;
    bool running;
    bool stop_at_target;
    bool init;

} dc_motor_context_t;

#define DC_MOTOR_BASE_SPEED 5
#define DC_WORM_PERIOD (300 * 200)

void dc_motor_start(dc_motor_context_t *dc_motor_context);
void dc_motor_stop(dc_motor_context_t *dc_motor_context);

void dc_motor_set_direction(dc_motor_context_t *dc_motor_context, bool direction);
void dc_motor_set_stop_at_target(dc_motor_context_t *dc_motor_context, bool stop);
void dc_motor_set_init(dc_motor_context_t *dc_motor_context, bool init);
void dc_motor_set_position(dc_motor_context_t *dc_motor_context, int32_t position);
void dc_motor_set_target(dc_motor_context_t *dc_motor_context, int32_t target);
void dc_motor_set_speed(dc_motor_context_t *dc_motor_context, double speed);

bool dc_motor_get_direction(dc_motor_context_t *dc_motor_context);
bool dc_motor_get_running(dc_motor_context_t *dc_motor_context);
bool dc_motor_get_stop_at_target(dc_motor_context_t *dc_motor_context);
bool dc_motor_get_init(dc_motor_context_t *dc_motor_context);
int32_t dc_motor_get_position(dc_motor_context_t *dc_motor_context);
int32_t dc_motor_get_target(dc_motor_context_t *dc_motor_context);
double dc_motor_get_speed(dc_motor_context_t *dc_motor_context);

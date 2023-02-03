#include <stdio.h>
#include "motor.h"

#define CMD_LEN_ERROR 1
#define CMD_INVALID_CHAR 3
#define CMD_UNKNOWN 0

#define STEPS_MUL 4
#define STEPS_OFF 0x800000

extern dc_motor_context_t dc_motor_context;

uint32_t hex(char c)
{
    if ((c >= '0') && (c <= '9')) return c - '0';
    if ((c >= 'A') && (c <= 'F')) return c - 'A' + 10;
    return 0;
}

int parse6(char *cmd, uint32_t *res)
{
    if ((cmd[9] != 0x0d) && (cmd[9] != 0x0a)) return 0;
    *res = hex(cmd[7]);
    *res <<= 4;
    *res |= hex(cmd[8]);
    *res <<= 4;
    *res |= hex(cmd[5]);
    *res <<= 4;
    *res |= hex(cmd[6]);
    *res <<= 4;
    *res |= hex(cmd[3]);
    *res <<= 4;
    *res |= hex(cmd[4]);
    return 1;  
}

int parse2(char *cmd, uint32_t *res)
{
    if ((cmd[5] != 0x0d) && (cmd[5] != 0x0a)) return 0;
    *res = hex(cmd[3]);
    *res <<= 4;
    *res |= hex(cmd[4]);
    return 1;  
}

char hexd[] = "0123456789ABCDEF";

int resp6(char *resp, uint32_t v)
{
    resp[0] = '=';
    resp[1] = hexd[(v & 0xF0) >> 4];
    resp[2] = hexd[(v & 0x0F)];
    resp[3] = hexd[(v & 0xF000) >> 12];
    resp[4] = hexd[(v & 0x0F00) >> 8];
    resp[5] = hexd[(v & 0xF00000) >> 20];
    resp[6] = hexd[(v & 0x0F0000) >> 16];
    resp[7] = 0x0d;
    return 1;
}

int resp3(char *resp, uint32_t v)
{
    resp[0] = '=';
    resp[1] = hexd[(v & 0xF0) >> 4];
    resp[2] = hexd[(v & 0x0F)];
    resp[3] = hexd[(v & 0x0F00) >> 8];
    resp[4] = 0x0d;
    return 1;
}


int resp2(char *resp, uint32_t v)
{
    resp[0] = '=';
    resp[1] = hexd[(v & 0xF0) >> 4];
    resp[2] = hexd[(v & 0x0F)];
    resp[3] = 0x0d;
    return 1;
}

int sw_ok(char *resp)
{
    resp[0] = '=';
    resp[1] = 0x0d;
    return 0;
}


int sw_error(char *resp, int code)
{
    resp[0] = '!';
    resp[1] = hexd[code];
    resp[2] = 0x0d;
    return 0;
}


static int set_position(char *cmd, char *resp)
{
    uint32_t pos;
    if (!parse6(cmd, &pos)) return sw_error(resp, CMD_LEN_ERROR);

    switch (cmd[2]) {
        case '1':
            dc_motor_set_position(&dc_motor_context, ((int32_t)pos - STEPS_OFF)*STEPS_MUL);
            break;
        case '2':
//            stepper_set_position(pos);
            break;
        case '3':
            dc_motor_set_position(&dc_motor_context, ((int32_t)pos - STEPS_OFF)*STEPS_MUL);
//            stepper_set_position(pos);
            break;
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
    return sw_ok(resp);
}

static int init_done(char *cmd, char *resp)
{
    dc_motor_set_init(&dc_motor_context, true);
    return sw_ok(resp);
}

static int set_mode(char *cmd, char *resp)
{
    uint32_t mode;
    if (!parse2(cmd, &mode)) return sw_error(resp, CMD_LEN_ERROR);

    switch (cmd[2]) {
        case '1':
            dc_motor_set_direction(&dc_motor_context, mode & 0x01);
            break;
        case '2':
//            stepper_set_position(pos);
            break;
        case '3':
            dc_motor_set_direction(&dc_motor_context, mode & 0x01);
//            stepper_set_position(pos);
            break;
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
    return sw_ok(resp);
}

static int set_target(char *cmd, char *resp)
{
    uint32_t pos;
    if (!parse6(cmd, &pos)) return sw_error(resp, CMD_LEN_ERROR);

    switch (cmd[2]) {
        case '1':
            dc_motor_set_target(&dc_motor_context, ((int32_t)pos - STEPS_OFF)*STEPS_MUL);
            break;
        case '2':
//            stepper_set_position(pos);
            break;
        case '3':
            dc_motor_set_target(&dc_motor_context, ((int32_t)pos - STEPS_OFF)*STEPS_MUL);
//            stepper_set_position(pos);
            break;
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
    return sw_ok(resp);
}

static int set_period(char *cmd, char *resp)
{
    uint32_t period;
    if (!parse6(cmd, &period)) return sw_error(resp, CMD_LEN_ERROR);

    switch (cmd[2]) {
        case '1':
            dc_motor_set_speed(&dc_motor_context, 50000.0 / period * STEPS_MUL);
            break;
        case '2':
//            stepper_set_position(pos);
            break;
        case '3':
            dc_motor_set_speed(&dc_motor_context, 50000.0 / period * STEPS_MUL);
//            stepper_set_position(pos);
            break;
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
    return sw_ok(resp);
}

static int start(char *cmd, char *resp)
{
    if ((cmd[3] != 0x0d) && (cmd[3] != 0x0a)) return sw_error(resp, CMD_LEN_ERROR);

    switch (cmd[2]) {
        case '1':
            dc_motor_start(&dc_motor_context);
            break;
        case '2':
//            stepper_set_position(pos);
            break;
        case '3':
            dc_motor_start(&dc_motor_context);
//            stepper_set_position(pos);
            break;
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
    return sw_ok(resp);
}

static int stop(char *cmd, char *resp)
{
    if ((cmd[3] != 0x0d) && (cmd[3] != 0x0a)) return sw_error(resp, CMD_LEN_ERROR);

    switch (cmd[2]) {
        case '1':
            dc_motor_stop(&dc_motor_context);
            break;
        case '2':
//            stepper_set_position(pos);
            break;
        case '3':
            dc_motor_stop(&dc_motor_context);
//            stepper_set_position(pos);
            break;
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
    return sw_ok(resp);
}

static int instant_stop(char *cmd, char *resp)
{
    if ((cmd[3] != 0x0d) && (cmd[3] != 0x0a)) return sw_error(resp, CMD_LEN_ERROR);

    switch (cmd[2]) {
        case '1':
            dc_motor_stop(&dc_motor_context);
            break;
        case '2':
//            stepper_set_position(pos);
            break;
        case '3':
            dc_motor_stop(&dc_motor_context);
//            stepper_set_position(pos);
            break;
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
    return sw_ok(resp);
}

static int set_aux(char *cmd, char *resp)
{
    return sw_ok(resp);
}

static int set_guiding(char *cmd, char *resp)
{
    return sw_ok(resp);
}

static int set_led(char *cmd, char *resp)
{
    return sw_ok(resp);
}

static int get_cpr(char *cmd, char *resp)
{
    if ((cmd[3] != 0x0d) && (cmd[3] != 0x0a)) return sw_error(resp, CMD_LEN_ERROR);
    switch (cmd[2]) {
        case '1':
            return resp6(resp, DC_WORM_PERIOD * 144 / STEPS_MUL);
        case '2':
            return resp6(resp, DC_WORM_PERIOD * 144 / STEPS_MUL);
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
}

static int get_freq(char *cmd, char *resp)
{
    if ((cmd[3] != 0x0d) && (cmd[3] != 0x0a)) return sw_error(resp, CMD_LEN_ERROR);
    switch (cmd[2]) {
        case '1':
            return resp6(resp, 1000000);
        case '2':
            return resp6(resp, 1000000);
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
}

static int get_target(char *cmd, char *resp)
{
    if ((cmd[3] != 0x0d) && (cmd[3] != 0x0a)) return sw_error(resp, CMD_LEN_ERROR);
    switch (cmd[2]) {
        case '1':
            return resp6(resp, dc_motor_get_target(&dc_motor_context) / STEPS_MUL + STEPS_OFF);
        case '2':
            return resp6(resp, dc_motor_get_target(&dc_motor_context) / STEPS_MUL + STEPS_OFF);
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
}

static int get_period(char *cmd, char *resp)
{
    if ((cmd[3] != 0x0d) && (cmd[3] != 0x0a)) return sw_error(resp, CMD_LEN_ERROR);
    switch (cmd[2]) {
        case '1':
            return resp6(resp, 300 / STEPS_MUL);
        case '2':
            return resp6(resp, 300 / STEPS_MUL);
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
}


static int get_pos(char *cmd, char *resp)
{
    if ((cmd[3] != 0x0d) && (cmd[3] != 0x0a)) return sw_error(resp, CMD_LEN_ERROR);
    switch (cmd[2]) {
        case '1':
            return resp6(resp, dc_motor_get_position(&dc_motor_context) / STEPS_MUL + STEPS_OFF);
        case '2':
            return resp6(resp, dc_motor_get_position(&dc_motor_context) / STEPS_MUL + STEPS_OFF);
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
}

#define STATUS_RUNNING  0x001
#define STATUS_TRACKING 0x010
#define STATUS_CCW      0x020
#define STATUS_FAST     0x020
#define STATUS_INIT     0x100


static int get_status(char *cmd, char *resp)
{
    if ((cmd[3] != 0x0d) && (cmd[3] != 0x0a)) return sw_error(resp, CMD_LEN_ERROR);
    int status = 0;
    switch (cmd[2]) {
        case '1':
            if (dc_motor_get_running(&dc_motor_context)) status |= STATUS_RUNNING;
            if (!dc_motor_get_stop_at_target(&dc_motor_context)) status |= STATUS_TRACKING;
            if (dc_motor_get_direction(&dc_motor_context)) status |= STATUS_CCW;
            if (dc_motor_get_init(&dc_motor_context)) status |= STATUS_INIT;
            
            return resp3(resp, status);
        case '2':
            return resp3(resp, 0);
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
}

static int get_high_speed(char *cmd, char *resp)
{
    if ((cmd[3] != 0x0d) && (cmd[3] != 0x0a)) return sw_error(resp, CMD_LEN_ERROR);
    int status = 0;
    switch (cmd[2]) {
        case '1':
            return resp2(resp, 1);
        case '2':
            return resp2(resp, 1);
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
}

static int get_1x(char *cmd, char *resp)
{
    if ((cmd[3] != 0x0d) && (cmd[3] != 0x0a)) return sw_error(resp, CMD_LEN_ERROR);
    switch (cmd[2]) {
        case '1':
            return resp6(resp, DC_WORM_PERIOD);
        case '2':
            return resp6(resp, DC_WORM_PERIOD);
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
}

static int get_version(char *cmd, char *resp)
{
    if ((cmd[3] != 0x0d) && (cmd[3] != 0x0a)) return sw_error(resp, CMD_LEN_ERROR);
    switch (cmd[2]) {
        case '1':
            return resp6(resp, 0x123456);
        case '2':
            return resp6(resp, 0x123456);
        default:
            return sw_error(resp, CMD_INVALID_CHAR);
    }
}



int handle_command(char *cmd, char *resp)
{
    if (cmd[0] != ':') return sw_error(resp, CMD_INVALID_CHAR);
    switch (cmd[1]) {
        case 'E':
            return set_position(cmd, resp);
        case 'F':
            return init_done(cmd, resp);
        case 'G':
            return set_mode(cmd, resp);
        case 'S':
            return set_target(cmd, resp);
        case 'I':
            return set_period(cmd, resp);
        case 'J':
            return start(cmd, resp);
        case 'K':
            return stop(cmd, resp);
        case 'L':
            return instant_stop(cmd, resp);
        case 'O':
            return set_aux(cmd, resp);
        case 'P':
            return set_guiding(cmd, resp);
        case 'V':
            return set_led(cmd, resp);
        case 'a':
            return get_cpr(cmd, resp);
        case 'b':
            return get_freq(cmd, resp);
        case 'h':
            return get_target(cmd, resp);
        case 'i':
            return get_period(cmd, resp);
        case 'j':
            return get_pos(cmd, resp);
        case 'f':
            return get_status(cmd, resp);
        case 'g':
            return get_high_speed(cmd, resp);
        case 'D':
            return get_1x(cmd, resp);
        case 'e':
            return get_version(cmd, resp);
        default:
            return sw_error(resp, CMD_UNKNOWN);
    }
}
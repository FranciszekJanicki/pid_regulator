#ifndef PID_REGULATOR_PID_REGULATOR_CONFIG_H
#define PID_REGULATOR_PID_REGULATOR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

typedef float float32_t;

typedef enum {
    PID_REGULATOR_ERR_OK = 0,
    PID_REGULATOR_ERR_FAIL,
    PID_REGULATOR_ERR_NULL,
    PID_REGULATOR_ERR_ZERO_DIVISION,
} pid_regulator_err_t;

typedef struct {
    float32_t prev_error;
    float32_t int_error;
    float32_t dot_error;
    float32_t sat_error;
    float32_t prev_sat_error;
    float32_t int_sat_error;
} pid_regulator_state_t;

typedef struct {
    float32_t prop_gain;
    float32_t int_gain;
    float32_t dot_gain;
    float32_t dot_time;
    float32_t sat_gain;
    float32_t min_control;
    float32_t max_control;
    float32_t dead_error;
} pid_regulator_config_t;

#ifdef __cplusplus
}
#endif

#endif // PID_REGULATOR_PID_REGULATOR_CONFIG_H
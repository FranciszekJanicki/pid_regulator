#include "pid_regulator.h"
#include "pid_regulator_config.h"
#include <assert.h>
#include <math.h>
#include <string.h>

static inline float32_t pid_regulator_get_prop_term(pid_regulator_t* regulator,
                                                    float32_t error)
{
    return regulator->config.prop_gain * error;
}

static inline float32_t pid_regulator_get_dot_term(pid_regulator_t* regulator,
                                                   float32_t error,
                                                   float32_t delta_time)
{
    float32_t dot_error = (error - regulator->state.prev_error) / delta_time;

    if (regulator->config.dot_time > 0.0F) {
        float32_t alpha =
            delta_time / (regulator->config.dot_time + delta_time);
        regulator->state.dot_error =
            alpha * dot_error + (1.0F - alpha) * regulator->state.dot_error;
    } else {
        regulator->state.dot_error = dot_error;
    }

    regulator->state.prev_error = error;

    return regulator->config.dot_gain * regulator->state.dot_error;
}

static inline float32_t pid_regulator_get_int_term(pid_regulator_t* regulator,
                                                   float32_t error,
                                                   float32_t delta_time)
{
    regulator->state.int_error +=
        ((error + regulator->state.prev_error) * delta_time / 2.0F);
    regulator->state.int_sat_error +=
        ((regulator->state.sat_error + regulator->state.prev_sat_error) *
         delta_time / 2.0F);

    return (regulator->config.int_gain * regulator->state.int_error) -
           (regulator->config.sat_gain * regulator->state.int_sat_error);
}

static inline float32_t pid_regulator_clamp_control(pid_regulator_t* regulator,
                                                    float32_t control)
{
    if (control != 0.0F) {
        if (fabsf(control) > regulator->config.max_control) {
            control = copysignf(regulator->config.max_control, control);
        } else if (fabsf(control) < regulator->config.min_control) {
            control = copysignf(regulator->config.min_control, control);
        }
    }

    return control;
}

pid_regulator_err_t pid_regulator_initialize(
    pid_regulator_t* regulator,
    pid_regulator_config_t const* config)
{
    if (regulator == NULL || config == NULL) {
        return PID_REGULATOR_ERR_NULL;
    }

    memset(regulator, 0, sizeof(*regulator));
    memcpy(&regulator->config, config, sizeof(*config));

    return PID_REGULATOR_ERR_OK;
}

pid_regulator_err_t pid_regulator_deinitialize(pid_regulator_t* regulator)
{
    if (regulator == NULL) {
        return PID_REGULATOR_ERR_NULL;
    }

    memset(regulator, 0, sizeof(*regulator));

    return PID_REGULATOR_ERR_OK;
}

pid_regulator_err_t pid_regulator_reset(pid_regulator_t* regulator)
{
    if (regulator == NULL) {
        return PID_REGULATOR_ERR_NULL;
    }

    memset(&regulator->state, 0, sizeof(regulator->state));

    return PID_REGULATOR_ERR_OK;
}

pid_regulator_err_t pid_regulator_get_control(pid_regulator_t* regulator,
                                              float32_t error,
                                              float32_t delta_time,
                                              float32_t* control)
{
    if (regulator == NULL || control == NULL) {
        return PID_REGULATOR_ERR_NULL;
    }

    if (delta_time == 0.0F) {
        return PID_REGULATOR_ERR_ZERO_DIVISION;
    }

    if (fabsf(error) < regulator->config.dead_error) {
        *control = 0.0F;

        return PID_REGULATOR_ERR_OK;
    }

    float32_t prop_term = pid_regulator_get_prop_term(regulator, error);
    float32_t int_term =
        pid_regulator_get_int_term(regulator, error, delta_time);
    float32_t dot_term =
        pid_regulator_get_dot_term(regulator, error, delta_time);

    *control = prop_term + int_term + dot_term;

    return PID_REGULATOR_ERR_OK;
}

pid_regulator_err_t pid_regulator_get_sat_control(pid_regulator_t* regulator,
                                                  float32_t error,
                                                  float32_t delta_time,
                                                  float32_t* sat_control)
{
    if (regulator == NULL || sat_control == NULL) {
        return PID_REGULATOR_ERR_NULL;
    }

    float32_t control;
    pid_regulator_err_t err =
        pid_regulator_get_control(regulator, error, delta_time, &control);
    if (err != PID_REGULATOR_ERR_OK) {
        return err;
    }

    *sat_control = pid_regulator_clamp_control(regulator, control);

    regulator->state.prev_sat_error = regulator->state.sat_error;
    regulator->state.sat_error = control - *sat_control;

    return PID_REGULATOR_ERR_OK;
}

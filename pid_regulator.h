#ifndef PID_REGULATOR_PID_REGULATOR_H
#define PID_REGULATOR_PID_REGULATOR_H

#include "pid_regulator_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pid_regulator_state_t state;
    pid_regulator_config_t config;
} pid_regulator_t;

pid_regulator_err_t pid_regulator_initialize(
    pid_regulator_t* regulator,
    pid_regulator_config_t const* config);
pid_regulator_err_t pid_regulator_deinitialize(pid_regulator_t* regulator);

pid_regulator_err_t pid_regulator_reset(pid_regulator_t* regulator);

pid_regulator_err_t pid_regulator_get_control(pid_regulator_t* regulator,
                                              float32_t error,
                                              float32_t delta_time,
                                              float32_t* control);
pid_regulator_err_t pid_regulator_get_sat_control(pid_regulator_t* regulator,
                                                  float32_t error,
                                                  float32_t delta_time,
                                                  float32_t* sat_control);

#ifdef __cplusplus
}
#endif

#endif // PID_REGULATOR_PID_REGULATOR_H

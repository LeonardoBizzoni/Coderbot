#include <stdio.h>

#include <base/base_inc.h>
#include <OS/os_inc.h>
#include <base/base_inc.c>
#include <OS/os_inc.c>

#include "coderbot.h"
#include "odometry.h"
#include "encoder.h"

fn void deadline_handler(i32 sig) {
  Err("A task missed its deadline!");
  (void)signal(SIGXCPU, SIG_DFL);
}

global u64 measured_ticks_left = 0;
global u64 measured_ticks_right = 0;

global OS_Handle tick_mutex;

#include "coderbot.c"
#include "odometry.c"
#include "encoder.c"

fn void start(CmdLine *cli) {
  os_atexit(cb_stop);

  tick_mutex = os_mutex_alloc();

  gpioInitialise();
  cbMotorGPIOinit(&cb_motor_left);
  cbMotorGPIOinit(&cb_motor_right);
  cbEncoderGPIOinit(&cb_encoder_left);
  cbEncoderGPIOinit(&cb_encoder_right);
  gpioSetISRFuncEx(cb_encoder_left.pin_a, EITHER_EDGE, 50,
                   cb_encoder_callback_isrA, &cb_encoder_left);
  gpioSetISRFuncEx(cb_encoder_left.pin_b, EITHER_EDGE, 50,
                   cb_encoder_callback_isrB, &cb_encoder_left);
  gpioSetISRFuncEx(cb_encoder_right.pin_a, EITHER_EDGE, 50,
                   cb_encoder_callback_isrA, &cb_encoder_right);
  gpioSetISRFuncEx(cb_encoder_right.pin_b, EITHER_EDGE, 50,
                   cb_encoder_callback_isrB, &cb_encoder_right);

  OS_Handle encoder = os_thread_start(encoder_task, 0);
  OS_Handle odometry = os_thread_start(odometry_task, 0);
  os_sleep_milliseconds(cli->count < 1 ? 10 : i64_from_str8(cli->args[0]));
  os_thread_kill(encoder);
  os_thread_kill(odometry);
}

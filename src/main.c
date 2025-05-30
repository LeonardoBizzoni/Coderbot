#include <pigpio.h>
#include <stdio.h>

#include <base/base_inc.h>
#include <OS/os_inc.h>
#include <base/base_inc.c>
#include <OS/os_inc.c>

#include "coderbot.h"
#include "odometry.h"
#include "encoder.h"

fn void deadline_handler(i32 sig) {
  fprintf(stderr, "\n\tA task missed its deadline!\n");
  cb_stop();
  os_exit(-1);
}

global u64 measured_ticks_left = 0;
global u64 measured_ticks_right = 0;

global OS_Handle tick_mutex;

#include "coderbot.c"
#include "odometry.c"
#include "encoder.c"

fn void start(CmdLine *cli) {
  tick_mutex = os_mutex_alloc();
  // giusto per vedere se è il codebase il problema
  if (!tick_mutex.h[0]) {
    fprintf(stderr, "`os_mutex_alloc` failed.\n");
    os_exit(-1);
  }

  i32 version = gpioInitialise();
  if (version < 0) {
    fprintf(stderr, "non va pigpio\n");
    os_exit(-1);
  } else {
    printf("pigpio version: %d\n", version);
  }
  cbMotorGPIOinit(&cb_motor_left);
  cbMotorGPIOinit(&cb_motor_right);
  cbEncoderGPIOinit(&cb_encoder_left);
  cbEncoderGPIOinit(&cb_encoder_right);
  os_atexit(cb_stop);

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
  // giusto per vedere se è il codebase il problema
  if (!encoder.h[0] || !odometry.h[0]) {
    fprintf(stderr, "`os_thread_start` failed.\n");
    os_exit(-1);
  }

  os_sleep_milliseconds(5 * 1e3);
  os_thread_kill(encoder);
  os_thread_kill(odometry);

  cb_stop();
}

#include <stdio.h>

#include <base/base_inc.h>
#include <OS/os_inc.h>
#include <base/base_inc.c>
#include <OS/os_inc.c>

#include "coderbot.h"
#include "odometry.h"
#include "encoder.h"
#include "cartesian_controller.h"

fn void deadline_handler(i32 sig) {
  fprintf(stderr, "\n\tA task missed its deadline!\n");
  cb_stop();
  os_exit(-1);
}

global struct {
  struct {
    OS_Handle mutex;
    u64 measured_left;
    u64 measured_right;
  } tick;
  struct {
    f32 left;
    f32 right;
  } distance_traveled; // in mm
  struct {
    OS_Handle mutex;
    f32 dof[3]; // VETTORE per POSE (totale)
  } pose;
  struct {
    OS_Handle mutex;
    f32 left;
    f32 right;
  } speed;
} state = {0};

#include "coderbot.c"
#include "odometry.c"
#include "encoder.c"
#include "cartesian_controller.c"

// task: cartesian controller (local planner)
// task: controllo encoder
// task: odometria

void start(CmdLine *cmd) {
  state.tick.mutex = os_mutex_alloc();
  state.pose.mutex = os_mutex_alloc();
  state.speed.mutex = os_mutex_alloc();

  i32 version = gpioInitialise();
  if (version < 0) {
    printf("gpioInitialise ha avuto un errore\n");
    os_exit(-1);
  } else {
    printf("version gpioInitialise: %d\n", version);
  }

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

  OS_Handle encoder_thd = os_thread_start(encoder_task, 0);
  OS_Handle odometry_thd = os_thread_start(odometry_task, 0);
  OS_Handle cartesian_thd = os_thread_start(cartesian_task, 0);

  os_sleep_milliseconds(4 * 1e3);
  os_thread_cancel(encoder_thd);
  os_thread_cancel(odometry_thd);
  os_thread_cancel(cartesian_thd);
  cb_stop();
}

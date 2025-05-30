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

void start(CmdLine *cmd) {
  gpioInitialise();
  cbMotorGPIOinit(&cb_motor_left);
  cbMotorGPIOinit(&cb_motor_right);
  cbEncoderGPIOinit(&cb_encoder_left);
  cbEncoderGPIOinit(&cb_encoder_right);
  os_atexit(cb_stop);

  f64 duty_cicle_from_ticks_left = 0;

  gpioSetISRFuncEx(cb_encoder_left.pin_a, EITHER_EDGE, 50,
                   cb_encoder_callback_isrA, &cb_encoder_left);
  gpioSetISRFuncEx(cb_encoder_left.pin_b, EITHER_EDGE, 50,
                   cb_encoder_callback_isrB, &cb_encoder_left);
  gpioSetISRFuncEx(cb_encoder_right.pin_a, EITHER_EDGE, 50,
                   cb_encoder_callback_isrA, &cb_encoder_right);
  gpioSetISRFuncEx(cb_encoder_right.pin_b, EITHER_EDGE, 50,
                   cb_encoder_callback_isrB, &cb_encoder_right);
  f64 duty_cicle_from_ticks_right = 0;
  if (cmd->count < 2) {
    duty_cicle_from_ticks_left = Duty_CycleFromTicks_Left;
    duty_cicle_from_ticks_right = Duty_CycleFromTicks_Right;
  } else {
    duty_cicle_from_ticks_left = f64_from_str8(cmd->args[0]);
    duty_cicle_from_ticks_right = f64_from_str8(cmd->args[1]);
  }

  gpioSetISRFuncEx(cb_encoder_left.pin_a, EITHER_EDGE, 50,
                   cb_encoder_callback_isrA, &cb_encoder_left);
  gpioSetISRFuncEx(cb_encoder_left.pin_b, EITHER_EDGE, 50,
                   cb_encoder_callback_isrB, &cb_encoder_left);
  gpioSetISRFuncEx(cb_encoder_right.pin_a, EITHER_EDGE, 50,
                   cb_encoder_callback_isrA, &cb_encoder_right);
  gpioSetISRFuncEx(cb_encoder_right.pin_b, EITHER_EDGE, 50,
                   cb_encoder_callback_isrB, &cb_encoder_right);

  i64 period_ms = 20;
  // (cm/s) / (mm/tick / 10mm) = (cm/s) * (tick/cm) = tick/s
  f64 target_ticksXsec_left = TargetSpeed / (MillimeterFromTicks_Left / 10.);
  f64 target_ticksXsec_right = TargetSpeed / (MillimeterFromTicks_Right / 10.);
  // (tick/s) * (ms / 1000ms) = tick
  u64 target_ticks_left = target_ticksXsec_left * ((f64)period_ms / 1000.);
  u64 target_ticks_right = target_ticksXsec_right * ((f64)period_ms / 1000.);

  cbDir_t motor_direction_left = forward, motor_direction_right = forward;
  f64 duty_cycle_left = InitialDutyCycle, duty_cycle_right = InitialDutyCycle;
  i64 accumalated_error_left = 0, accumalated_error_right = 0, activations = 0;

  printf("tick left attesi per sec: %lf\n", target_ticksXsec_left);
  printf("tick right attesi per sec: %lf\n", target_ticksXsec_right);
  printf("tick left attesi: %ld\n", target_ticks_left);
  printf("tick right attesi: %ld\n", target_ticks_right);

  for (OS_Handle timer = os_timer_start();
       !os_timer_elapsed(OS_TimerGranularity_sec, timer, 2);) {
    OS_Handle exec_timer_start = os_timer_start();

    i64 delta_left = target_ticks_left - cb_encoder_left.ticks;
    i64 delta_right = target_ticks_right - cb_encoder_right.ticks;

    activations += 1;
    accumalated_error_left += delta_left;
    accumalated_error_right += delta_right;

    duty_cycle_left = duty_cicle_from_ticks_left * (f64)delta_left;
    duty_cycle_right = duty_cicle_from_ticks_right * (f64)delta_right;

    /* printf("duty_cycle_too_high_counter: %ld\n", duty_cycle_too_high_counter); */
    printf("Left:\n\ttarget ticks: %ld\n\tmeasured ticks: %ld\n\tdelta: %ld"
           "\n\taccumulated error: %ld\n\taccumulated error average: %lf"
           "\n\tduty cycle uncapped: %lf\n",
           target_ticks_left, cb_encoder_left.ticks, delta_left, accumalated_error_left,
           (f64)accumalated_error_left / (f64)activations, duty_cycle_left);
    printf("Right:\n\ttarget ticks: %ld\n\tmeasured ticks: %ld\n\tdelta: %ld"
           "\n\taccumulated error: %ld\n\taccumulated error average: %lf"
           "\n\tduty cycle uncapped: %lf\n",
           target_ticks_right, cb_encoder_right.ticks, delta_right, accumalated_error_right,
           (f64)accumalated_error_right / (f64)activations, duty_cycle_right);

    motor_direction_left = duty_cycle_left < 0 ? backward : forward;
    motor_direction_right = duty_cycle_right < 0 ? backward : forward;

    duty_cycle_left = Abs(duty_cycle_left);
    duty_cycle_right = Abs(duty_cycle_right);
    duty_cycle_left = Clamp(Abs(duty_cycle_left), 0.1, 0.6);
    duty_cycle_right = Clamp(Abs(duty_cycle_right), 0.1, 0.6);

    printf("Left duty cycle will be: %lf\n", duty_cycle_left);
    printf("Right duty cycle will be: %lf\n\n", duty_cycle_right);

    cb_encoder_left.ticks = 0;
    cb_encoder_right.ticks = 0;
    cbMotorMove(&cb_motor_left, motor_direction_left, duty_cycle_left);
    cbMotorMove(&cb_motor_right, motor_direction_right, duty_cycle_right);

    OS_Handle exec_timer_end = os_timer_start();
    i64 task_exec_time = os_timer_elapsed(OS_TimerGranularity_ms,
                                          exec_timer_start, exec_timer_end);
    os_sleep_milliseconds(ClampBot(period_ms - task_exec_time, 0));
  }
}

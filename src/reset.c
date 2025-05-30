#include <stdio.h>

#include <base/base_inc.h>
#include <OS/os_inc.h>
#include <base/base_inc.c>
#include <OS/os_inc.c>

#include "coderbot.h"
#include "coderbot.c"

void start(CmdLine *cmd) {
  gpioInitialise();
  cbMotorGPIOinit(&cb_motor_left);
  cbMotorGPIOinit(&cb_motor_right);
  cbEncoderGPIOinit(&cb_encoder_left);
  cbEncoderGPIOinit(&cb_encoder_right);

  cbMotorReset(&cb_motor_left);
  cbMotorReset(&cb_motor_right);
  gpioSetISRFuncEx(cb_encoder_left.pin_a, EITHER_EDGE, 1000, 0, 0);
  gpioSetISRFuncEx(cb_encoder_left.pin_b, EITHER_EDGE, 1000, 0, 0);
  gpioSetISRFuncEx(cb_encoder_right.pin_a, EITHER_EDGE, 1000, 0, 0);
  gpioSetISRFuncEx(cb_encoder_right.pin_b, EITHER_EDGE, 1000, 0, 0);
  gpioTerminate();
}

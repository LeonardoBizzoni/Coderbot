global cbEncoder_t cb_encoder_left =
  { PIN_ENCODER_LEFT_A, PIN_ENCODER_LEFT_B, -1 };
global cbEncoder_t cb_encoder_right =
  { PIN_ENCODER_RIGHT_A, PIN_ENCODER_RIGHT_B, -1};

global cbMotor_t cb_motor_left =
  { PIN_LEFT_FORWARD, PIN_LEFT_BACKWARD, forward };
global cbMotor_t cb_motor_right =
  { PIN_RIGHT_FORWARD, PIN_RIGHT_BACKWARD, forward };

fn void cb_stop(void) {
  gpioSetISRFuncEx(cb_encoder_left.pin_a, EITHER_EDGE, 1000, 0, 0);
  gpioSetISRFuncEx(cb_encoder_left.pin_b, EITHER_EDGE, 1000, 0, 0);
  gpioSetISRFuncEx(cb_encoder_right.pin_a, EITHER_EDGE, 1000, 0, 0);
  gpioSetISRFuncEx(cb_encoder_right.pin_b, EITHER_EDGE, 1000, 0, 0);
  cbMotorReset(&cb_motor_left);
  cbMotorReset(&cb_motor_right);
  gpioTerminate();
}

fn void cb_encoder_callback_isrA(i32 gpio, i32 level, u32 tick, void *_enc) {
  cbEncoder_t *enc = _enc;
  if (!enc || gpio == enc->last_gpio) { return; }
  enc->last_gpio = gpio;
  enc->level_a = level;
  if(level ^ enc->level_b) {
    enc->direction = forward;
    os_mutex_lock(state.tick.mutex);
    enc->ticks += forward;
    os_mutex_unlock(state.tick.mutex);
  }
}

fn void cb_encoder_callback_isrB(i32 gpio, i32 level, u32 tick, void *_enc) {
  cbEncoder_t *enc = _enc;
  if (!enc || gpio == enc->last_gpio) { return; }
  enc->last_gpio = gpio;
  enc->level_b = level;
  if(level ^ enc->level_a) {
    enc->direction = backward;
    os_mutex_lock(state.tick.mutex);
    enc->ticks += backward;
    os_mutex_unlock(state.tick.mutex);
  }
}

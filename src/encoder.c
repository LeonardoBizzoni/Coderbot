fn void encoder_task(void *_args) {
  f64 target_speed_mms = 70.0;
  f64 period_hz = 500.0;
  f64 period_ms = (1.0 / period_hz) * 1000.0;
  f64 target_ticksXsec_left = target_speed_mms / MillimeterFromTicks_Left;
  f64 target_ticksXsec_right = target_speed_mms / MillimeterFromTicks_Right;
  f64 target_ticks_left = target_ticksXsec_left * (period_ms / 1000.0);
  f64 target_ticks_right = target_ticksXsec_right * (period_ms / 1000.0);

  i64 accumalated_error_left = 0, accumalated_error_right = 0;
  cbDir_t motor_direction_left = forward, motor_direction_right = forward;
  f64 duty_cycle_left = InitialDutyCycle, duty_cycle_right = InitialDutyCycle;

  /* runtime (WCET) ancora da misurare */
  /* lnx_sched_set_deadline(1e6, period_ms * 1e9, period_ms * 1e9, deadline_handler); */
  lnx_sched_set_deadline(2 * 1e9, 2 * 1e9, 2 * 1e9, deadline_handler);

  for (;;) {
    os_mutex_lock(tick_mutex);
    measured_ticks_left = cb_encoder_left.ticks;
    measured_ticks_right = cb_encoder_right.ticks;
    os_mutex_unlock(tick_mutex);

    i64 delta_left = target_ticks_left - measured_ticks_left;
    i64 delta_right = target_ticks_right - measured_ticks_right;

    duty_cycle_left = ((f64)delta_left * Kp_Left) + (accumalated_error_left * Ki_Left);
    duty_cycle_right = ((f64)delta_right * Kp_Right) + (accumalated_error_right * Ki_Right);
    accumalated_error_left += delta_left;
    accumalated_error_right += delta_right;

    /* TODO(lb): add a lock for logging so its not messy? */
    /* printf("Left:\n\ttarget ticks: %ld\n\tmeasured ticks: %ld\n\tdelta: %ld" */
    /*        "\n\taccumulated error: %ld\n\tduty cycle uncapped: %lf\n", */
    /*        target_ticks_left, encoder_left.ticks, delta_left, */
    /*        accumalated_error_left, duty_cycle_left); */
    /* printf("Right:\n\ttarget ticks: %ld\n\tmeasured ticks: %ld\n\tdelta: %ld" */
    /*        "\n\taccumulated error: %ld\n\tduty cycle uncapped: %lf\n", */
    /*        target_ticks_right, encoder_right.ticks, delta_right, */
    /*        accumalated_error_right, duty_cycle_right); */

    motor_direction_left = duty_cycle_left < 0 ? backward : forward;
    motor_direction_right = duty_cycle_right < 0 ? backward : forward;
    duty_cycle_left = Clamp(Abs(duty_cycle_left), 0.1, 0.6);
    duty_cycle_right = Clamp(Abs(duty_cycle_right), 0.1, 0.6);

    /* printf("[LEFT] Capped duty cycle will be: %lf\n", duty_cycle_left); */
    /* printf("[RIGHT] Capped duty cycle will be: %lf\n\n", duty_cycle_right); */

    cb_encoder_left.ticks = 0;
    cb_encoder_right.ticks = 0;
    cbMotorMove(&cb_motor_left, motor_direction_left, duty_cycle_left);
    cbMotorMove(&cb_motor_right, motor_direction_right, duty_cycle_right);

    os_thread_cancelpoint();
    lnx_sched_yield();
  }
}

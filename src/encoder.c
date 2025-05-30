fn void encoder_task(void *_args) {
  f64 duty_cicle_from_ticks_left = Duty_CycleFromTicks_Left;
  f64 duty_cicle_from_ticks_right = Duty_CycleFromTicks_Right;

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

  /* runtime (WCET) ancora da misurare */
  /* lnx_sched_set_deadline(1e6, period_ms * 1e9, period_ms * 1e9, deadline_handler); */
  lnx_sched_set_deadline(2 * 1e9, 2 * 1e9, 3 * 1e9, deadline_handler);

  for (;;) {
    os_mutex_lock(tick_mutex);
    measured_ticks_left = cb_encoder_left.ticks;
    measured_ticks_right = cb_encoder_right.ticks;
    os_mutex_unlock(tick_mutex);

    i64 delta_left = target_ticks_left - measured_ticks_left;
    i64 delta_right = target_ticks_right - measured_ticks_right;

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

    os_thread_cancelpoint();
    lnx_sched_yield();
  }
}

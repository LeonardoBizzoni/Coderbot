fn void encoder_task(void *_args) {
  i64 period_ms = 20;
  lnx_sched_set_deadline((period_ms - 1) * 1e6, (period_ms - 1) * 1e6,
                         period_ms * 1e6, deadline_handler);

  for (;;) {
    os_mutex_lock(state.speed.mutex);
    // (cm/s) / (mm/tick / 10mm) = (cm/s) * (tick/cm) = tick/s
    f32 target_ticksXsec_left = state.speed.left / (MillimeterFromTicks_Left / 10.);
    f32 target_ticksXsec_right = state.speed.right / (MillimeterFromTicks_Right / 10.);
    os_mutex_unlock(state.speed.mutex);

    // (tick/s) * (ms / 1000ms) = tick
    u64 target_ticks_left = target_ticksXsec_left * ((f32)period_ms / 1000.);
    u64 target_ticks_right = target_ticksXsec_right * ((f32)period_ms / 1000.);

    cbDir_t motor_direction_left = forward, motor_direction_right = forward;
    f32 duty_cycle_left = InitialDutyCycle, duty_cycle_right = InitialDutyCycle;
    i64 accumalated_error_left = 0, accumalated_error_right = 0, activations = 0;

    os_mutex_lock(state.tick.mutex);
    state.tick.measured_left = cb_encoder_left.ticks;
    state.tick.measured_right = cb_encoder_right.ticks;
    os_mutex_unlock(state.tick.mutex);

    i64 delta_left = target_ticks_left - state.tick.measured_left;
    i64 delta_right = target_ticks_right - state.tick.measured_right;

    activations += 1;
    accumalated_error_left += delta_left;
    accumalated_error_right += delta_right;

    duty_cycle_left = Kp_Left * delta_left + Ki_Left * accumalated_error_left;
    duty_cycle_right = Kp_Right * delta_right + Ki_Right * accumalated_error_right;

    printf("Left:\n\ttarget ticks: %ld\n\tmeasured ticks: %ld\n\tdelta: %ld"
           "\n\taccumulated error: %ld\n\taccumulated error average: %lf"
           "\n\tduty cycle uncapped: %lf\n",
           target_ticks_left, cb_encoder_left.ticks, delta_left, accumalated_error_left,
           (f32)accumalated_error_left / (f32)activations, duty_cycle_left);
    printf("Right:\n\ttarget ticks: %ld\n\tmeasured ticks: %ld\n\tdelta: %ld"
           "\n\taccumulated error: %ld\n\taccumulated error average: %lf"
           "\n\tduty cycle uncapped: %lf\n",
           target_ticks_right, cb_encoder_right.ticks, delta_right, accumalated_error_right,
           (f32)accumalated_error_right / (f32)activations, duty_cycle_right);

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

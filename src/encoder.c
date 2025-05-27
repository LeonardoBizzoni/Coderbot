fn void encoder_task(void *_args) {
  //                     runtime ≤ deadline ≤ period
  lnx_sched_set_deadline(  1e6,      1e6,   0.5 * 1e9, deadline_handler);
  /* Si questi valori di runtime/deadline/period sono a cazzo di cane
   * e si dobbiamo misurarli. */

  for (;;) {
    Info("Task per controllo encoder");
    lnx_sched_yield();
  }
}

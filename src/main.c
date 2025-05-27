#include <stdio.h>

#include <base/base_inc.h>
#include <OS/os_inc.h>
#include <base/base_inc.c>
#include <OS/os_inc.c>

fn void deadline_handler(i32 sig) {
  Err("A task missed its deadline!");
  (void)signal(SIGXCPU, SIG_DFL);
}

#include "odometry.c"
#include "encoder.c"

fn void start(CmdLine *cli) {
  OS_Handle encoder = os_thread_start(encoder_task, 0);
  OS_Handle odometry = os_thread_start(odometry_task, 0);

  os_thread_join(encoder);
  os_thread_join(odometry);
}

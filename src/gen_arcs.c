#include <base/base_inc.h>
#include <OS/os_inc.h>
#include <base/base_inc.c>
#include <OS/os_inc.c>

#include "cartesian_controller.h"

#define Chunks 22

global struct {
  struct {f32 x, y;} waypoints[N_POINTS * Chunks];
} state = {0};

#include "cartesian_controller.c"

fn void start(CmdLine *cmd) {
  generate_arc_points(0, 900, 900, -90.f, -270.f); // C
  generate_line_points(400, 0, 1800, 90.f); // I
  { // A
    generate_line_points(600, 0, 1800, 75.f);
    generate_line_points(600 + 1800 * cosf(75.f * M_PI / 180.f), 1800, 1800, -75.f);
    generate_line_points(600 + 900 * cosf(75.f * M_PI / 180.f), 900,
                         2 * 900 * cosf(75.f * M_PI / 180.f), 0.f);
  }
  generate_arc_points(3000, 900, 900, 0.f, 360.f); // O

  generate_arc_points(6000, 900, 900, -90.f, -270.f); // C
  { // A
    generate_line_points(6000 + 400, 0, 1800, 75.f);
    generate_line_points(6000 + 400 + 1800 * cosf(75.f * M_PI / 180.f), 1800, 1800, -75.f);
    generate_line_points(6000 + 400 + 900 * cosf(75.f * M_PI / 180.f), 900,
                         2 * 900 * cosf(75.f * M_PI / 180.f), 0.f);
  }
  { // M
    generate_line_points(7700, 0, 1800, 90.f); // I
    generate_line_points(7700, 1800, 900, -45.f); // "\"
    generate_line_points(7700 + 900 * cosf(45.f * M_PI / 180.f),
                         1800 - 900 * sinf(45.f * M_PI / 180.f), 900, 45.f); // "/"
    generate_line_points(7700 + 2 * 900 * cosf(45.f * M_PI / 180.f), 0, 1800, 90.f); // I
  }
  generate_line_points(9400, 0, 1800, 90.f); // I
  { // L
    generate_line_points(10000, 0, 1800, 90.f); // I
    generate_line_points(10000, 0, 1800, 0.f); // _
  }
  { // L
    generate_line_points(10000 + 1800 + 400, 0, 1800, 90.f); // I
    generate_line_points(10000 + 1800 + 400, 0, 1800, 0.f); // _
  }
  { // A
    generate_line_points(14400, 0, 1800, 75.f);
    generate_line_points(14400 + 1800 * cosf(75.f * M_PI / 180.f), 1800, 1800, -75.f);
    generate_line_points(14400 + 900 * cosf(75.f * M_PI / 180.f), 900,
                         2 * 900 * cosf(75.f * M_PI / 180.f), 0.f);
  }

  for (i32 i = 0; i < N_POINTS * Chunks; ++i) {
    printf("(%f, %f), ", state.waypoints[i].x, state.waypoints[i].y);
  }
  printf("\b\b\n");
}

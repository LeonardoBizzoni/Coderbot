#include <base/base_inc.h>
#include <OS/os_inc.h>
#include <base/base_inc.c>
#include <OS/os_inc.c>

#include "cartesian_controller.h"

#define Chunks 1
#define N_POINTS 50

global struct {f32 x, y;} waypoints[N_POINTS * Chunks] = {0};

global i32 chunk = 0;
fn void generate_arc_points(f32 center_x, f32 center_y, f32 radius,
                            f32 start_angle, f32 end_angle) {
  f32 start_radians = start_angle * M_PI / 180.f;
  f32 end_radians = end_angle * M_PI / 180.f;
  for (i32 i = 0; i < N_POINTS; ++i) {
    f32 t = (f32)i / (N_POINTS - 1);
    f32 radians = start_radians + t * (end_radians - start_radians);
    waypoints[i + chunk * N_POINTS].x = center_x + radius * cosf(radians);
    waypoints[i + chunk * N_POINTS].y = center_y + radius * sinf(radians);
  }
  chunk += 1;
}

fn void generate_line_points(f32 start_x, f32 start_y, f32 length_mm, f32 angle) {
  f32 radians = angle * M_PI / 180.f;
  for (i32 i = 0; i < N_POINTS; ++i) {
    f32 amount = ClampTop(i * (length_mm / N_POINTS), length_mm);
    waypoints[i + chunk * N_POINTS].x = start_x + amount * cosf(radians);
    waypoints[i + chunk * N_POINTS].y = start_y + amount * sinf(radians);
  }
  chunk += 1;
}

fn void start(CmdLine *cmd) {
  Arena *arena = ArenaBuild();

  OS_Handle trajectory = fs_open(Strlit("src/trajectory.h"), OS_acfWrite);
  fs_write(trajectory, Strlit("#ifndef GEN_TRAJECTORY_H\n#define GEN_TRAJECTORY_H\n"));
  fs_write(trajectory, str8_format(arena, "#define N_POINTS %d\n", N_POINTS * Chunks));

  generate_arc_points(0, -900, 900, 90.f, 0.f);

  fs_write(trajectory, Strlit("global Points waypoints[N_POINTS] = {"));
  for (i32 i = 0; i < N_POINTS * Chunks; ++i) {
    printf("(%f, %f), ", waypoints[i].x, waypoints[i].y);
    fs_write(trajectory, str8_format(arena, "{%f,%f}", waypoints[i].x, waypoints[i].y));
    if (i < (N_POINTS * Chunks) - 1) {
      fs_write(trajectory, Strlit(","));
    }
  }
  printf("\b\b\n");

  fs_write(trajectory, Strlit("};\n#endif\n"));
  fs_close(trajectory);
}

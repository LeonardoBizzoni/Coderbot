#ifndef CARTESIAN_CONTROLLER_H
#define CARTESIAN_CONTROLLER_H

// TOLLERANZA dell'(errore dell')ANGOLO (1°)
#define ANGLE_TOLLERANCE 0.0175
#define WAYPOINTS_WINDOW 10

typedef struct {
  f32 x, y;
} Points;

fn i32 nearest_point_position(f32 *pose_dof, i32 last_idx);

#endif

#ifndef CARTESIAN_CONTROLLER_H
#define CARTESIAN_CONTROLLER_H

// TOLLERANZA dell'(errore dell')ANGOLO (1°)
#define ANGLE_TOLLERANCE 0.0175
#define N_POINTS 50

typedef struct {
  f32 x, y;
} Points;

fn void generate_line_points(f32 start_x, f32 start_y, f32 length_mm, f32 angle);
fn void generate_arc_points(f32 center_x, f32 center_y, f32 radius,
                            f32 start_angle, f32 end_angle);

fn i32 nearest_point_position(f32 *pose_dof);

#endif

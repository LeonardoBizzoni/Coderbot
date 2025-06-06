#ifndef ODOMETRY_H
#define ODOMETRY_H

#define THRESHOLD 0.005

fn void pose_update_from_ticks(int delta_left_ticks, int delta_right_ticks);

fn void pose_print_info(void);
fn void pose_update_from_ticks(int delta_left_ticks, int delta_right_ticks);

fn void moltiplica_matrici_3x3(f64 lhs[3][3], f64 rhs[3][3], f64 output[3][3]);

#endif

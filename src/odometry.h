#ifndef ODOMETRY_H
#define ODOMETRY_H

fn void pose_update_from_ticks(int delta_left_ticks, int delta_right_ticks);

fn void pose_print_info(void);
fn void pose_update_from_ticks(int delta_left_ticks, int delta_right_ticks);

fn void prod_matrix_4_4(double res[4][4], double A[4][4], double B[4][4]);

#endif

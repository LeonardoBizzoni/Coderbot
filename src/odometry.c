#include <math.h>
#include <stdio.h>

fn void odometry_task(void *_args) {
  //                     runtime ≤ deadline ≤ period
  lnx_sched_set_deadline(  1e6,      1e6,      1e9, deadline_handler);
  /* Si questi valori di runtime/deadline/period sono a cazzo di cane
   * e si dobbiamo misurarli. */

  for (;;) {
    Info("Task per odometria");
    lnx_sched_yield();
  }
}


// funzione di moltiplicazione matrici 4x4
fn void prod_matrix_4_4(double res[4][4], double A[4][4], double B[4][4]) {
  memZero(res, sizeof(double[4][4]));
  for(int i = 0; i < 4; ++i) {
    for(int j = 0; j < 4; ++j) {
      for(int k = 0; k < 4; ++k) {
        res[i][j] += A[i][k] * B[k][j];
      }
    }
  }
}

// aggiorna la pose usando i tick encoder
fn void pose_update_from_ticks(double pose[4][4], int delta_left_ticks, int delta_right_ticks) {

    // Calcola la distanza percorsa da ciascuna ruota in mm
    double dL = delta_left_ticks * MillimeterFromTicks_Left;
    double dR = delta_right_ticks * MillimeterFromTicks_Right;

    // Calcola la distanza percorsa dal centro del robot
    double d = (dL + dR) / 2.0;

    // Calcola la variazione dell'angolo di orientamento (in radianti)
    // in base alla differenza tra le due ruote
    double delta_theta = (dR - dL) / BASELINE_MM;

    // Costruisce la matrice omogenea 4x4 di roto-traslazione T
    double T[4][4] = {
        {cos(delta_theta), -sin(delta_theta), 0, d},
        {sin(delta_theta),  cos(delta_theta), 0, 0},
        {0,                 0,                1, 0},
        {0,                 0,                0, 1}
    };

    prod_matrix_4_4(pose, pose, T);
}

// stampa la posizione del robot + angoli RPY e ZYZ
fn void pose_print_info(double pose[4][4]) {
    double x = pose[0][3];
    double y = pose[1][3];
    double z = pose[2][3];

    // Angoli ZYZ (Euler)
    double theta_z1 = atan2(pose[1][2], pose[0][2]);
    double theta_y  = atan2(sqrt(pow(pose[0][2],2)+pow(pose[1][2],2)), pose[2][2]);
    double theta_z2 = atan2(pose[2][1], -pose[2][0]);

    // Angoli RPY (Roll-Pitch-Yaw)
    double roll  = atan2(pose[1][0], pose[0][0]);
    double pitch = atan2(-pose[2][0], sqrt(pow(pose[2][1],2)+pow(pose[2][2],2)));
    double yaw   = atan2(pose[2][1], pose[2][2]);

    /* TODO(lb): add a lock to avoid messy logging? */
    printf("\nPosizione → x = %.2f mm, y = %.2f mm, z = %.2f mm\n", x, y, z);
    printf("ZYZ Euler (°): Z1 = %.2f, Y = %.2f, Z2 = %.2f\n",
           theta_z1 * 180/M_PI, theta_y * 180/M_PI, theta_z2 * 180/M_PI);
    printf("RPY (°): Roll = %.2f, Pitch = %.2f, Yaw = %.2f\n\n",
           roll * 180/M_PI, pitch * 180/M_PI, yaw * 180/M_PI);
}

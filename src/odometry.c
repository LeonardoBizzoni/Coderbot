#include <math.h>
#include <stdio.h>

global f64 pose[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};      // MATRICE per POSE (totale)
global f64 pose_dof[3] = {0, 0, 0,};                            // VETTORE per POSE (totale)
global f64 new_pose[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // MATRICE per POSE (corrente)

// MATRICI di SUPPORTO
global f64 rt[3][3],   // rototraslazione
           r[3][3],    // rotazione
           t1[3][3],   // traslazione al CIR
           t2[3][3],   // traslazione dal CIR
           temp[3][3]; // temporanea

fn void odometry_task(void *_args) {
  //                     runtime ≤ deadline ≤ period
  lnx_sched_set_deadline(2 * 1e9, 2 * 1e9, 2 * 1e9, deadline_handler);
  /* Si questi valori di runtime/deadline/period sono a cazzo di cane
   * e si dobbiamo misurarli. */

  for (;;) {
    os_mutex_lock(tick_mutex);
    u64 ticks_left = measured_ticks_left;
    u64 ticks_right = measured_ticks_right;
    os_mutex_unlock(tick_mutex);

    /* ODOMETRIA */
    // calcolo MILLIMETRI percorsi
    millimeter_traveled_left = ticks_left * MillimeterFromTicks_Left; // ruota sinistra
    millimeter_traveled_right = ticks_right * MillimeterFromTicks_Right; // ruota destra

    // calcolo ANGOLO
    f64 delta_theta = -(millimeter_traveled_left - millimeter_traveled_right) / BASELINE_MM;

    // SOGLIA (ticks sinistra e destra mai esattamente uguali, anche se dritto)
    if (Abs(delta_theta) < THRESHOLD) {
      // movimento: DRITTO
      // calcolo matrice di ROTOTRASLAZIONE corrente
      rt[0][0] = 1; rt[0][1] = 0; rt[0][2] = (millimeter_traveled_left + millimeter_traveled_right) / 2;
      rt[1][0] = 0; rt[1][1] = 1; rt[1][2] = 0;
      rt[2][0] = 0; rt[2][1] = 0; rt[2][2] = 1;
    } else {
      // movimento: CURVANDO
      f64 d = (millimeter_traveled_right / delta_theta) - (BASELINE_MM / 2);
      // TRASLAZIONE al CIR
      t1[0][0] = 1; t1[0][1] = 0; t1[0][2] = 0;
      t1[1][0] = 0; t1[1][1] = 1; t1[1][2] = -d;
      t1[2][0] = 0; t1[2][1] = 0; t1[2][2] = 1;
      // ROTAZIONE
      r[0][0] = cos(delta_theta); r[0][1] = -sin(delta_theta); r[0][2] = 0;
      r[1][0] = sin(delta_theta); r[1][1] = cos(delta_theta); r[1][2] = 0;
      r[2][0] = 0; r[2][1] = 0; r[2][2] = 1;
      // TRASLAZIONE dal CIR
      t2[0][0] = 1; t2[0][1] = 0; t2[0][2] = 0;
      t2[1][0] = 0; t2[1][1] = 1; t2[1][2] = d;
      t2[2][0] = 0; t2[2][1] = 0; t2[2][2] = 1;
      // calcolo matrice di ROTOTRASLAZIONE corrente
      moltiplica_matrici_3x3(t2, r, temp);
      moltiplica_matrici_3x3(temp, t1, rt);
    }

    // calcolo (nuova) POSE
    moltiplica_matrici_3x3(pose, rt, new_pose);
    for (usize r = 0; r < 3; r++) {
      for (usize c = 0; c < 3; c++) {
        pose[r][c] = new_pose[r][c];
      }
    }

    // creazione VETTORE per POSE
    pose_dof[0] = pose[0][2]; // posizione x
    pose_dof[1] = pose[1][2]; // posizione y
    pose_dof[2] = atan2(pose[1][0], pose[0][0]); // angolo theta

    os_thread_cancelpoint();
    lnx_sched_yield();
  }
}

fn void moltiplica_matrici_3x3(f64 lhs[3][3], f64 rhs[3][3], f64 output[3][3]) {
  memZero(output, sizeof(f64[3][3]));
  for (usize i = 0; i < 3; ++i) {
    for (usize j = 0; j < 3; ++j) {
      for (usize r = 0; r < 3; ++r) {
        output[i][j] += lhs[i][r] * rhs[r][j];
      }
    }
  }
}

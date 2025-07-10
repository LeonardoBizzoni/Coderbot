#include <math.h>
#include <stdio.h>

global f32 pose[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};      // MATRICE per POSE (totale)
global f32 new_pose[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // MATRICE per POSE (corrente)

// dichiarazione delle matrici usate
global f32 t_fin[3][3] = {0}, // rototraslazione
           r[3][3]     = {0}, // rotazione
           t1[3][3]    = {0}, // traslazione al CIR
           t2[3][3]    = {0}, // traslazione dal CIR
           temp[3][3]  = {0}; // temporanea

fn void odometry_task(void *_args) {
  lnx_sched_set_deadline(2 * 1e6, 30 * 1e6, 30 * 1e6, deadline_handler);

  state.distance_traveled.left = 0;
  state.distance_traveled.right = 0;

  for (;;) {
    os_mutex_lock(state.tick.mutex);
    i64 ticks_left = state.tick.measured_left;
    i64 ticks_right = state.tick.measured_right;
    state.tick.measured_left = 0;
    state.tick.measured_right = 0;
    os_mutex_unlock(state.tick.mutex);

    // ODOMETRIA
    // calcolo della distanza percorsa in millimetri della ruota sinistra e destra
    f32 distance_left = ticks_left * MillimeterFromTicks_Left;
    f32 distance_right = ticks_right * MillimeterFromTicks_Right;

    state.distance_traveled.left += distance_left;
    state.distance_traveled.right += distance_right;

    // calcolo dell'angolo di rotazione
    f32 theta = -(distance_left - distance_right) / BASELINE_MM;

    // soglia di tolleranza -> controllo la direzione del robot se va dritto
    if (Abs(theta) < TOLERANCE) {
      /* traslazione lungo x, calcolo della matrice di rototraslazione per traiettoria dritta */
      t_fin[0][0] = 1; t_fin[0][1] = 0; t_fin[0][2] = (distance_left + distance_right) / 2;
      t_fin[1][0] = 0; t_fin[1][1] = 1; t_fin[1][2] = 0;
      t_fin[2][0] = 0; t_fin[2][1] = 0; t_fin[2][2] = 1;
    } else {
      // traiettoria lungo una curva
      // distanza del centro del robot dal CIR
      f32 d_cir = (distance_right / theta) - (BASELINE_MM / 2);
      // matrice t1 di traslazione al CIR
      t1[0][0] = 1; t1[0][1] = 0; t1[0][2] = 0;
      t1[1][0] = 0; t1[1][1] = 1; t1[1][2] = -d_cir;
      t1[2][0] = 0; t1[2][1] = 0; t1[2][2] = 1;
      // matrice r di rotazione attorno al CIR
      r[0][0] = cos(theta); r[0][1] = -sin(theta); r[0][2] = 0;
      r[1][0] = sin(theta); r[1][1] = cos(theta);  r[1][2] = 0;
      r[2][0] = 0;          r[2][1] = 0;           r[2][2] = 1;
      // matrice t2 di ri-traslazione dal CIR
      t2[0][0] = 1; t2[0][1] = 0; t2[0][2] = 0;
      t2[1][0] = 0; t2[1][1] = 1; t2[1][2] = d_cir;
      t2[2][0] = 0; t2[2][1] = 0; t2[2][2] = 1;
      // calcolo matrice di ROTOTRASLAZIONE corrente
      prodotto_matrici_3x3(t2, r, temp);
      prodotto_matrici_3x3(temp, t1, t_fin);
    }

    // calcolo (nuova) POSE
    prodotto_matrici_3x3(pose, t_fin, new_pose);
    memcopy(pose, new_pose, sizeof(f32[3][3]));

    // creazione VETTORE per POSE
    DeferLoop(os_mutex_lock(state.pose.mutex), os_mutex_unlock(state.pose.mutex)) {
      state.pose.dof[0] = pose[0][2]; // posizione x
      state.pose.dof[1] = pose[1][2]; // posizione y
      state.pose.dof[2] = atan2(pose[1][0], pose[0][0]); // angolo theta

#ifdef ENABLE_ODOMETRY_PRINT
      printf("(%f, %f), ", state.pose.dof[0], state.pose.dof[1]);
#endif
    }

    os_thread_cancelpoint();
    lnx_sched_yield();
  }
}

fn void prodotto_matrici_3x3(f32 lhs[3][3], f32 rhs[3][3], f32 output[3][3]) {
  memzero(output, sizeof(f32[3][3]));
  output[0][0] = (lhs[0][0] * rhs[0][0]) + (lhs[0][1] * rhs[1][0]) + (lhs[0][2] * rhs[2][0]);
  output[0][1] = (lhs[0][0] * rhs[0][1]) + (lhs[0][1] * rhs[1][1]) + (lhs[0][2] * rhs[2][1]);
  output[0][2] = (lhs[0][0] * rhs[0][2]) + (lhs[0][1] * rhs[1][2]) + (lhs[0][2] * rhs[2][2]);

  output[1][0] = (lhs[1][0] * rhs[0][0]) + (lhs[1][1] * rhs[1][0]) + (lhs[1][2] * rhs[2][0]);
  output[1][1] = (lhs[1][0] * rhs[0][1]) + (lhs[1][1] * rhs[1][1]) + (lhs[1][2] * rhs[2][1]);
  output[1][2] = (lhs[1][0] * rhs[0][2]) + (lhs[1][1] * rhs[1][2]) + (lhs[1][2] * rhs[2][2]);

  output[2][0] = (lhs[2][0] * rhs[0][0]) + (lhs[2][1] * rhs[1][0]) + (lhs[2][2] * rhs[2][0]);
  output[2][1] = (lhs[2][0] * rhs[0][1]) + (lhs[2][1] * rhs[1][1]) + (lhs[2][2] * rhs[2][1]);
  output[2][2] = (lhs[2][0] * rhs[0][2]) + (lhs[2][1] * rhs[1][2]) + (lhs[2][2] * rhs[2][2]);
}

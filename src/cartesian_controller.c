fn i32 nearest_point_position(f32 *pose_dof, i32 last_idx) {
  i32 nearest = last_idx;
  f32 min_distance_squared = INFINITY;
  for (i32 i = ClampBot(0, last_idx - WAYPOINTS_WINDOW);
       i < ClampTop(N_POINTS, last_idx + WAYPOINTS_WINDOW);
       ++i) {
    f32 dx = waypoints[i].x - pose_dof[0];
    f32 dy = waypoints[i].y - pose_dof[1];
    f32 distance_squared = dx * dx + dy * dy;
    if (distance_squared < min_distance_squared) {
      min_distance_squared = distance_squared;
      nearest = i;
    }
  }

  return nearest;
}

#if PLATFORM_CODERBOT
fn void cartesian_task(void *_args) {
  lnx_sched_set_deadline(2 * 1e6, 30 * 1e6, 30 * 1e6, deadline_handler);
  i32 last_nearest_idx = 0;

  for (;;) {
    /* CARTESIAN CONTROLLER */
    // POSIZIONE CORRENTE REALE (da odometria) APPROSSIMATA AL PUNTO della TRAIETTORIA PIU' VICINO

    f32 pose[3] = {0};
    memzero(pose, sizeof(pose));
    DeferLoop(os_mutex_lock(state.pose.mutex), os_mutex_unlock(state.pose.mutex)) {
      memcopy(pose, state.pose.dof, sizeof(pose));
    }

    i32 current_position = nearest_point_position(pose, last_nearest_idx);
    last_nearest_idx = current_position;
    if(current_position >= N_POINTS - 3) {
      Info("Trajectory finished");
      DeferLoop(os_mutex_lock(state.speed.mutex), os_mutex_unlock(state.speed.mutex)) {
        state.speed.left = 0;
        state.speed.right = 0;
      }
      lnx_signal_send_private(SIGUSR1);
      return;
    }

    // waypoints[current_position + 3] POSIZIONE che si PUNTA (per evitare errori, non troppo vicina, quindi +3 posizioni)
    // waypoints[current_position].XY POSIZIONE CORRENTE REALE (da odometria) APPROSSIMATA AL PUNTO della TRAIETTORIA PIU' VICINO
    f32 delta_x = waypoints[current_position + 3].x - waypoints[current_position].x;
    f32 delta_y = waypoints[current_position + 3].y - waypoints[current_position].y;
    f32 desired_theta = atan2(delta_y, delta_x);
    f32 delta_theta_c = desired_theta - pose[2]; // errore dell'angolo

    // NORMALIZZAZIONE delta_theta_c in [-π, π]
    while (delta_theta_c > M_PI) { delta_theta_c -= 2 * M_PI; }
    while (delta_theta_c < -M_PI) { delta_theta_c += 2 * M_PI; }

    DeferLoop(os_mutex_lock(state.speed.mutex), os_mutex_unlock(state.speed.mutex)) {
      // CONFRONTO con TOLLERANZA dell'(errore dell')ANGOLO
      if (Abs(delta_theta_c) < ANGLE_TOLLERANCE) {
        // ANGOLO (piu' o meno) CORRETTO, si PROCEDE in LINEA RETTA per PUNTARE alla POSIZIONE
        state.speed.left = TargetSpeed;
        state.speed.right = TargetSpeed;
      } else {
        // CORREZIONE VELOCITA' per STERZARE
        if(delta_theta_c < 0) {
          // sterzare a DESTRA
          state.speed.left = TargetSpeed;
          state.speed.right = TargetSpeed * (1 / Abs((delta_theta_c * 180) / M_PI)); // correzione VELOCITA' PROPORZIONALE all'ERRORE
        } else {
          // sterzare a SINISTRA
          state.speed.left = TargetSpeed * (1 / Abs((delta_theta_c * 180) / M_PI)); // correzione VELOCITA' PROPORZIONALE all'ERRORE
          state.speed.right = TargetSpeed;
        }
      }
    }

    os_thread_cancelpoint();
    lnx_sched_yield();
  }
}
#endif

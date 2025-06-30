fn void generate_arc_points(f32 center_x, f32 center_y, f32 radius,
                            f32 start_angle, f32 end_angle) {
  f32 start_radians = start_angle * M_PI / 180.f;
  f32 end_radians = end_angle * M_PI / 180.f;
  for (i32 i = 0, j = N_POINTS - 1;
       i < N_POINTS && j >= 0;
       ++i, --j)
    {
      f32 t = (f32)i / (N_POINTS - 1);
      f32 radians = start_radians + t * (end_radians - start_radians);
      state.waypoints[j].x = center_x + radius * cosf(radians);
      state.waypoints[j].y = center_y + radius * sinf(radians);
    }
}

fn i32 nearest_point_position(f32 *pose_dof) {
  f32 min_distance_squared = (state.waypoints[0].x - pose_dof[0]) * (state.waypoints[0].x - pose_dof[0]) +
                             (state.waypoints[0].y - pose_dof[1]) * (state.waypoints[0].y - pose_dof[1]);
  i32 nearest_index = 0;
  for (i32 i = 1; i < N_POINTS; ++i) {
    f32 dx = state.waypoints[i].x - pose_dof[0];
    f32 dy = state.waypoints[i].y - pose_dof[1];
    f32 distance_squared = dx * dx + dy * dy;
    if (distance_squared < min_distance_squared) {
      min_distance_squared = distance_squared;
      nearest_index = i;
    }
  }

  return nearest_index;
}

fn void cartesian_task(void *_args) {
  lnx_sched_set_deadline(29 * 1e6, 30 * 1e6, 30 * 1e6, deadline_handler);
  for (;;) {
    /* CARTESIAN CONTROLLER */
    // POSIZIONE CORRENTE REALE (da odometria) APPROSSIMATA AL PUNTO della TRAIETTORIA PIU' VICINO

    f32 pose[3] = {0};
    memzero(pose, sizeof(pose));
    DeferLoop(os_mutex_lock(state.pose.mutex), os_mutex_unlock(state.pose.mutex)) {
      memcopy(pose, state.pose.dof, sizeof(pose));
    }

    i32 current_position = nearest_point_position(pose);
    if(current_position >= N_POINTS - 3) {
      DeferLoop(os_mutex_lock(state.speed.mutex), os_mutex_unlock(state.speed.mutex)) {
        state.speed.left = 0;
        state.speed.right = 0;
      }
      return;
    }

    // waypoints[current_position + 3] POSIZIONE che si PUNTA (per evitare errori, non troppo vicina, quindi +3 posizioni)
    // waypoints[current_position].XY POSIZIONE CORRENTE REALE (da odometria) APPROSSIMATA AL PUNTO della TRAIETTORIA PIU' VICINO
    f32 delta_x = state.waypoints[current_position + 3].x - state.waypoints[current_position].x;
    f32 delta_y = state.waypoints[current_position + 3].y - state.waypoints[current_position].y;
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

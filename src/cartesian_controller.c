global Points waypoints[N_POINTS] = {0};

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
      waypoints[j].x = center_x + radius * cosf(radians);
      waypoints[j].y = center_y + radius * sinf(radians);
    }
}

fn i32 nearest_point_position(f32 *pose_dof) {
  f32 min_distance_squared = (waypoints[0].x - pose_dof[0]) * (waypoints[0].x - pose_dof[0]) +
                             (waypoints[0].y - pose_dof[1]) * (waypoints[0].y - pose_dof[1]);
  i32 nearest_index = 0;
  for (i32 i = 1; i < N_POINTS; ++i) {
    f32 dx = waypoints[i].x - pose_dof[0];
    f32 dy = waypoints[i].y - pose_dof[1];
    f32 distance_squared = dx * dx + dy * dy;
    if (distance_squared < min_distance_squared) {
      min_distance_squared = distance_squared;
      nearest_index = i;
    }
  }

  return nearest_index;
}

fn void cartesian_task(void *args) {
  // creazione TRAIETTORIA come SUCCESSIONE di PUNTI nel PIANO CARTESIANO
  // centrato in (0cm, -90cm), raggio: 90cm, circonferenza considerata da 0° a 90°
  generate_arc_points(0, -900, 900, 0.f, 90.f);

  lnx_sched_set_deadline(2 * 1e6, 2 * 1e6, 3 * 1e6, deadline_handler);
  for (;;) {
    /* CARTESIAN CONTROLLER */
    // POSIZIONE CORRENTE REALE (da odometria) APPROSSIMATA AL PUNTO della TRAIETTORIA PIU' VICINO
    os_mutex_lock(pose_mutex);
    f32 pose[3] = {0};
    memCopy(pose, pose, 3 * sizeof(f32));
    os_mutex_unlock(pose_mutex);

    i32 current_position = nearest_point_position(pose);
    if(current_position >= N_POINTS - 3) {
      os_mutex_lock(speed_mutex);
      DeferLoop(os_mutex_unlock(speed_mutex)) {
        speed_left = 0;
        speed_right = 0;
      }
      return;
    }

    // waypoints[current_position + 3] POSIZIONE che si PUNTA (per evitare errori, non troppo vicina, quindi +3 posizioni)
    // waypoints[current_position].XY POSIZIONE CORRENTE REALE (da odometria) APPROSSIMATA AL PUNTO della TRAIETTORIA PIU' VICINO
    float delta_x = waypoints[current_position + 3].x - waypoints[current_position].x;
    float delta_y = waypoints[current_position + 3].y - waypoints[current_position].y;
    float desired_theta = atan2(delta_y, delta_x);
    float delta_theta_c = desired_theta - pose[2]; // errore dell'angolo

    // NORMALIZZAZIONE delta_theta_c in [-π, π]
    while (delta_theta_c > M_PI) { delta_theta_c -= 2 * M_PI; }
    while (delta_theta_c < -M_PI) { delta_theta_c += 2 * M_PI; }

    os_mutex_lock(speed_mutex);
    DeferLoop(os_mutex_unlock(speed_mutex)) {
      // CONFRONTO con TOLLERANZA dell'(errore dell')ANGOLO
      if (Abs(delta_theta_c) < ANGLE_TOLLERANCE) {
        // ANGOLO (piu' o meno) CORRETTO, si PROCEDE in LINEA RETTA per PUNTARE alla POSIZIONE
        speed_left = TargetSpeed;
        speed_right = TargetSpeed;
      } else {
        // CORREZIONE VELOCITA' per STERZARE
        if(delta_theta_c < 0) {
          // sterzare a DESTRA
          speed_left = TargetSpeed;
          speed_right = TargetSpeed * (1 / Abs((delta_theta_c * 180) / M_PI)); // correzione VELOCITA' PROPORZIONALE all'ERRORE
        } else {
          // sterzare a SINISTRA
          speed_left = TargetSpeed * (1 / Abs((delta_theta_c * 180) / M_PI)); // correzione VELOCITA' PROPORZIONALE all'ERRORE
          speed_right = TargetSpeed;
        }
      }
    }
  }
}

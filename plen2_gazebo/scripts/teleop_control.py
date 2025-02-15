robot_parameters:
  ros__parameters:
    max_joint_velocity: 1.5  # Maximum joint speed
    default_joint_stiffness: 50.0
    default_joint_damping: 0.5

    joint_pids:
      left_shoulder_pitch: {p: 20.0, i: 0.2, d: 0.02}
      right_shoulder_pitch: {p: 20.0, i: 0.2, d: 0.02}
      left_thigh_pitch: {p: 25.0, i: 0.2, d: 0.02}
      right_thigh_pitch: {p: 25.0, i: 0.2, d: 0.02}
      left_ankle_pitch: {p: 10.0, i: 0.2, d: 0.02}
      right_ankle_pitch: {p: 10.0, i: 0.2, d: 0.02}


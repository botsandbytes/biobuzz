// AUTO-GENERATED FILE - DO NOT EDIT DIRECTLY
package org.firstinspires.ftc.teamcode.robot.config.generated;

import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.MatchProfile;

@SuppressWarnings("unused")
public final class config {
  private config() {}

  public static Pedro pedro;
  public static Turret turret;
  public static Vision vision;
  public static Sentinel sentinel;
  public static Casablanca casablanca;
  public static Shooter shooter;
  public static Auto auto;
  public static Teleop teleop;

  public static double PEDRO_LOCALIZATION_X_POD_OFFSET;
  public static double PEDRO_LOCALIZATION_Y_POD_OFFSET;
  public static double PEDRO_HEADING_PID_P;
  public static double PEDRO_HEADING_PID_I;
  public static double PEDRO_HEADING_PID_D;
  public static double PEDRO_HEADING_KS;
  public static double PEDRO_TRANSLATION_GAIN_THRESHOLD;
  public static double PEDRO_TRANSLATION_FORWARD_SMALL_ERROR_PID_P;
  public static double PEDRO_TRANSLATION_FORWARD_SMALL_ERROR_PID_I;
  public static double PEDRO_TRANSLATION_FORWARD_SMALL_ERROR_PID_D;
  public static double PEDRO_TRANSLATION_FORWARD_LARGE_ERROR_PID_P;
  public static double PEDRO_TRANSLATION_FORWARD_LARGE_ERROR_PID_I;
  public static double PEDRO_TRANSLATION_FORWARD_LARGE_ERROR_PID_D;
  public static double PEDRO_TRANSLATION_STRAFE_SMALL_ERROR_PID_P;
  public static double PEDRO_TRANSLATION_STRAFE_SMALL_ERROR_PID_I;
  public static double PEDRO_TRANSLATION_STRAFE_SMALL_ERROR_PID_D;
  public static double PEDRO_TRANSLATION_STRAFE_LARGE_ERROR_PID_P;
  public static double PEDRO_TRANSLATION_STRAFE_LARGE_ERROR_PID_I;
  public static double PEDRO_TRANSLATION_STRAFE_LARGE_ERROR_PID_D;
  public static double PEDRO_FEEDFORWARD_COAST_KV;
  public static double PEDRO_FEEDFORWARD_BRAKE_KV;
  public static double PEDRO_BRAKING_FORWARD_LINEAR;
  public static double PEDRO_BRAKING_FORWARD_QUADRATIC;
  public static double PEDRO_BRAKING_STRAFE_LINEAR;
  public static double PEDRO_BRAKING_STRAFE_QUADRATIC;
  public static double PEDRO_BRAKING_HEADING_LINEAR;
  public static double PEDRO_BRAKING_HEADING_QUADRATIC;
  public static double PEDRO_MOTION_MAX_FORWARD_VELOCITY;
  public static double PEDRO_MOTION_MAX_STRAFE_VELOCITY;
  public static double PEDRO_MOTION_NATURAL_FORWARD_DECELERATION;
  public static double PEDRO_MOTION_NATURAL_STRAFE_DECELERATION;
  public static double PEDRO_MOTION_MAX_PATH_SPEED;
  public static boolean TURRET_ENABLED;
  public static double TURRET_MAX_POWER_OUTPUT;
  public static boolean TURRET_SERVO_DIRECTION_INVERTED;
  public static double TURRET_KS_POSITIVE;
  public static double TURRET_KS_NEGATIVE;
  public static String TURRET_ORIENTATION_LOGO;
  public static String TURRET_ORIENTATION_USB;
  public static double TURRET_TRAVEL_MIN_ANGLE;
  public static double TURRET_TRAVEL_MAX_ANGLE;
  public static boolean TURRET_STALL_ENABLED;
  public static double TURRET_STALL_POWER_THRESHOLD;
  public static double TURRET_STALL_MIN_PROGRESS_DEG;
  public static double TURRET_STALL_TIMEOUT_SEC;
  public static boolean TURRET_RUNAWAY_ENABLED;
  public static double TURRET_RUNAWAY_DIVERGENCE_DEG;
  public static double TURRET_PIDF_P;
  public static double TURRET_PIDF_I;
  public static double TURRET_PIDF_D;
  public static double TURRET_PIDF_F;
  public static double TURRET_TOLERANCE_MAX_DRIFT;
  public static double TURRET_TOLERANCE_NEAR_CUTOFF;
  public static double TURRET_TOLERANCE_NEAR_VAL;
  public static double TURRET_TOLERANCE_MIN_DEG;
  public static double TURRET_TOLERANCE_MAX_DEG;
  public static boolean TURRET_ANALOG_ENCODER_ENABLED;
  public static double TURRET_ANALOG_ENCODER_ZERO_VOLTAGE;
  public static double TURRET_ANALOG_ENCODER_DEGREES_PER_VOLT;
  public static double TURRET_ANALOG_ENCODER_MIN_VOLTAGE;
  public static double TURRET_ANALOG_ENCODER_MAX_VOLTAGE;
  public static double TURRET_ANALOG_ENCODER_FULL_SCALE_VOLTAGE;
  public static boolean TURRET_ANALOG_ENCODER_INVERTED;
  public static double VISION_CAMERA_POSITION_X;
  public static double VISION_CAMERA_POSITION_Y;
  public static double VISION_CAMERA_POSITION_Z;
  public static double VISION_CAMERA_ORIENTATION_YAW;
  public static double VISION_CAMERA_ORIENTATION_PITCH;
  public static double VISION_CAMERA_ORIENTATION_ROLL;
  public static double SENTINEL_ROBOT_WIDTH;
  public static double SENTINEL_ROTATION_LOOKAHEAD_TIME;
  public static double SENTINEL_GOALS_SIZE;
  public static double SENTINEL_GOALS_MIN_Y;
  public static double SENTINEL_GOALS_RED_GOAL_X;
  public static double SENTINEL_GOALS_RED_GOAL_Y;
  public static double SENTINEL_GOALS_BLUE_GOAL_X;
  public static double SENTINEL_GOALS_BLUE_GOAL_Y;
  public static boolean CASABLANCA_ENABLE_DEPTH_PROTECTION;
  public static boolean CASABLANCA_ENABLE_SIDE_PROTECTION;
  public static double CASABLANCA_REPULSION_POWER;
  public static double CASABLANCA_REPULSION_DECEL_SAFETY_FACTOR;
  public static double CASABLANCA_DEPTH_SLOW_DOWN;
  public static double CASABLANCA_DEPTH_HARD_STOP;
  public static double CASABLANCA_SIDE_SLOW_DOWN;
  public static double CASABLANCA_SIDE_HARD_STOP;
  public static double CASABLANCA_LANE_BLEND_DISTANCE;
  public static double CASABLANCA_FRICTION_X;
  public static double CASABLANCA_FRICTION_Y;
  public static double CASABLANCA_FRICTION_ROT;
  public static boolean CASABLANCA_SMOOTHING_TELEOP_ENABLED;
  public static boolean CASABLANCA_SMOOTHING_AUTO_ENABLED;
  public static double CASABLANCA_SMOOTHING_FORWARD_INCREASE_TIME;
  public static double CASABLANCA_SMOOTHING_FORWARD_DECREASE_TIME;
  public static double CASABLANCA_SMOOTHING_STRAFE_INCREASE_TIME;
  public static double CASABLANCA_SMOOTHING_STRAFE_DECREASE_TIME;
  public static double CASABLANCA_SMOOTHING_TURN_INCREASE_TIME;
  public static double CASABLANCA_SMOOTHING_TURN_DECREASE_TIME;
  public static double CASABLANCA_SMOOTHING_MAX_LOOP_DT;
  public static boolean CASABLANCA_HEADING_LOCK_ENABLED;
  public static double CASABLANCA_HEADING_LOCK_PID_P;
  public static double CASABLANCA_HEADING_LOCK_PID_I;
  public static double CASABLANCA_HEADING_LOCK_PID_D;
  public static double CASABLANCA_HEADING_LOCK_KS;
  public static double CASABLANCA_HEADING_LOCK_INTENT_THRESHOLD;
  public static double CASABLANCA_HEADING_LOCK_KS_MOVING;
  public static double CASABLANCA_HEADING_LOCK_MOVING_SPEED_THRESHOLD;
  public static double CASABLANCA_HEADING_LOCK_MAX_POWER;
  public static double CASABLANCA_HEADING_LOCK_ERROR_DEADBAND_DEG;
  public static double CASABLANCA_HEADING_LOCK_SETTLE_RATE_DPS;
  public static Pose CASABLANCA_HEADING_TUNER_CALIBRATION_POSE;
  public static double CASABLANCA_HEADING_TUNER_STEP_DEGREES;
  public static double CASABLANCA_HEADING_TUNER_MAXIMUM_SWEEP_DEGREES;
  public static double CASABLANCA_HEADING_TUNER_POSITION_DRIFT_INCHES;
  public static double CASABLANCA_HEADING_TUNER_HEADING_TOLERANCE_DEGREES;
  public static double CASABLANCA_HEADING_TUNER_RATE_TOLERANCE_DPS;
  public static double CASABLANCA_HEADING_TUNER_OVERSHOOT_LIMIT_DEGREES;
  public static double CASABLANCA_HEADING_TUNER_SETTLE_SEC;
  public static double CASABLANCA_HEADING_TUNER_TRIAL_TIMEOUT_SEC;
  public static double CASABLANCA_HEADING_TUNER_STOPPED_SPEED_IPS;
  public static int CASABLANCA_HEADING_TUNER_SAMPLE_PERIOD_MS;
  public static double CASABLANCA_HEADING_TUNER_MAXIMUM_SAMPLE_GAP_SEC;
  public static double CASABLANCA_HEADING_TUNER_P_MIN;
  public static double CASABLANCA_HEADING_TUNER_P_MAX;
  public static int CASABLANCA_HEADING_TUNER_P_COUNT;
  public static double CASABLANCA_HEADING_TUNER_D_MIN;
  public static double CASABLANCA_HEADING_TUNER_D_MAX;
  public static int CASABLANCA_HEADING_TUNER_D_COUNT;
  public static String CASABLANCA_JUMP_TUNER_TRIAL_AXIS;
  public static double CASABLANCA_JUMP_TUNER_ROTATION_TRIAL_POWER;
  public static double CASABLANCA_JUMP_TUNER_DRIVE_ANGLE_DEGREES;
  public static double CASABLANCA_JUMP_TUNER_MAXIMUM_ANGLE_DEGREES;
  public static double CASABLANCA_JUMP_TUNER_ANGULAR_STOP_MARGIN_DEGREES;
  public static double CASABLANCA_JUMP_TUNER_ANGULAR_SPEED_BOUND_DPS;
  public static String CASABLANCA_JUMP_TUNER_IMU_NAME;
  public static String CASABLANCA_JUMP_TUNER_IMU_LOGO;
  public static String CASABLANCA_JUMP_TUNER_IMU_USB;
  public static Pose CASABLANCA_JUMP_TUNER_CALIBRATION_POSE;
  public static double CASABLANCA_JUMP_TUNER_TRIAL_POWER;
  public static double CASABLANCA_JUMP_TUNER_MAX_TRAVEL_INCHES;
  public static double CASABLANCA_JUMP_TUNER_DRIVE_DISTANCE_INCHES;
  public static double CASABLANCA_JUMP_TUNER_STOP_MARGIN_INCHES;
  public static double CASABLANCA_JUMP_TUNER_REACTION_TIME_SEC;
  public static double CASABLANCA_JUMP_TUNER_SPEED_BOUND_IPS;
  public static double CASABLANCA_JUMP_TUNER_BASELINE_SEC;
  public static double CASABLANCA_JUMP_TUNER_PEAK_HOLD_SEC;
  public static double CASABLANCA_JUMP_TUNER_SETTLE_SEC;
  public static double CASABLANCA_JUMP_TUNER_STOPPED_SPEED_IPS;
  public static double CASABLANCA_JUMP_TUNER_BASELINE_RATE_DPS;
  public static double CASABLANCA_JUMP_TUNER_JUMP_TILT_DEG;
  public static double CASABLANCA_JUMP_TUNER_JUMP_RATE_DPS;
  public static double CASABLANCA_JUMP_TUNER_JUMP_HOLD_SEC;
  public static double CASABLANCA_JUMP_TUNER_EMERGENCY_TILT_DEG;
  public static double CASABLANCA_JUMP_TUNER_IMU_STALE_SEC;
  public static double CASABLANCA_JUMP_TUNER_TRIAL_TIMEOUT_SEC;
  public static double CASABLANCA_JUMP_TUNER_STALL_TIMEOUT_SEC;
  public static double CASABLANCA_JUMP_TUNER_MINIMUM_PROGRESS_INCHES;
  public static double CASABLANCA_JUMP_TUNER_MINIMUM_PROGRESS_DEGREES;
  public static double CASABLANCA_JUMP_TUNER_HEADING_DRIFT_DEG;
  public static double CASABLANCA_JUMP_TUNER_LATERAL_DRIFT_INCHES;
  public static double CASABLANCA_JUMP_TUNER_POWER_TOLERANCE;
  public static int CASABLANCA_JUMP_TUNER_SAMPLE_PERIOD_MS;
  public static double CASABLANCA_JUMP_TUNER_MINIMUM_SLEW_TIME;
  public static double CASABLANCA_JUMP_TUNER_MAXIMUM_SLEW_TIME;
  public static double CASABLANCA_JUMP_TUNER_SEARCH_FACTOR;
  public static int CASABLANCA_JUMP_TUNER_CANDIDATE_COUNT;
  public static boolean SHOOTER_USE_FTC_PID;
  public static double SHOOTER_KS;
  public static double SHOOTER_NOMINAL_VOLTAGE;
  public static double SHOOTER_MAX_VOLTAGE_COMPENSATION;
  public static double SHOOTER_VOLTAGE_REFRESH_SEC;
  public static double SHOOTER_MAX_RPM;
  public static double SHOOTER_CONSTANT_RPM;
  public static double SHOOTER_MIN_TRANSFER_THRESHOLD;
  public static double SHOOTER_MAX_VELOCITY_THRESHOLD;
  public static double SHOOTER_FEED_RELEASE_THRESHOLD;
  public static double SHOOTER_FEED_INTAKE_POWER;
  public static double SHOOTER_FEED_TRANSFER_POWER;
  public static double SHOOTER_LONG_HOOD_POWER_THRESHOLD;
  public static double SHOOTER_BALL_DETECTION_DIP_FRACTION;
  public static double SHOOTER_BALL_DETECTION_REBOUND_FRACTION;
  public static double SHOOTER_BALL_DETECTION_BASELINE_ALPHA;
  public static int SHOOTER_BALL_DETECTION_REFRACTORY_MS;
  public static double SHOOTER_PIDF_P;
  public static double SHOOTER_PIDF_I;
  public static double SHOOTER_PIDF_D;
  public static double SHOOTER_PIDF_F;
  public static double SHOOTER_MOTOR_PIDF_P;
  public static double SHOOTER_MOTOR_PIDF_I;
  public static double SHOOTER_MOTOR_PIDF_D;
  public static double SHOOTER_MOTOR_PIDF_F;
  public static double SHOOTER_INTEGRAL_BAND_TICKS;
  public static double SHOOTER_INTEGRAL_MAX_CONTRIBUTION;
  public static double SHOOTER_BALLISTICS_V0;
  public static double SHOOTER_BALLISTICS_K;
  public static double SHOOTER_BALLISTICS_MAGNUS_L;
  public static double SHOOTER_BALLISTICS_LAUNCH_HEIGHT_INCHES;
  public static double SHOOTER_BALLISTICS_GOAL_HEIGHT_INCHES;
  public static double SHOOTER_BALLISTICS_G_INCHES_PER_SEC2;
  public static double SHOOTER_BALLISTICS_MIN_HOOD_ANGLE_DEG;
  public static double SHOOTER_BALLISTICS_MAX_HOOD_ANGLE_DEG;
  public static double SHOOTER_BALLISTICS_MIN_HOOD_SERVO_POS;
  public static double SHOOTER_BALLISTICS_MAX_HOOD_SERVO_POS;
  public static double SHOOTER_BALLISTICS_MAX_MOVING_SPEED_IPS;
  public static double SHOOTER_BALLISTICS_MIN_VALID_DISTANCE;
  public static double SHOOTER_BALLISTICS_MAX_VALID_DISTANCE;
  public static double SHOOTER_BALLISTICS_LEAD_BIAS_GAIN;
  public static double SHOOTER_BALLISTICS_V0_REFERENCE_RPM;
  public static double SHOOTER_BALLISTICS_PREFERRED_SHOT_RPM;
  public static double SHOOTER_BALLISTICS_MAX_SHOT_RPM;
  public static double SHOOTER_BALLISTICS_V0_MARGIN_FRACTION;
  public static double SHOOTER_BALLISTICS_RPM_UPDATE_DEADBAND;
  public static double SHOOTER_BALLISTICS_FLIGHT_TIME_SEC_PER_INCH;
  public static double SHOOTER_BALLISTICS_LEAD_MIN_SPEED_IPS;
  public static double[] SHOOTER_SHOT_TABLE_POINTS;
  public static double[] AUTO_SHOT_TIME_POINTS;
  public static int AUTO_BALLS_PER_SHOT_COUNT;
  public static int AUTO_DRINK_WAIT_MS;
  public static int AUTO_SHOOT_WAIT_MS;
  public static double AUTO_LAUNCH_POWER;
  public static double AUTO_TRANSFER_POWER;
  public static int AUTO_OPPOSITE_DRINK_WAIT_MS;
  public static int AUTO_OPPOSITE_SHOOT_WAIT_MS;
  public static double AUTO_OPPOSITE_LAUNCH_POWER;
  public static double AUTO_OPPOSITE_TRANSFER_POWER;
  public static int AUTO_ENDGAME_PERIOD_MS;
  public static int AUTO_ENDGAME_EVACUATE_MS;
  public static int AUTO_ENDGAME_FREEZE_MS;
  public static double AUTO_ENDGAME_EXIT_CLEARANCE;
  public static double TELEOP_MAX_SPEED;
  public static boolean TELEOP_FIELD_CENTRIC;
  public static double TELEOP_FIELD_CENTRIC_OFFSET_DEG;
  public static double TELEOP_INTAKE_POWER;
  public static double TELEOP_TRANSFER_POWER;
  public static double TELEOP_MANUAL_REV_POWER;
  public static Pose TELEOP_POSES_RED_START;
  public static Pose TELEOP_POSES_RED_SCORE;
  public static Pose TELEOP_POSES_RED_DRINK;
  public static Pose TELEOP_POSES_RED_PARK;
  public static Pose TELEOP_POSES_BLUE_START;
  public static Pose TELEOP_POSES_BLUE_SCORE;
  public static Pose TELEOP_POSES_BLUE_DRINK;
  public static Pose TELEOP_POSES_BLUE_PARK;

  static {
    reload();
  }

  public static synchronized void reload() {
    ConfigLoader.reload();
    pedro = ConfigLoader.load(Pedro.class, "pedro");
    turret = ConfigLoader.load(Turret.class, "turret");
    vision = ConfigLoader.load(Vision.class, "vision");
    sentinel = ConfigLoader.load(Sentinel.class, "sentinel");
    casablanca = ConfigLoader.load(Casablanca.class, "casablanca");
    shooter = ConfigLoader.load(Shooter.class, "shooter");
    auto = ConfigLoader.load(Auto.class, "auto");
    teleop = ConfigLoader.load(Teleop.class, "teleop");
    PEDRO_LOCALIZATION_X_POD_OFFSET = pedro.localization.x_pod_offset;
    PEDRO_LOCALIZATION_Y_POD_OFFSET = pedro.localization.y_pod_offset;
    PEDRO_HEADING_PID_P = pedro.heading.pid.p;
    PEDRO_HEADING_PID_I = pedro.heading.pid.i;
    PEDRO_HEADING_PID_D = pedro.heading.pid.d;
    PEDRO_HEADING_KS = pedro.heading.ks;
    PEDRO_TRANSLATION_GAIN_THRESHOLD = pedro.translation.gain_threshold;
    PEDRO_TRANSLATION_FORWARD_SMALL_ERROR_PID_P = pedro.translation.forward.small_error_pid.p;
    PEDRO_TRANSLATION_FORWARD_SMALL_ERROR_PID_I = pedro.translation.forward.small_error_pid.i;
    PEDRO_TRANSLATION_FORWARD_SMALL_ERROR_PID_D = pedro.translation.forward.small_error_pid.d;
    PEDRO_TRANSLATION_FORWARD_LARGE_ERROR_PID_P = pedro.translation.forward.large_error_pid.p;
    PEDRO_TRANSLATION_FORWARD_LARGE_ERROR_PID_I = pedro.translation.forward.large_error_pid.i;
    PEDRO_TRANSLATION_FORWARD_LARGE_ERROR_PID_D = pedro.translation.forward.large_error_pid.d;
    PEDRO_TRANSLATION_STRAFE_SMALL_ERROR_PID_P = pedro.translation.strafe.small_error_pid.p;
    PEDRO_TRANSLATION_STRAFE_SMALL_ERROR_PID_I = pedro.translation.strafe.small_error_pid.i;
    PEDRO_TRANSLATION_STRAFE_SMALL_ERROR_PID_D = pedro.translation.strafe.small_error_pid.d;
    PEDRO_TRANSLATION_STRAFE_LARGE_ERROR_PID_P = pedro.translation.strafe.large_error_pid.p;
    PEDRO_TRANSLATION_STRAFE_LARGE_ERROR_PID_I = pedro.translation.strafe.large_error_pid.i;
    PEDRO_TRANSLATION_STRAFE_LARGE_ERROR_PID_D = pedro.translation.strafe.large_error_pid.d;
    PEDRO_FEEDFORWARD_COAST_KV = pedro.feedforward.coast_kv;
    PEDRO_FEEDFORWARD_BRAKE_KV = pedro.feedforward.brake_kv;
    PEDRO_BRAKING_FORWARD_LINEAR = pedro.braking.forward.linear;
    PEDRO_BRAKING_FORWARD_QUADRATIC = pedro.braking.forward.quadratic;
    PEDRO_BRAKING_STRAFE_LINEAR = pedro.braking.strafe.linear;
    PEDRO_BRAKING_STRAFE_QUADRATIC = pedro.braking.strafe.quadratic;
    PEDRO_BRAKING_HEADING_LINEAR = pedro.braking.heading.linear;
    PEDRO_BRAKING_HEADING_QUADRATIC = pedro.braking.heading.quadratic;
    PEDRO_MOTION_MAX_FORWARD_VELOCITY = pedro.motion.max_forward_velocity;
    PEDRO_MOTION_MAX_STRAFE_VELOCITY = pedro.motion.max_strafe_velocity;
    PEDRO_MOTION_NATURAL_FORWARD_DECELERATION = pedro.motion.natural_forward_deceleration;
    PEDRO_MOTION_NATURAL_STRAFE_DECELERATION = pedro.motion.natural_strafe_deceleration;
    PEDRO_MOTION_MAX_PATH_SPEED = pedro.motion.max_path_speed;
    TURRET_ENABLED = turret.enabled;
    TURRET_MAX_POWER_OUTPUT = turret.max_power_output;
    TURRET_SERVO_DIRECTION_INVERTED = turret.servo_direction_inverted;
    TURRET_KS_POSITIVE = turret.ks_positive;
    TURRET_KS_NEGATIVE = turret.ks_negative;
    TURRET_ORIENTATION_LOGO = turret.orientation.logo;
    TURRET_ORIENTATION_USB = turret.orientation.usb;
    TURRET_TRAVEL_MIN_ANGLE = turret.travel.min_angle;
    TURRET_TRAVEL_MAX_ANGLE = turret.travel.max_angle;
    TURRET_STALL_ENABLED = turret.stall.enabled;
    TURRET_STALL_POWER_THRESHOLD = turret.stall.power_threshold;
    TURRET_STALL_MIN_PROGRESS_DEG = turret.stall.min_progress_deg;
    TURRET_STALL_TIMEOUT_SEC = turret.stall.timeout_sec;
    TURRET_RUNAWAY_ENABLED = turret.runaway.enabled;
    TURRET_RUNAWAY_DIVERGENCE_DEG = turret.runaway.divergence_deg;
    TURRET_PIDF_P = turret.pidf.p;
    TURRET_PIDF_I = turret.pidf.i;
    TURRET_PIDF_D = turret.pidf.d;
    TURRET_PIDF_F = turret.pidf.f;
    TURRET_TOLERANCE_MAX_DRIFT = turret.tolerance.max_drift;
    TURRET_TOLERANCE_NEAR_CUTOFF = turret.tolerance.near_cutoff;
    TURRET_TOLERANCE_NEAR_VAL = turret.tolerance.near_val;
    TURRET_TOLERANCE_MIN_DEG = turret.tolerance.min_deg;
    TURRET_TOLERANCE_MAX_DEG = turret.tolerance.max_deg;
    TURRET_ANALOG_ENCODER_ENABLED = turret.analog_encoder.enabled;
    TURRET_ANALOG_ENCODER_ZERO_VOLTAGE = turret.analog_encoder.zero_voltage;
    TURRET_ANALOG_ENCODER_DEGREES_PER_VOLT = turret.analog_encoder.degrees_per_volt;
    TURRET_ANALOG_ENCODER_MIN_VOLTAGE = turret.analog_encoder.min_voltage;
    TURRET_ANALOG_ENCODER_MAX_VOLTAGE = turret.analog_encoder.max_voltage;
    TURRET_ANALOG_ENCODER_FULL_SCALE_VOLTAGE = turret.analog_encoder.full_scale_voltage;
    TURRET_ANALOG_ENCODER_INVERTED = turret.analog_encoder.inverted;
    VISION_CAMERA_POSITION_X = vision.camera_position.x;
    VISION_CAMERA_POSITION_Y = vision.camera_position.y;
    VISION_CAMERA_POSITION_Z = vision.camera_position.z;
    VISION_CAMERA_ORIENTATION_YAW = vision.camera_orientation.yaw;
    VISION_CAMERA_ORIENTATION_PITCH = vision.camera_orientation.pitch;
    VISION_CAMERA_ORIENTATION_ROLL = vision.camera_orientation.roll;
    SENTINEL_ROBOT_WIDTH = sentinel.robot_width;
    SENTINEL_ROTATION_LOOKAHEAD_TIME = sentinel.rotation_lookahead_time;
    SENTINEL_GOALS_SIZE = sentinel.goals.size;
    SENTINEL_GOALS_MIN_Y = sentinel.goals.min_y;
    SENTINEL_GOALS_RED_GOAL_X = sentinel.goals.red_goal_x;
    SENTINEL_GOALS_RED_GOAL_Y = sentinel.goals.red_goal_y;
    SENTINEL_GOALS_BLUE_GOAL_X = sentinel.goals.blue_goal_x;
    SENTINEL_GOALS_BLUE_GOAL_Y = sentinel.goals.blue_goal_y;
    CASABLANCA_ENABLE_DEPTH_PROTECTION = casablanca.enable_depth_protection;
    CASABLANCA_ENABLE_SIDE_PROTECTION = casablanca.enable_side_protection;
    CASABLANCA_REPULSION_POWER = casablanca.repulsion.power;
    CASABLANCA_REPULSION_DECEL_SAFETY_FACTOR = casablanca.repulsion.decel_safety_factor;
    CASABLANCA_DEPTH_SLOW_DOWN = casablanca.depth.slow_down;
    CASABLANCA_DEPTH_HARD_STOP = casablanca.depth.hard_stop;
    CASABLANCA_SIDE_SLOW_DOWN = casablanca.side.slow_down;
    CASABLANCA_SIDE_HARD_STOP = casablanca.side.hard_stop;
    CASABLANCA_LANE_BLEND_DISTANCE = casablanca.lane_blend_distance;
    CASABLANCA_FRICTION_X = casablanca.friction.x;
    CASABLANCA_FRICTION_Y = casablanca.friction.y;
    CASABLANCA_FRICTION_ROT = casablanca.friction.rot;
    CASABLANCA_SMOOTHING_TELEOP_ENABLED = casablanca.smoothing.teleop_enabled;
    CASABLANCA_SMOOTHING_AUTO_ENABLED = casablanca.smoothing.auto_enabled;
    CASABLANCA_SMOOTHING_FORWARD_INCREASE_TIME = casablanca.smoothing.forward.increase_time;
    CASABLANCA_SMOOTHING_FORWARD_DECREASE_TIME = casablanca.smoothing.forward.decrease_time;
    CASABLANCA_SMOOTHING_STRAFE_INCREASE_TIME = casablanca.smoothing.strafe.increase_time;
    CASABLANCA_SMOOTHING_STRAFE_DECREASE_TIME = casablanca.smoothing.strafe.decrease_time;
    CASABLANCA_SMOOTHING_TURN_INCREASE_TIME = casablanca.smoothing.turn.increase_time;
    CASABLANCA_SMOOTHING_TURN_DECREASE_TIME = casablanca.smoothing.turn.decrease_time;
    CASABLANCA_SMOOTHING_MAX_LOOP_DT = casablanca.smoothing.max_loop_dt;
    CASABLANCA_HEADING_LOCK_ENABLED = casablanca.heading_lock.enabled;
    CASABLANCA_HEADING_LOCK_PID_P = casablanca.heading_lock.pid.p;
    CASABLANCA_HEADING_LOCK_PID_I = casablanca.heading_lock.pid.i;
    CASABLANCA_HEADING_LOCK_PID_D = casablanca.heading_lock.pid.d;
    CASABLANCA_HEADING_LOCK_KS = casablanca.heading_lock.ks;
    CASABLANCA_HEADING_LOCK_INTENT_THRESHOLD = casablanca.heading_lock.intent_threshold;
    CASABLANCA_HEADING_LOCK_KS_MOVING = casablanca.heading_lock.ks_moving;
    CASABLANCA_HEADING_LOCK_MOVING_SPEED_THRESHOLD = casablanca.heading_lock.moving_speed_threshold;
    CASABLANCA_HEADING_LOCK_MAX_POWER = casablanca.heading_lock.max_power;
    CASABLANCA_HEADING_LOCK_ERROR_DEADBAND_DEG = casablanca.heading_lock.error_deadband_deg;
    CASABLANCA_HEADING_LOCK_SETTLE_RATE_DPS = casablanca.heading_lock.settle_rate_dps;
    CASABLANCA_HEADING_TUNER_CALIBRATION_POSE = casablanca.heading_tuner.calibration_pose;
    CASABLANCA_HEADING_TUNER_STEP_DEGREES = casablanca.heading_tuner.step_degrees;
    CASABLANCA_HEADING_TUNER_MAXIMUM_SWEEP_DEGREES = casablanca.heading_tuner.maximum_sweep_degrees;
    CASABLANCA_HEADING_TUNER_POSITION_DRIFT_INCHES = casablanca.heading_tuner.position_drift_inches;
    CASABLANCA_HEADING_TUNER_HEADING_TOLERANCE_DEGREES =
        casablanca.heading_tuner.heading_tolerance_degrees;
    CASABLANCA_HEADING_TUNER_RATE_TOLERANCE_DPS = casablanca.heading_tuner.rate_tolerance_dps;
    CASABLANCA_HEADING_TUNER_OVERSHOOT_LIMIT_DEGREES =
        casablanca.heading_tuner.overshoot_limit_degrees;
    CASABLANCA_HEADING_TUNER_SETTLE_SEC = casablanca.heading_tuner.settle_sec;
    CASABLANCA_HEADING_TUNER_TRIAL_TIMEOUT_SEC = casablanca.heading_tuner.trial_timeout_sec;
    CASABLANCA_HEADING_TUNER_STOPPED_SPEED_IPS = casablanca.heading_tuner.stopped_speed_ips;
    CASABLANCA_HEADING_TUNER_SAMPLE_PERIOD_MS = casablanca.heading_tuner.sample_period_ms;
    CASABLANCA_HEADING_TUNER_MAXIMUM_SAMPLE_GAP_SEC =
        casablanca.heading_tuner.maximum_sample_gap_sec;
    CASABLANCA_HEADING_TUNER_P_MIN = casablanca.heading_tuner.p_min;
    CASABLANCA_HEADING_TUNER_P_MAX = casablanca.heading_tuner.p_max;
    CASABLANCA_HEADING_TUNER_P_COUNT = casablanca.heading_tuner.p_count;
    CASABLANCA_HEADING_TUNER_D_MIN = casablanca.heading_tuner.d_min;
    CASABLANCA_HEADING_TUNER_D_MAX = casablanca.heading_tuner.d_max;
    CASABLANCA_HEADING_TUNER_D_COUNT = casablanca.heading_tuner.d_count;
    CASABLANCA_JUMP_TUNER_TRIAL_AXIS = casablanca.jump_tuner.trial_axis;
    CASABLANCA_JUMP_TUNER_ROTATION_TRIAL_POWER = casablanca.jump_tuner.rotation_trial_power;
    CASABLANCA_JUMP_TUNER_DRIVE_ANGLE_DEGREES = casablanca.jump_tuner.drive_angle_degrees;
    CASABLANCA_JUMP_TUNER_MAXIMUM_ANGLE_DEGREES = casablanca.jump_tuner.maximum_angle_degrees;
    CASABLANCA_JUMP_TUNER_ANGULAR_STOP_MARGIN_DEGREES =
        casablanca.jump_tuner.angular_stop_margin_degrees;
    CASABLANCA_JUMP_TUNER_ANGULAR_SPEED_BOUND_DPS = casablanca.jump_tuner.angular_speed_bound_dps;
    CASABLANCA_JUMP_TUNER_IMU_NAME = casablanca.jump_tuner.imu_name;
    CASABLANCA_JUMP_TUNER_IMU_LOGO = casablanca.jump_tuner.imu_logo;
    CASABLANCA_JUMP_TUNER_IMU_USB = casablanca.jump_tuner.imu_usb;
    CASABLANCA_JUMP_TUNER_CALIBRATION_POSE = casablanca.jump_tuner.calibration_pose;
    CASABLANCA_JUMP_TUNER_TRIAL_POWER = casablanca.jump_tuner.trial_power;
    CASABLANCA_JUMP_TUNER_MAX_TRAVEL_INCHES = casablanca.jump_tuner.max_travel_inches;
    CASABLANCA_JUMP_TUNER_DRIVE_DISTANCE_INCHES = casablanca.jump_tuner.drive_distance_inches;
    CASABLANCA_JUMP_TUNER_STOP_MARGIN_INCHES = casablanca.jump_tuner.stop_margin_inches;
    CASABLANCA_JUMP_TUNER_REACTION_TIME_SEC = casablanca.jump_tuner.reaction_time_sec;
    CASABLANCA_JUMP_TUNER_SPEED_BOUND_IPS = casablanca.jump_tuner.speed_bound_ips;
    CASABLANCA_JUMP_TUNER_BASELINE_SEC = casablanca.jump_tuner.baseline_sec;
    CASABLANCA_JUMP_TUNER_PEAK_HOLD_SEC = casablanca.jump_tuner.peak_hold_sec;
    CASABLANCA_JUMP_TUNER_SETTLE_SEC = casablanca.jump_tuner.settle_sec;
    CASABLANCA_JUMP_TUNER_STOPPED_SPEED_IPS = casablanca.jump_tuner.stopped_speed_ips;
    CASABLANCA_JUMP_TUNER_BASELINE_RATE_DPS = casablanca.jump_tuner.baseline_rate_dps;
    CASABLANCA_JUMP_TUNER_JUMP_TILT_DEG = casablanca.jump_tuner.jump_tilt_deg;
    CASABLANCA_JUMP_TUNER_JUMP_RATE_DPS = casablanca.jump_tuner.jump_rate_dps;
    CASABLANCA_JUMP_TUNER_JUMP_HOLD_SEC = casablanca.jump_tuner.jump_hold_sec;
    CASABLANCA_JUMP_TUNER_EMERGENCY_TILT_DEG = casablanca.jump_tuner.emergency_tilt_deg;
    CASABLANCA_JUMP_TUNER_IMU_STALE_SEC = casablanca.jump_tuner.imu_stale_sec;
    CASABLANCA_JUMP_TUNER_TRIAL_TIMEOUT_SEC = casablanca.jump_tuner.trial_timeout_sec;
    CASABLANCA_JUMP_TUNER_STALL_TIMEOUT_SEC = casablanca.jump_tuner.stall_timeout_sec;
    CASABLANCA_JUMP_TUNER_MINIMUM_PROGRESS_INCHES = casablanca.jump_tuner.minimum_progress_inches;
    CASABLANCA_JUMP_TUNER_MINIMUM_PROGRESS_DEGREES = casablanca.jump_tuner.minimum_progress_degrees;
    CASABLANCA_JUMP_TUNER_HEADING_DRIFT_DEG = casablanca.jump_tuner.heading_drift_deg;
    CASABLANCA_JUMP_TUNER_LATERAL_DRIFT_INCHES = casablanca.jump_tuner.lateral_drift_inches;
    CASABLANCA_JUMP_TUNER_POWER_TOLERANCE = casablanca.jump_tuner.power_tolerance;
    CASABLANCA_JUMP_TUNER_SAMPLE_PERIOD_MS = casablanca.jump_tuner.sample_period_ms;
    CASABLANCA_JUMP_TUNER_MINIMUM_SLEW_TIME = casablanca.jump_tuner.minimum_slew_time;
    CASABLANCA_JUMP_TUNER_MAXIMUM_SLEW_TIME = casablanca.jump_tuner.maximum_slew_time;
    CASABLANCA_JUMP_TUNER_SEARCH_FACTOR = casablanca.jump_tuner.search_factor;
    CASABLANCA_JUMP_TUNER_CANDIDATE_COUNT = casablanca.jump_tuner.candidate_count;
    SHOOTER_USE_FTC_PID = shooter.use_ftc_pid;
    SHOOTER_KS = shooter.ks;
    SHOOTER_NOMINAL_VOLTAGE = shooter.nominal_voltage;
    SHOOTER_MAX_VOLTAGE_COMPENSATION = shooter.max_voltage_compensation;
    SHOOTER_VOLTAGE_REFRESH_SEC = shooter.voltage_refresh_sec;
    SHOOTER_MAX_RPM = shooter.max_rpm;
    SHOOTER_CONSTANT_RPM = shooter.constant_rpm;
    SHOOTER_MIN_TRANSFER_THRESHOLD = shooter.min_transfer_threshold;
    SHOOTER_MAX_VELOCITY_THRESHOLD = shooter.max_velocity_threshold;
    SHOOTER_FEED_RELEASE_THRESHOLD = shooter.feed_release_threshold;
    SHOOTER_FEED_INTAKE_POWER = shooter.feed_intake_power;
    SHOOTER_FEED_TRANSFER_POWER = shooter.feed_transfer_power;
    SHOOTER_LONG_HOOD_POWER_THRESHOLD = shooter.long_hood_power_threshold;
    SHOOTER_BALL_DETECTION_DIP_FRACTION = shooter.ball_detection.dip_fraction;
    SHOOTER_BALL_DETECTION_REBOUND_FRACTION = shooter.ball_detection.rebound_fraction;
    SHOOTER_BALL_DETECTION_BASELINE_ALPHA = shooter.ball_detection.baseline_alpha;
    SHOOTER_BALL_DETECTION_REFRACTORY_MS = shooter.ball_detection.refractory_ms;
    SHOOTER_PIDF_P = shooter.pidf.p;
    SHOOTER_PIDF_I = shooter.pidf.i;
    SHOOTER_PIDF_D = shooter.pidf.d;
    SHOOTER_PIDF_F = shooter.pidf.f;
    SHOOTER_MOTOR_PIDF_P = shooter.motor_pidf.p;
    SHOOTER_MOTOR_PIDF_I = shooter.motor_pidf.i;
    SHOOTER_MOTOR_PIDF_D = shooter.motor_pidf.d;
    SHOOTER_MOTOR_PIDF_F = shooter.motor_pidf.f;
    SHOOTER_INTEGRAL_BAND_TICKS = shooter.integral.band_ticks;
    SHOOTER_INTEGRAL_MAX_CONTRIBUTION = shooter.integral.max_contribution;
    SHOOTER_BALLISTICS_V0 = shooter.ballistics.v0;
    SHOOTER_BALLISTICS_K = shooter.ballistics.k;
    SHOOTER_BALLISTICS_MAGNUS_L = shooter.ballistics.magnus_l;
    SHOOTER_BALLISTICS_LAUNCH_HEIGHT_INCHES = shooter.ballistics.launch_height_inches;
    SHOOTER_BALLISTICS_GOAL_HEIGHT_INCHES = shooter.ballistics.goal_height_inches;
    SHOOTER_BALLISTICS_G_INCHES_PER_SEC2 = shooter.ballistics.g_inches_per_sec2;
    SHOOTER_BALLISTICS_MIN_HOOD_ANGLE_DEG = shooter.ballistics.min_hood_angle_deg;
    SHOOTER_BALLISTICS_MAX_HOOD_ANGLE_DEG = shooter.ballistics.max_hood_angle_deg;
    SHOOTER_BALLISTICS_MIN_HOOD_SERVO_POS = shooter.ballistics.min_hood_servo_pos;
    SHOOTER_BALLISTICS_MAX_HOOD_SERVO_POS = shooter.ballistics.max_hood_servo_pos;
    SHOOTER_BALLISTICS_MAX_MOVING_SPEED_IPS = shooter.ballistics.max_moving_speed_ips;
    SHOOTER_BALLISTICS_MIN_VALID_DISTANCE = shooter.ballistics.min_valid_distance;
    SHOOTER_BALLISTICS_MAX_VALID_DISTANCE = shooter.ballistics.max_valid_distance;
    SHOOTER_BALLISTICS_LEAD_BIAS_GAIN = shooter.ballistics.lead_bias_gain;
    SHOOTER_BALLISTICS_V0_REFERENCE_RPM = shooter.ballistics.v0_reference_rpm;
    SHOOTER_BALLISTICS_PREFERRED_SHOT_RPM = shooter.ballistics.preferred_shot_rpm;
    SHOOTER_BALLISTICS_MAX_SHOT_RPM = shooter.ballistics.max_shot_rpm;
    SHOOTER_BALLISTICS_V0_MARGIN_FRACTION = shooter.ballistics.v0_margin_fraction;
    SHOOTER_BALLISTICS_RPM_UPDATE_DEADBAND = shooter.ballistics.rpm_update_deadband;
    SHOOTER_BALLISTICS_FLIGHT_TIME_SEC_PER_INCH = shooter.ballistics.flight_time_sec_per_inch;
    SHOOTER_BALLISTICS_LEAD_MIN_SPEED_IPS = shooter.ballistics.lead_min_speed_ips;
    SHOOTER_SHOT_TABLE_POINTS = shooter.shot_table.points;
    AUTO_SHOT_TIME_POINTS = auto.shot_time_points;
    AUTO_BALLS_PER_SHOT_COUNT = auto.balls_per_shot_count;
    AUTO_DRINK_WAIT_MS = auto.drink_wait_ms;
    AUTO_SHOOT_WAIT_MS = auto.shoot_wait_ms;
    AUTO_LAUNCH_POWER = auto.launch_power;
    AUTO_TRANSFER_POWER = auto.transfer_power;
    AUTO_OPPOSITE_DRINK_WAIT_MS = auto.opposite_drink_wait_ms;
    AUTO_OPPOSITE_SHOOT_WAIT_MS = auto.opposite_shoot_wait_ms;
    AUTO_OPPOSITE_LAUNCH_POWER = auto.opposite_launch_power;
    AUTO_OPPOSITE_TRANSFER_POWER = auto.opposite_transfer_power;
    AUTO_ENDGAME_PERIOD_MS = auto.endgame.period_ms;
    AUTO_ENDGAME_EVACUATE_MS = auto.endgame.evacuate_ms;
    AUTO_ENDGAME_FREEZE_MS = auto.endgame.freeze_ms;
    AUTO_ENDGAME_EXIT_CLEARANCE = auto.endgame.exit_clearance;
    TELEOP_MAX_SPEED = teleop.max_speed;
    TELEOP_FIELD_CENTRIC = teleop.field_centric;
    TELEOP_FIELD_CENTRIC_OFFSET_DEG = teleop.field_centric_offset_deg;
    TELEOP_INTAKE_POWER = teleop.intake_power;
    TELEOP_TRANSFER_POWER = teleop.transfer_power;
    TELEOP_MANUAL_REV_POWER = teleop.manual_rev_power;
    TELEOP_POSES_RED_START = teleop.poses.red.start;
    TELEOP_POSES_RED_SCORE = teleop.poses.red.score;
    TELEOP_POSES_RED_DRINK = teleop.poses.red.drink;
    TELEOP_POSES_RED_PARK = teleop.poses.red.park;
    TELEOP_POSES_BLUE_START = teleop.poses.blue.start;
    TELEOP_POSES_BLUE_SCORE = teleop.poses.blue.score;
    TELEOP_POSES_BLUE_DRINK = teleop.poses.blue.drink;
    TELEOP_POSES_BLUE_PARK = teleop.poses.blue.park;
  }

  public static final class Pedro {
    public static final class Localization {
      /** Forward odometry pod offset in inches, recovered from local Pedro 3 tuning. */
      public double x_pod_offset;

      /** Strafe odometry pod offset in inches, recovered from local Pedro 3 tuning. */
      public double y_pod_offset;
    }

    public Localization localization;

    public static final class Heading {
      /**
       * Heading feedback PID coefficients, bound as SDK PIDFCoefficients. Current Pedro 3 tuning is
       * proportional-only: keep I and D at zero unless separately tuned.
       */
      public com.qualcomm.robotcore.hardware.PIDFCoefficients pid;

      /**
       * Static friction feedforward magnitude. Pedro heading uses zero with the recovered
       * proportional-only tuning; separate from PID feedback. Minimum: 0.0 Maximum: 1.0
       */
      public double ks;
    }

    public Heading heading;

    public static final class Translation {
      /**
       * Absolute translation error in inches at which each axis switches to its large-error PID
       * gains. Minimum: 0.0
       */
      public double gain_threshold;

      public static final class Forward {
        /**
         * Translation PID coefficients below gain_threshold, bound as SDK PIDFCoefficients.
         * Recovered Pedro 3 tuning uses only P; I and D are zero.
         */
        public com.qualcomm.robotcore.hardware.PIDFCoefficients small_error_pid;

        /**
         * Translation PID coefficients at or above gain_threshold, bound as SDK PIDFCoefficients.
         * Recovered Pedro 3 tuning uses only P; I and D are zero.
         */
        public com.qualcomm.robotcore.hardware.PIDFCoefficients large_error_pid;
      }

      public Forward forward;

      public static final class Strafe {
        /**
         * Translation PID coefficients below gain_threshold, bound as SDK PIDFCoefficients.
         * Recovered Pedro 3 tuning uses only P; I and D are zero.
         */
        public com.qualcomm.robotcore.hardware.PIDFCoefficients small_error_pid;

        /**
         * Translation PID coefficients at or above gain_threshold, bound as SDK PIDFCoefficients.
         * Recovered Pedro 3 tuning uses only P; I and D are zero.
         */
        public com.qualcomm.robotcore.hardware.PIDFCoefficients large_error_pid;
      }

      public Strafe strafe;
    }

    public Translation translation;

    public static final class Feedforward {
      /** Foresight coast feedforward gain recovered from local Pedro 3 tuning. Minimum: 0.0 */
      public double coast_kv;

      /** Foresight brake feedforward gain recovered from local Pedro 3 tuning. Minimum: 0.0 */
      public double brake_kv;
    }

    public Feedforward feedforward;

    public static final class Braking {
      public static final class Forward {
        /**
         * Linear coefficient in Foresight stopping displacement: linear * velocity + quadratic *
         * velocity * abs(velocity). Translation uses inches and heading uses radians.
         */
        public double linear;

        /**
         * Quadratic coefficient in Foresight stopping displacement: linear * velocity + quadratic *
         * velocity * abs(velocity). Translation uses inches and heading uses radians.
         */
        public double quadratic;
      }

      public Forward forward;

      public static final class Strafe {
        /**
         * Linear coefficient in Foresight stopping displacement: linear * velocity + quadratic *
         * velocity * abs(velocity). Translation uses inches and heading uses radians.
         */
        public double linear;

        /**
         * Quadratic coefficient in Foresight stopping displacement: linear * velocity + quadratic *
         * velocity * abs(velocity). Translation uses inches and heading uses radians.
         */
        public double quadratic;
      }

      public Strafe strafe;

      public static final class Heading {
        /**
         * Linear coefficient in Foresight stopping displacement: linear * velocity + quadratic *
         * velocity * abs(velocity). Translation uses inches and heading uses radians.
         */
        public double linear;

        /**
         * Quadratic coefficient in Foresight stopping displacement: linear * velocity + quadratic *
         * velocity * abs(velocity). Translation uses inches and heading uses radians.
         */
        public double quadratic;
      }

      public Heading heading;
    }

    public Braking braking;

    public static final class Motion {
      /**
       * Forward velocity in inches per second recovered verbatim from local tuning; unusually low,
       * verify on robot. Minimum: 0.0
       */
      public double max_forward_velocity;

      /** Strafe velocity in inches per second recovered from local Pedro 3 tuning. Minimum: 0.0 */
      public double max_strafe_velocity;

      /**
       * Foresight natural forward deceleration in inches per second squared, recovered from local
       * Pedro 3 tuning. Minimum: 0.0
       */
      public double natural_forward_deceleration;

      /**
       * Foresight natural strafe deceleration in inches per second squared, recovered from local
       * Pedro 3 tuning. Minimum: 0.0
       */
      public double natural_strafe_deceleration;

      /**
       * Maximum Foresight path speed as a fraction of measured top speed. Explicit speed policy,
       * not equivalent to the old v2 path-completion threshold. Minimum: 0.0 Maximum: 1.0
       */
      public double max_path_speed;
    }

    public Motion motion;
  }

  public static final class Turret {
    /** Enable/disable this subsystem, controller, watchdog, or feature. */
    public boolean enabled;

    /**
     * Maximum power limit (0.0 to 1.0) commanded to the turret CRServo. Limits maximum turret
     * rotation speed. Minimum: 0.0 Maximum: 1.0
     */
    public double max_power_output;

    /**
     * Flips which CRServo direction counts as increasing turret angle. Set this (not the analog
     * encoder's 'inverted' flag) when the turret drives away from its target.
     */
    public boolean servo_direction_inverted;

    /**
     * Static friction feedforward (kS) magnitude applied when turning turret in positive direction
     * (error > 0). Minimum: 0.0 Maximum: 1.0
     */
    public double ks_positive;

    /**
     * Static friction feedforward (kS) magnitude applied when turning turret in negative direction
     * (error < 0). Minimum: 0.0 Maximum: 1.0
     */
    public double ks_negative;

    public static final class Orientation {
      /**
       * Mounting orientation direction of the hub's logo panel (e.g. RIGHT, LEFT, UP, DOWN,
       * FORWARD, BACKWARD).
       */
      public String logo;

      /** Mounting direction of the hub's USB ports. */
      public String usb;
    }

    public Orientation orientation;

    public static final class Travel {
      /**
       * Most negative turret angle (degrees, relative to the chassis) the turret is allowed to
       * reach. Minimum: -180.0 Maximum: 0.0
       */
      public double min_angle;

      /**
       * Most positive turret angle (degrees, relative to the chassis) the turret is allowed to
       * reach. Minimum: 0.0 Maximum: 180.0
       */
      public double max_angle;
    }

    public Travel travel;

    public static final class Stall {
      /** Enable/disable this subsystem, controller, watchdog, or feature. */
      public boolean enabled;

      /**
       * Commanded turret power above which the stall watchdog starts checking for motion. Below
       * this the servo may legitimately not move, so no stall is inferred. Minimum: 0.0 Maximum:
       * 1.0
       */
      public double power_threshold;

      /**
       * Turret motion (degrees) that counts as progress and resets the stall watchdog. Set above
       * the analog encoder's noise floor. Minimum: 0.0
       */
      public double min_progress_deg;

      /**
       * Seconds of commanded power with no turret motion before the stall watchdog latches a fault
       * and cuts power. Protects the turret from being driven into a hard stop when the encoder
       * reading is wrong. Minimum: 0.0
       */
      public double timeout_sec;
    }

    public Stall stall;

    public static final class Runaway {
      /** Enable/disable this subsystem, controller, watchdog, or feature. */
      public boolean enabled;

      /**
       * How far (degrees) the turret's aim error may grow beyond its best-so-far value before the
       * runaway watchdog cuts power. Catches an inverted control loop, where positive power moves
       * the turret away from its target, before it reaches a hard stop. Minimum: 0.0
       */
      public double divergence_deg;
    }

    public Runaway runaway;
    public com.qualcomm.robotcore.hardware.PIDFCoefficients pidf;

    public static final class Tolerance {
      /**
       * Maximum allowed chassis drift (inches) from target pose before auto-corrective drive
       * motions engage. Minimum: 0.0
       */
      public double max_drift;

      /**
       * Distance boundary (inches) below which the target is considered close, switching to static
       * tolerance. Minimum: 0.0
       */
      public double near_cutoff;

      /** Target alignment tolerance (degrees) when within the near cutoff range. Minimum: 0.0 */
      public double near_val;

      /**
       * Minimum dynamic alignment tolerance (degrees) allowed. Prevents the turret from hunting
       * indefinitely when very close. Minimum: 0.0
       */
      public double min_deg;

      /**
       * Maximum dynamic alignment tolerance (degrees) allowed. Prevents launching if the turret
       * orientation error exceeds this. Minimum: 0.0
       */
      public double max_deg;
    }

    public Tolerance tolerance;

    public static final class AnalogEncoder {
      /** Enable/disable this subsystem, controller, watchdog, or feature. */
      public boolean enabled;

      /**
       * Analog voltage (V) corresponding to 0 degree turret angle position. Minimum: 0.0 Maximum:
       * 3.3
       */
      public double zero_voltage;

      /**
       * Scale factor (degrees/volt) converting analog input voltage delta to turret angle in
       * degrees.
       */
      public double degrees_per_volt;

      /** Minimum safe voltage limit for the analog encoder. Minimum: 0.0 Maximum: 3.3 */
      public double min_voltage;

      /** Maximum safe voltage limit for the analog encoder. Minimum: 0.0 Maximum: 3.3 */
      public double max_voltage;

      /**
       * Full electrical span (volts) of the turret potentiometer, i.e. the voltage at which it
       * wraps back to zero. Minimum: 0.0
       */
      public double full_scale_voltage;

      /** True if increasing turret angle produces decreasing analog voltage. */
      public boolean inverted;
    }

    public AnalogEncoder analog_encoder;
  }

  public static final class Vision {
    public static final class CameraPosition {
      /** X value coordinate. */
      public double x;

      /** Y value coordinate. */
      public double y;

      /** Z value coordinate. */
      public double z;
    }

    public CameraPosition camera_position;

    public static final class CameraOrientation {
      /** Yaw rotation (degrees). */
      public double yaw;

      /** Pitch rotation (degrees). */
      public double pitch;

      /** Roll rotation (degrees). */
      public double roll;
    }

    public CameraOrientation camera_orientation;
  }

  public static final class Sentinel {
    /**
     * Total physical width of the robot (inches) used to build the collision footprint. Determines
     * how close the robot can get to walls or goals. Minimum: 0.0
     */
    public double robot_width;

    /**
     * Time horizon (seconds) used to project future rotation based on live angular velocity
     * (|omega| * time). Seeded at 0.075s (derived from 0.45rad / assumed 6.0rad/s max omega);
     * calibrate on robot using Friction Calibration TeleOp. Minimum: 0.0
     */
    public double rotation_lookahead_time;

    public static final class Goals {
      /** Length (inches) of one side of the square scoring goal zones on the field. Minimum: 0.0 */
      public double size;

      /**
       * The Y-coordinate boundary (inches) on the field where the scoring zones begin. Used to
       * calculate red/blue alliance goal boxes. Minimum: 0.0
       */
      public double min_y;

      /** Goal center X coordinate for RED alliance. Minimum: 0.0 */
      public double red_goal_x;

      /** Goal center Y coordinate for RED alliance. Minimum: 0.0 */
      public double red_goal_y;

      /** Goal center X coordinate for BLUE alliance. Minimum: 0.0 */
      public double blue_goal_x;

      /** Goal center Y coordinate for BLUE alliance. Minimum: 0.0 */
      public double blue_goal_y;
    }

    public Goals goals;
  }

  public static final class Casablanca {
    /**
     * Enable/disable depth (X-axis) goal zone proximity protection. Safety policy parameter; leave
     * true outside of debugging.
     */
    public boolean enable_depth_protection;

    /**
     * Enable/disable side (Y-axis) goal zone proximity protection. Safety policy parameter; leave
     * true outside of debugging.
     */
    public boolean enable_side_protection;

    public static final class Repulsion {
      /**
       * Motor power (0.0 to 1.0) commanded to push the robot away when it violates a goal zone
       * boundary. Driver/safety policy parameter. Minimum: 0.0 Maximum: 1.0
       */
      public double power;

      /**
       * Safety margin factor (0.0 to 1.0) for predictive braking (e.g. 0.7 = 30% safety margin over
       * physics model prediction). Risk tolerance/policy parameter. Minimum: 0.0 Maximum: 1.0
       */
      public double decel_safety_factor;
    }

    public Repulsion repulsion;

    public static final class Depth {
      /**
       * Distance (inches) from goal zone at which proximity speed reduction starts. Driver comfort
       * / early-warning feel parameter. Minimum: 0.0
       */
      public double slow_down;

      /**
       * Distance (inches) from goal zone where drive power towards zone is cut to zero. Safety
       * policy parameter. Minimum: 0.0
       */
      public double hard_stop;
    }

    public Depth depth;

    public static final class Side {
      /**
       * Distance (inches) from goal zone at which proximity speed reduction starts. Driver comfort
       * / early-warning feel parameter. Minimum: 0.0
       */
      public double slow_down;

      /**
       * Distance (inches) from goal zone where drive power towards zone is cut to zero. Safety
       * policy parameter. Minimum: 0.0
       */
      public double hard_stop;
    }

    public Side side;

    /**
     * Distance (inches) over which safety slows downs are interpolated from 0% to 100% strength to
     * prevent jerky speed changes. Minimum: 0.0
     */
    public double lane_blend_distance;

    public static final class Friction {
      /** X value coordinate. */
      public double x;

      /** Y value coordinate. */
      public double y;

      /**
       * Minimum turn/yaw drive power (0.0 to 1.0) needed to overcome static wheel stiction.
       * Calibrated using Friction Calibration OpMode. Minimum: 0.0 Maximum: 1.0
       */
      public double rot;
    }

    public Friction friction;

    public static final class Smoothing {
      /**
       * Apply directional power slew to Casablanca TeleOp input. Per-instance commands may override
       * it.
       */
      public boolean teleop_enabled;

      /**
       * Apply directional slew to Pedro FOLLOW/HOLD output. False preserves native autonomous
       * output independently of TeleOp.
       */
      public boolean auto_enabled;

      public static final class Forward {
        /**
         * Seconds per unit increase in signed robot-axis power; 0 is unrestricted. Includes
         * acceleration in the positive direction and braking from negative power. Tune pod contact
         * for each axis. Minimum: 0.0
         */
        public double increase_time;

        /**
         * Seconds per unit decrease in signed robot-axis power; 0 is unrestricted. Includes braking
         * from positive power and acceleration in the negative direction. Minimum: 0.0
         */
        public double decrease_time;
      }

      public Forward forward;

      public static final class Strafe {
        /**
         * Seconds per unit increase in signed robot-axis power; 0 is unrestricted. Includes
         * acceleration in the positive direction and braking from negative power. Tune pod contact
         * for each axis. Minimum: 0.0
         */
        public double increase_time;

        /**
         * Seconds per unit decrease in signed robot-axis power; 0 is unrestricted. Includes braking
         * from positive power and acceleration in the negative direction. Minimum: 0.0
         */
        public double decrease_time;
      }

      public Strafe strafe;

      public static final class Turn {
        /**
         * Seconds per unit increase in signed robot-axis power; 0 is unrestricted. Includes
         * acceleration in the positive direction and braking from negative power. Tune pod contact
         * for each axis. Minimum: 0.0
         */
        public double increase_time;

        /**
         * Seconds per unit decrease in signed robot-axis power; 0 is unrestricted. Includes braking
         * from positive power and acceleration in the negative direction. Minimum: 0.0
         */
        public double decrease_time;
      }

      public Turn turn;

      /**
       * Maximum elapsed seconds used by the shared slew limiter per update; prevents a large power
       * step after a stalled loop. Minimum: 0.001 Maximum: 0.2
       */
      public double max_loop_dt;
    }

    public Smoothing smoothing;

    public static final class HeadingLock {
      /** Enable/disable this subsystem, controller, watchdog, or feature. */
      public boolean enabled;

      /**
       * Heading feedback PID coefficients, bound as SDK PIDFCoefficients. Current Pedro 3 tuning is
       * proportional-only: keep I and D at zero unless separately tuned.
       */
      public com.qualcomm.robotcore.hardware.PIDFCoefficients pid;

      /**
       * Static friction feedforward magnitude. Pedro heading uses zero with the recovered
       * proportional-only tuning; separate from PID feedback. Minimum: 0.0 Maximum: 1.0
       */
      public double ks;

      /**
       * Raw joystick turn threshold (0.0 to 1.0) above which driver active steering takes control
       * and disengages heading lock. Minimum: 0.0 Maximum: 1.0
       */
      public double intent_threshold;

      /**
       * Minimum rotational drive power (0.0 to 1.0) needed to overcome friction while the robot is
       * already translating. Minimum: 0.0 Maximum: 1.0
       */
      public double ks_moving;

      /**
       * Chassis translational speed threshold (inches/sec) above which full ks_moving friction
       * feedforward is applied. Minimum: 0.0
       */
      public double moving_speed_threshold;

      /**
       * Maximum motor power (0.0 to 1.0) the loop is allowed to command. Clamped to be completely
       * imperceptible. Minimum: 0.0
       */
      public double max_power;

      /**
       * Heading error (degrees) below which heading lock applies zero correction, to avoid
       * chattering/jitter from sensor noise while holding a heading at rest. Minimum: 0.0 Maximum:
       * 15.0
       */
      public double error_deadband_deg;

      /**
       * Rotation rate (degrees/sec) below which the robot counts as settled. Heading lock waits for
       * this before capturing the heading to hold, so it does not latch mid-coast and fight the
       * robot's own momentum. Minimum: 0.0
       */
      public double settle_rate_dps;
    }

    public HeadingLock heading_lock;

    public static final class HeadingTuner {
      /**
       * Actual field starting pose [x inches, y inches, heading degrees] for every manually
       * repositioned jump-tuning trial. Collision protection uses this pose.
       */
      public Pose calibration_pose;

      /**
       * Signed heading step magnitude in each direction for TeleOp heading-lock tuning. Minimum:
       * 1.0 Maximum: 45.0
       */
      public double step_degrees;

      /**
       * Maximum deviation from each heading trial starting angle before an immediate stop. Minimum:
       * 1.0 Maximum: 90.0
       */
      public double maximum_sweep_degrees;

      /**
       * Maximum translation during an in-place heading trial before aborting. Minimum: 0.1 Maximum:
       * 12.0
       */
      public double position_drift_inches;

      /**
       * Required heading settling tolerance; must be at least the TeleOp heading-lock deadband.
       * Minimum: 0.1 Maximum: 10.0
       */
      public double heading_tolerance_degrees;

      /** Maximum absolute yaw rate for settled heading measurements. Minimum: 0.1 Maximum: 30.0 */
      public double rate_tolerance_dps;

      /**
       * Reject heading gains whose measured overshoot exceeds this limit. Minimum: 0.1 Maximum:
       * 20.0
       */
      public double overshoot_limit_degrees;

      /**
       * Stationary observation time after stopping; includes delayed chassis rocking in the jump
       * result. Minimum: 0.1 Maximum: 3.0
       */
      public double settle_sec;

      /**
       * Maximum duration of one jump-tuning trial including baseline and settling. Minimum: 0.5
       * Maximum: 30.0
       */
      public double trial_timeout_sec;

      /**
       * Linear velocity magnitude below which a jump-tuning trial is considered stationary.
       * Minimum: 0.01 Maximum: 5.0
       */
      public double stopped_speed_ips;

      /**
       * Delay between jump-tuner control-loop updates, in milliseconds. Minimum: 1.0 Maximum: 50.0
       */
      public int sample_period_ms;

      /**
       * Abort a heading trial after a control-loop delay above this duration. Minimum: 0.01
       * Maximum: 0.5
       */
      public double maximum_sample_gap_sec;

      /**
       * Lowest P gain tested by the TeleOp heading-lock step-response tuner (radians). Minimum: 0.0
       */
      public double p_min;

      /**
       * Highest P gain tested by the TeleOp heading-lock step-response tuner (radians). Minimum:
       * 0.0
       */
      public double p_max;

      /** Number of evenly spaced P candidates; 1 tests p_min only. Minimum: 1.0 Maximum: 10.0 */
      public int p_count;

      /**
       * Lowest D gain tested by the TeleOp heading-lock tuner using measured yaw rate. Minimum: 0.0
       */
      public double d_min;

      /**
       * Highest D gain tested by the TeleOp heading-lock tuner using measured yaw rate. Minimum:
       * 0.0
       */
      public double d_max;

      /** Number of evenly spaced D candidates; 1 tests d_min only. Minimum: 1.0 Maximum: 10.0 */
      public int d_count;
    }

    public HeadingTuner heading_tuner;

    public static final class JumpTuner {
      /**
       * Pod contact tuning axis: FORWARD, STRAFE, or TURN. Results apply only to that axis and
       * tested power; mixed inputs need robot validation.
       */
      public String trial_axis;

      /**
       * Requested yaw input for TURN pod contact trials, before friction compensation. Minimum:
       * 0.01 Maximum: 1.0
       */
      public double rotation_trial_power;

      /**
       * Powered angular travel before requesting a ramped stop during a TURN pod contact trial.
       * Minimum: 1.0 Maximum: 90.0
       */
      public double drive_angle_degrees;

      /**
       * Angular travel budget including stopping during a TURN pod contact trial. Minimum: 1.0
       * Maximum: 180.0
       */
      public double maximum_angle_degrees;

      /** Angular braking reserve within the TURN trial travel budget. Minimum: 0.1 Maximum: 45.0 */
      public double angular_stop_margin_degrees;

      /**
       * Conservative full-power yaw speed bound for TURN trial braking prediction, degrees per
       * second. Minimum: 1.0
       */
      public double angular_speed_bound_dps;

      /**
       * Hardware-map name of the Control Hub IMU used when Pinpoint firmware lacks pitch/roll.
       * Defaults to the FTC built-in IMU name. Pinpoint firmware 3 or newer is preferred
       * automatically.
       */
      public String imu_name;

      /** REV IMU logo-facing direction for the jump tuner; must match the physical mounting. */
      public String imu_logo;

      /** REV IMU USB-facing direction for the jump tuner; must match the physical mounting. */
      public String imu_usb;

      /**
       * Actual field starting pose [x inches, y inches, heading degrees] for every manually
       * repositioned jump-tuning trial. Collision protection uses this pose.
       */
      public Pose calibration_pose;

      /**
       * Requested forward/reverse input power for jump tuning, before Casablanca friction
       * compensation. Results are validated only at this power. Minimum: 0.01 Maximum: 1.0
       */
      public double trial_power;

      /**
       * Maximum measured cumulative travel per trial, including stopping. Power is removed earlier
       * using a stopping reserve. The tuner rejects values above the requested 48-inch ceiling.
       * Minimum: 1.0 Maximum: 48.0
       */
      public double max_travel_inches;

      /**
       * Maximum powered travel before requesting a ramped stop; total travel including stopping
       * remains subject to max_travel_inches. Minimum: 0.1 Maximum: 48.0
       */
      public double drive_distance_inches;

      /**
       * Unpowered stopping reserve in inches inside the per-trial traversal limit. Minimum: 0.1
       * Maximum: 24.0
       */
      public double stop_margin_inches;

      /**
       * Loop/sensor reaction-time allowance used in jump-tuning stopping-room prediction. Minimum:
       * 0.001 Maximum: 1.0
       */
      public double reaction_time_sec;

      /**
       * Conservative forward speed estimate at full power used by jump-tuning stopping-room
       * prediction; seeded from the prior 75.6428 in/s measurement. Minimum: 0.1
       */
      public double speed_bound_ips;

      /**
       * Stationary IMU baseline duration before each jump-tuning trial. Minimum: 0.1 Maximum: 5.0
       */
      public double baseline_sec;

      /**
       * Required duration at the requested trial power before a passing trial can certify a
       * setting. Minimum: 0.01 Maximum: 1.0
       */
      public double peak_hold_sec;

      /**
       * Stationary observation time after stopping; includes delayed chassis rocking in the jump
       * result. Minimum: 0.1 Maximum: 3.0
       */
      public double settle_sec;

      /**
       * Linear velocity magnitude below which a jump-tuning trial is considered stationary.
       * Minimum: 0.01 Maximum: 5.0
       */
      public double stopped_speed_ips;

      /**
       * Maximum pitch/roll angular-rate magnitude while taking the stationary baseline. Minimum:
       * 0.1 Maximum: 30.0
       */
      public double baseline_rate_dps;

      /**
       * Diagnostic combined chassis tilt threshold. Does not establish 4-bar pod contact or reject
       * a contact-confirmed trial. Minimum: 0.1 Maximum: 20.0
       */
      public double jump_tilt_deg;

      /**
       * Diagnostic combined pitch/roll rate threshold. Pod contact is confirmed by the observer.
       * Minimum: 1.0 Maximum: 500.0
       */
      public double jump_rate_dps;

      /**
       * Duration for the diagnostic tilt/rate indicator; pod contact verdict comes from the
       * observer. Minimum: 0.001 Maximum: 1.0
       */
      public double jump_hold_sec;

      /**
       * Baseline-relative tilt that aborts the trial immediately; must exceed jump_tilt_deg.
       * Minimum: 0.2 Maximum: 30.0
       */
      public double emergency_tilt_deg;

      /**
       * Maximum time without a new IMU orientation sample before aborting the trial. Minimum: 0.01
       * Maximum: 1.0
       */
      public double imu_stale_sec;

      /**
       * Maximum duration of one jump-tuning trial including baseline and settling. Minimum: 0.5
       * Maximum: 30.0
       */
      public double trial_timeout_sec;

      /**
       * Maximum time without measurable travel while drive power is requested. Minimum: 0.1
       * Maximum: 5.0
       */
      public double stall_timeout_sec;

      /**
       * Travel increment that refreshes the jump-tuner stall watchdog. Minimum: 0.01 Maximum: 2.0
       */
      public double minimum_progress_inches;

      /**
       * Angular travel that refreshes the TURN pod-contact trial stall watchdog. Minimum: 0.1
       * Maximum: 10.0
       */
      public double minimum_progress_degrees;

      /**
       * Maximum heading deviation from the trial start before aborting. Minimum: 0.1 Maximum: 45.0
       */
      public double heading_drift_deg;

      /**
       * Maximum sideways deviation from the straight calibration lane before aborting. Minimum: 0.1
       * Maximum: 12.0
       */
      public double lateral_drift_inches;

      /**
       * Allowed difference from the requested power when checking trial excitation and zero output.
       * Minimum: 1.0E-4 Maximum: 0.1
       */
      public double power_tolerance;

      /**
       * Delay between jump-tuner control-loop updates, in milliseconds. Minimum: 1.0 Maximum: 50.0
       */
      public int sample_period_ms;

      /**
       * Fastest nonzero directional slew time tested after an unrestricted contact trial fails.
       * Minimum: 0.001 Maximum: 2.0
       */
      public double minimum_slew_time;

      /**
       * Slowest directional slew time considered when finding a contact-confirmed baseline.
       * Minimum: 0.001 Maximum: 3.0
       */
      public double maximum_slew_time;

      /**
       * Multiplicative step between candidate slew settings; must exceed one. Minimum: 1.01
       * Maximum: 2.0
       */
      public double search_factor;

      /**
       * Maximum number of candidate settings evaluated; a pass requires both forward and reverse
       * trials. Minimum: 1.0 Maximum: 50.0
       */
      public int candidate_count;
    }

    public JumpTuner jump_tuner;
  }

  public static final class Shooter {
    /**
     * When true, uses the REV firmware velocity PID on the motor (RUN_USING_ENCODER with
     * setVelocity, gains from shooter.motor_pidf). When false, uses the custom Pedro PIDF
     * controller, anti-windup integrator, and voltage compensation (RUN_WITHOUT_ENCODER, gains from
     * shooter.pidf).
     */
    public boolean use_ftc_pid;

    /**
     * Static friction feedforward magnitude. Pedro heading uses zero with the recovered
     * proportional-only tuning; separate from PID feedback. Minimum: 0.0 Maximum: 1.0
     */
    public double ks;

    /**
     * Bus voltage the flywheel feedforward (ks and pidf.f) was characterized at. Shooter.periodic()
     * scales its motor command by nominal_voltage divided by the measured bus voltage, so a sagging
     * pack still reaches the commanded wheel speed instead of quietly landing short and failing the
     * feed gate. Set this to the voltage shown while the Shooter Characterization OpMode is
     * running, not to a fresh-off-the-charger reading. Minimum: 1.0 Maximum: 20.0
     */
    public double nominal_voltage;

    /**
     * Largest factor the flywheel voltage compensation may scale its command by. Bounds the
     * correction so a brownout or a bad voltage reading cannot rail the flywheel; 1.4 covers a 12.0
     * V feedforward down to about 8.6 V. Minimum: 1.0 Maximum: 3.0
     */
    public double max_voltage_compensation;

    /**
     * How often the flywheel re-reads bus voltage. The hub's voltage is a separate ADC round trip
     * rather than part of the bulk read, so polling it every loop costs milliseconds per tick; a
     * pack does not sag meaningfully inside this interval. Minimum: 0.0 Maximum: 5.0
     */
    public double voltage_refresh_sec;

    /**
     * Maximum rotational speed (RPM) of the motor. Used to scale power percentage to velocity.
     * Minimum: 0.0
     */
    public double max_rpm;

    /**
     * Continuous fixed flywheel target speed (RPM). Flywheel holds this RPM constantly so
     * time-to-shoot collapses to alignment time. Minimum: 0.0 Maximum: 3000.0
     */
    public double constant_rpm;

    /**
     * Flywheel speed threshold (percentage, 0.0 to 1.0) required to trigger the transfer belt.
     * Prevents launching rings before the shooter is up to speed. Minimum: 0.0 Maximum: 1.0
     */
    public double min_transfer_threshold;

    /**
     * Flywheel velocity upper bound scale factor (e.g. 1.05) to prevent overshooting before feeding
     * rings. Applies only when arming a feed, not while one is already running, since a ring can
     * only ever slow the flywheel down. Minimum: 1.0 Maximum: 2.0
     */
    public double max_velocity_threshold;

    /**
     * Fraction of target flywheel speed a feed already in progress may sag to before the intake
     * cuts out. Must be below min_transfer_threshold: it is the lower half of a hysteresis band, so
     * each ring entering the flywheel does not drop the speed under the arming floor and stutter
     * the intake off and back on. Raise it toward min_transfer_threshold for stricter shot energy,
     * lower it if the intake still hitches while feeding. Minimum: 0.0 Maximum: 1.0
     */
    public double feed_release_threshold;

    /**
     * Front intake power (0.0 to 1.0) when feeding rings to the shooter. Minimum: 0.0 Maximum: 1.0
     */
    public double feed_intake_power;

    /**
     * Transfer motor power (0.0 to 1.0) when feeding rings to the shooter. Minimum: 0.0 Maximum:
     * 1.0
     */
    public double feed_transfer_power;

    /**
     * Launch power ratio threshold above which the high hood position is selected for long-range
     * shots. Minimum: 0.0 Maximum: 1.0
     */
    public double long_hood_power_threshold;

    public static final class BallDetection {
      /**
       * How far flywheel speed must fall below its running reference, as a fraction, to count as a
       * ball leaving the shooter. Measured on this robot: real balls dip 12.2%-26.4%, the deepest
       * non-ball (the wheel coasting down after a shot) reaches 5.4%. Keep this between them; see
       * FlywheelDipDetector. Minimum: 0.0 Maximum: 1.0
       */
      public double dip_fraction;

      /**
       * How far flywheel speed must climb back up off a dip's own trough, as a fraction, before the
       * next ball can register. Must be strictly below dip_fraction. Ending a dip on a rebound off
       * the trough — not on a return to the pre-ball reference — is what lets balls that arrive
       * close together each get counted; see FlywheelDipDetector. Minimum: 0.0 Maximum: 1.0
       */
      public double rebound_fraction;

      /**
       * EMA weight per control loop for the ball detector's running reference speed. Small enough
       * that one ball cannot drag the reference down with it, large enough to track a flywheel
       * settling into a new cruise speed between shots. Minimum: 0.0 Maximum: 1.0
       */
      public double baseline_alpha;

      /**
       * Minimum spacing (milliseconds) between counted ball events. One ball produces a dip with a
       * ragged floor; without this, noise on the way back up would count as a second ball. Minimum:
       * 0.0
       */
      public int refractory_ms;
    }

    public BallDetection ball_detection;
    public com.qualcomm.robotcore.hardware.PIDFCoefficients pidf;

    /**
     * REV firmware velocity PIDF (p, i, d, f) applied directly to the flywheel motors via
     * DcMotorEx.setPIDFCoefficients when shooter.use_ftc_pid is true. Distinct from shooter.pidf,
     * which only feeds the software PIDFController used when use_ftc_pid is false and is never sent
     * to the motor. Keep i at 0 here: a live integral on the motor firmware winds up during a
     * feed's velocity sag and overshoots the setpoint once the ball leaves, which is what stalled
     * the third shot of a magazine.
     */
    public com.qualcomm.robotcore.hardware.PIDFCoefficients motor_pidf;

    public static final class Integral {
      /**
       * Flywheel velocity error (ticks/s) within which the integral term accumulates. Outside this
       * band the integrator holds its value, so spin-up and ball-feed transients never charge it.
       * Minimum: 0.0 Maximum: 2000.0
       */
      public double band_ticks;

      /**
       * Maximum absolute motor power the flywheel integral term may contribute. Bounds windup
       * independently of the integral gain. Minimum: 0.0 Maximum: 1.0
       */
      public double max_contribution;
    }

    public Integral integral;

    public static final class Ballistics {
      /** Fitted initial ring exit velocity (in/s) at constant RPM. Minimum: 0.0 */
      public double v0;

      /**
       * Fitted lumped quadratic drag coefficient (1/in) for 2D trajectory integration. Minimum: 0.0
       */
      public double k;

      /**
       * Fitted lumped Magnus lift coefficient (1/s) for 2D backspin trajectory integration.
       * Minimum: 0.0
       */
      public double magnus_l;

      /** Physical ring release height (inches) above field carpet floor. Minimum: 0.0 */
      public double launch_height_inches;

      /**
       * Center height (inches) of target scoring goal basket above field carpet floor. Minimum: 0.0
       */
      public double goal_height_inches;

      /** Gravitational acceleration constant (in/s²). Minimum: 0.0 */
      public double g_inches_per_sec2;

      /** Minimum physical hood elevation angle (degrees). Minimum: 0.0 Maximum: 90.0 */
      public double min_hood_angle_deg;

      /** Maximum physical hood elevation angle (degrees). Minimum: 0.0 Maximum: 90.0 */
      public double max_hood_angle_deg;

      /**
       * Continuous servo position (0.0 to 1.0) corresponding to min_hood_angle_deg, the flattest
       * arc. The hood linkage is reversed, so this is numerically GREATER than max_hood_servo_pos;
       * that is expected and the angle mapping handles it. Do not 'fix' the ordering. Minimum: 0.0
       * Maximum: 1.0
       */
      public double min_hood_servo_pos;

      /**
       * Continuous servo position (0.0 to 1.0) corresponding to max_hood_angle_deg, the steepest
       * arc. Servo 0.0 is as tall as the arc gets, so this is normally the smaller number of the
       * pair. Minimum: 0.0 Maximum: 1.0
       */
      public double max_hood_servo_pos;

      /** Maximum allowed robot translation speed (in/s) for valid moving shots. Minimum: 0.0 */
      public double max_moving_speed_ips;

      /** Minimum calibrated shot distance (inches) for valid solution. Minimum: 0.0 */
      public double min_valid_distance;

      /** Maximum calibrated shot distance (inches) for valid solution. Minimum: 0.0 */
      public double max_valid_distance;

      /** Empirical calibration multiplier for virtual lead target calculation. Minimum: 0.0 */
      public double lead_bias_gain;

      /**
       * Flywheel setpoint the fitted v0 exit velocity was measured at. Exit speed and flywheel
       * setpoint convert through v0 / v0_reference_rpm, which is how the solver turns a required
       * exit speed into an RPM. Must match the RPM the ballistics calibration actually ran at
       * (constant_rpm), or every solved RPM is scaled wrong. Minimum: 1.0 Maximum: 3000.0
       */
      public double v0_reference_rpm;

      /**
       * Flywheel setpoint the hood aims around. The solver holds this speed and lets the hood alone
       * cover every distance it can, so the flywheel stays parked at one well-calibrated,
       * already-spun-up speed. It only rises above this when no hood angle can reach the goal, and
       * never falls below it. This is the main shot-strength knob: raise it if shots land weak
       * across the board. Minimum: 0.0 Maximum: 3000.0
       */
      public double preferred_shot_rpm;

      /**
       * Highest flywheel setpoint the RPM solver may command, for distances past the hood's
       * authority at preferred_shot_rpm. A target that cannot reach goal height even at this speed
       * is reported as an invalid solution instead of being silently under-shot. Minimum: 0.0
       * Maximum: 3000.0
       */
      public double max_shot_rpm;

      /**
       * Headroom above the slowest exit speed that still reaches the goal (0.12 = 12% faster),
       * applied only to long shots that had to raise RPM past preferred_shot_rpm. Keeps those
       * solutions off the boundary where the flat and lofted trajectory branches merge; larger
       * values give a steeper, more forgiving entry angle at the cost of a hotter shot. Minimum:
       * 0.0 Maximum: 1.0
       */
      public double v0_margin_fraction;

      /**
       * Minimum change in solved RPM (ticks/s) before the flywheel setpoint is re-commanded. Every
       * setpoint write discards the flywheel's accumulated anti-windup integral, so without a
       * deadband a continuously drifting target would keep the integrator reset and the flywheel
       * permanently short of speed. Minimum: 0.0
       */
      public double rpm_update_deadband;

      /**
       * Ring time of flight per inch of range, used only to lead a moving shot. A flat estimate on
       * purpose: the measured shot table records hood and RPM but not flight time, and lead only
       * shifts the aim azimuth slightly, so a rough value is adequate here where a wrong hood
       * position would not be. Minimum: 0.0 Maximum: 0.1
       */
      public double flight_time_sec_per_inch;

      /**
       * Robot speed below which moving-shot lead is skipped entirely. A robot holding position
       * still reports a small noisy velocity, and lead turns that noise into aim-azimuth jitter
       * larger than the turret's settle tolerance, so the turret hunts instead of settling. Leading
       * a nearly-stationary robot gains nothing. Minimum: 0.0 Maximum: 100.0
       */
      public double lead_min_speed_ips;
    }

    public Ballistics ballistics;

    public static final class ShotTable {
      /**
       * Measured shot table as flat distance_in, hood_servo, flywheel_rpm triples, recorded by the
       * Ballistics Calibration OpMode. Distances must strictly increase. Looked up by linear
       * interpolation between rows and clamped outside them; a distance outside the calibrated
       * range is reported as an invalid solution rather than extrapolated.
       */
      public double[] points;
    }

    public ShotTable shot_table;
  }

  public static final class Auto {
    /**
     * Measured shoot windows as flat target_rpm, shoot_window_ms pairs, recorded by the Shot Timing
     * Tuner OpMode against whatever flywheel RPM the shot table commanded at each measured
     * distance. RPMs must strictly increase. Looked up by linear interpolation on the flywheel's
     * target RPM and clamped outside the measured range, since a shot just past the last row does
     * not suddenly get faster. Keying on RPM rather than distance means the table stays correct
     * after shooter.shot_table is recalibrated — the shot only ever behaved the way it did at the
     * RPM it was actually fired at, not at whatever distance a recalibrated table now maps that RPM
     * to. ShotController ends a scoring cycle when it counts auto.balls_per_shot_count balls OR
     * this window expires, whichever comes first — the window is the safety net for a jam, not the
     * primary signal. auto.shoot_wait_ms is only the fallback when this table is empty.
     */
    public double[] shot_time_points;

    /**
     * Balls loaded before each autonomous scoring cycle. ShotController counts them live via
     * shooter.ball_detection and ends the shot the moment this many are detected, without waiting
     * out the rest of auto.shot_time_points' window. Minimum: 1.0
     */
    public int balls_per_shot_count;

    /** Wait delay (milliseconds) at the drink gate when intake gathers rings. Minimum: 0.0 */
    public int drink_wait_ms;

    /**
     * Fallback wait time (milliseconds) allowed to fire a shot, used only when
     * auto.shot_time_points is empty. Prefer the measured table: a single flat number cannot
     * express that a close-range magazine takes longer than a long-range one. Minimum: 0.0
     */
    public int shoot_wait_ms;

    public double launch_power;

    /**
     * Target power coefficient (0.0 to 1.0) for autonomous transfer motor. Minimum: 0.0 Maximum:
     * 1.0
     */
    public double transfer_power;

    /**
     * Wait delay (milliseconds) at the drink gate for opposite alliance autonomous. Minimum: 0.0
     */
    public int opposite_drink_wait_ms;

    /**
     * Wait time (milliseconds) allowed to fire a shot for opposite alliance autonomous. Minimum:
     * 0.0
     */
    public int opposite_shoot_wait_ms;

    /**
     * Target power coefficient (0.0 to 1.0) for opposite alliance autonomous launcher. Minimum: 0.0
     * Maximum: 1.0
     */
    public double opposite_launch_power;

    /**
     * Target power coefficient (0.0 to 1.0) for opposite alliance autonomous transfer. Minimum: 0.0
     * Maximum: 1.0
     */
    public double opposite_transfer_power;

    public static final class Endgame {
      /**
       * Length of the autonomous period (milliseconds), measured from the moment the OpMode starts.
       * The endgame timers below are all counted backwards from it. Minimum: 0.0
       */
      public int period_ms;

      /**
       * Time left in autonomous (milliseconds) at which the auto sequence is cancelled and the
       * robot drives the shortest straight line to a spot where it is committed — entirely inside a
       * launch zone or entirely outside every one. A robot straddling a zone boundary at the buzzer
       * gets neither the zone's benefit nor out of the next alliance's way. Minimum: 0.0
       */
      public int evacuate_ms;

      /**
       * Backstop deadline (milliseconds left in autonomous) at which the robot stops driving even
       * if it never reached a committed spot. Normally motion stops earlier, on arrival. Must be
       * smaller than auto.endgame.evacuate_ms, which is the window the move gets to finish in.
       * Minimum: 0.0
       */
      public int freeze_ms;

      /**
       * Inches of commitment required on whichever side of the launch zone boundary the robot parks
       * on: its whole footprint must clear the line by this much, inside or out. Absorbs the
       * follower's settling error so a spot that ends slightly short is still unambiguous. Measured
       * against the footprint's half-diagonal, so it holds at any heading. Minimum: 0.0
       */
      public double exit_clearance;
    }

    public Endgame endgame;
  }

  public static final class Teleop {
    /**
     * Maximum speed scaler (0.0 to 1.0) applied to chassis manual TeleOp drive inputs. Minimum: 0.0
     * Maximum: 1.0
     */
    public double max_speed;

    /**
     * When true, driver translation sticks are interpreted in the field frame (stick direction is a
     * fixed field direction regardless of robot heading). When false, sticks are robot-relative.
     */
    public boolean field_centric;

    /**
     * Field heading (degrees) that a fully-forward driver stick should drive toward when
     * field_centric is true. 90 means stick-forward drives along field +Y, i.e. away from the
     * driver wall at y=0. Set this to whatever direction is 'away' from where the drivers
     * physically stand. Minimum: -360.0 Maximum: 360.0
     */
    public double field_centric_offset_deg;

    /** Target motor power (0.0 to 1.0) for intake rollers in TeleOp. Minimum: 0.0 Maximum: 1.0 */
    public double intake_power;

    /**
     * Target power coefficient (0.0 to 1.0) for autonomous transfer motor. Minimum: 0.0 Maximum:
     * 1.0
     */
    public double transfer_power;

    /**
     * Target flywheel power ratio (0.0 to 1.0) for manual rev command in TeleOp. Minimum: 0.0
     * Maximum: 1.0
     */
    public double manual_rev_power;

    public static final class Poses {
      public static final class Red {
        /** Starting position coordinate pose. */
        public Pose start;

        /** High goal target scoring position coordinate pose. */
        public Pose score;

        /** Drink gate alignment coordinate pose. */
        public Pose drink;

        /** Parking destination coordinate pose. */
        public Pose park;
      }

      public Red red;

      public static final class Blue {
        /** Starting position coordinate pose. */
        public Pose start;

        /** High goal target scoring position coordinate pose. */
        public Pose score;

        /** Drink gate alignment coordinate pose. */
        public Pose drink;

        /** Parking destination coordinate pose. */
        public Pose park;
      }

      public Blue blue;
    }

    public Poses poses;
  }

  public static final class NormalAuto {
    /** Control point pose on path to drink gate. */
    public Pose drinkCp;

    /** Ending position coordinate pose at the drink gate. */
    public Pose drinkEnd;

    /** Final scoring target position coordinate pose. */
    public Pose finalScore;

    /** Control point pose on path to first sample pickup. */
    public Pose pickup1Cp;

    /** Ending position coordinate pose for first sample pickup. */
    public Pose pickup1End;

    /** Control point pose on path to second sample pickup. */
    public Pose pickup2Cp;

    /** Ending position coordinate pose for second sample pickup. */
    public Pose pickup2End;

    /** Control point pose on path to third sample pickup. */
    public Pose pickup3Cp;

    /** Ending position coordinate pose for third sample pickup. */
    public Pose pickup3End;

    /** High goal target scoring position coordinate pose. */
    public Pose score;

    /** Starting position coordinate pose. */
    public Pose start;

    // Shared operational parameters (same for both alliances)
    /**
     * Measured shoot windows as flat target_rpm, shoot_window_ms pairs, recorded by the Shot Timing
     * Tuner OpMode against whatever flywheel RPM the shot table commanded at each measured
     * distance. RPMs must strictly increase. Looked up by linear interpolation on the flywheel's
     * target RPM and clamped outside the measured range, since a shot just past the last row does
     * not suddenly get faster. Keying on RPM rather than distance means the table stays correct
     * after shooter.shot_table is recalibrated — the shot only ever behaved the way it did at the
     * RPM it was actually fired at, not at whatever distance a recalibrated table now maps that RPM
     * to. ShotController ends a scoring cycle when it counts auto.balls_per_shot_count balls OR
     * this window expires, whichever comes first — the window is the safety net for a jam, not the
     * primary signal. auto.shoot_wait_ms is only the fallback when this table is empty.
     */
    public double[] shotTimePoints;

    /**
     * Balls loaded before each autonomous scoring cycle. ShotController counts them live via
     * shooter.ball_detection and ends the shot the moment this many are detected, without waiting
     * out the rest of auto.shot_time_points' window.
     */
    public int ballsPerShotCount;

    /** Wait delay (milliseconds) at the drink gate when intake gathers rings. */
    public int drinkWaitMs;

    /**
     * Fallback wait time (milliseconds) allowed to fire a shot, used only when
     * auto.shot_time_points is empty. Prefer the measured table: a single flat number cannot
     * express that a close-range magazine takes longer than a long-range one.
     */
    public int shootWaitMs;

    public double launchPower;

    /** Target power coefficient (0.0 to 1.0) for autonomous transfer motor. */
    public double transferPower;

    /** Wait delay (milliseconds) at the drink gate for opposite alliance autonomous. */
    public int oppositeDrinkWaitMs;

    /** Wait time (milliseconds) allowed to fire a shot for opposite alliance autonomous. */
    public int oppositeShootWaitMs;

    /** Target power coefficient (0.0 to 1.0) for opposite alliance autonomous launcher. */
    public double oppositeLaunchPower;

    /** Target power coefficient (0.0 to 1.0) for opposite alliance autonomous transfer. */
    public double oppositeTransferPower;

    public String endgame;
  }

  public static final class OppositeAuto {
    /** Control point pose on path to drink gate. */
    public Pose drinkCp;

    /** Ending position coordinate pose at the drink gate. */
    public Pose drinkEnd;

    /** Parking destination coordinate pose. */
    public Pose park;

    /** Control point pose on path to second sample pickup. */
    public Pose pickup2Cp;

    /** Ending position coordinate pose for second sample pickup. */
    public Pose pickup2End;

    /** Control point pose on path to third sample pickup. */
    public Pose pickup3Cp;

    /** Ending position coordinate pose for third sample pickup. */
    public Pose pickup3End;

    /** Control point pose on path to fourth sample pickup. */
    public Pose pickup4Cp;

    /** Ending position coordinate pose for fourth sample pickup. */
    public Pose pickup4End;

    /** High goal target scoring position coordinate pose. */
    public Pose score;

    /** Starting position coordinate pose. */
    public Pose start;

    // Shared operational parameters (same for both alliances)
    /**
     * Measured shoot windows as flat target_rpm, shoot_window_ms pairs, recorded by the Shot Timing
     * Tuner OpMode against whatever flywheel RPM the shot table commanded at each measured
     * distance. RPMs must strictly increase. Looked up by linear interpolation on the flywheel's
     * target RPM and clamped outside the measured range, since a shot just past the last row does
     * not suddenly get faster. Keying on RPM rather than distance means the table stays correct
     * after shooter.shot_table is recalibrated — the shot only ever behaved the way it did at the
     * RPM it was actually fired at, not at whatever distance a recalibrated table now maps that RPM
     * to. ShotController ends a scoring cycle when it counts auto.balls_per_shot_count balls OR
     * this window expires, whichever comes first — the window is the safety net for a jam, not the
     * primary signal. auto.shoot_wait_ms is only the fallback when this table is empty.
     */
    public double[] shotTimePoints;

    /**
     * Balls loaded before each autonomous scoring cycle. ShotController counts them live via
     * shooter.ball_detection and ends the shot the moment this many are detected, without waiting
     * out the rest of auto.shot_time_points' window.
     */
    public int ballsPerShotCount;

    /** Wait delay (milliseconds) at the drink gate when intake gathers rings. */
    public int drinkWaitMs;

    /**
     * Fallback wait time (milliseconds) allowed to fire a shot, used only when
     * auto.shot_time_points is empty. Prefer the measured table: a single flat number cannot
     * express that a close-range magazine takes longer than a long-range one.
     */
    public int shootWaitMs;

    public double launchPower;

    /** Target power coefficient (0.0 to 1.0) for autonomous transfer motor. */
    public double transferPower;

    public String endgame;
  }

  public static MatchProfile loadMatchProfile(Alliance alliance) {
    return MatchProfile.loadMatchProfile(alliance);
  }
}

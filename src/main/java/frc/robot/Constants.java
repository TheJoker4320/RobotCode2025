// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    public static final int LOW_SPEED_SWERVE_BUTTON = XboxController.Button.kA.value;
    public static final int MEDIUM_SPEED_SWERVE_BUTTON = XboxController.Button.kB.value;
    public static final int REGULAR_SPEED_SWERVE_BUTTON = XboxController.Button.kY.value;
    public static final int RESET_HEADING_SWERVE_BUTTON = XboxController.Button.kLeftBumper.value;
    public static final int REFERENCE_FRAME_SWERVE_BUTTON = XboxController.Button.kRightBumper.value;

    public static final double DRIVE_DEADBAND = 0.05;
  }

  public static class PoseEstimatorConstants {
    // The larger the following values are the less the pose estimator trusts the measurements - if we see 
    // large ambiguity increase the values, if we see high precision than decrease the values
    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));      // TODO: Calibrate values
    public static final Matrix<N3, N1> VISION_STANDARD_DEVIATIONS = VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(15));      // These values are most likely too high - should be tested, TODO: Calibrate values
  
    public static final double MAXIMUM_ANGULAR_VELOCITY = 720;

    public static final double REEF_X_OFFSET = -0.4;
    public static final double REEF_Y_RIGHT_OFFSET = -0.17;
    public static final double REEF_Y_LEFT_OFFSET = 0.17;
    public static final double APRIL_TAG_ANGLE_OFFSET = Math.PI + Math.PI / 2;

    // It is by definition that 0 degree angle is towards the red alliance drivers - so for the blue alliance it is consistent
    // but for the red alliance we would expect 0 to face the blue alliance drivers so we must shift the gyro angle by 180 degrees;
    public static final double BLUE_GYRO_OFFSET = 0;          // Values is in degrees
    public static final double RED_GYRO_OFFSET = 180;         // Values is in degrees

    public static final HashMap<Integer, Pose2d> REEF_APRIL_TAG_POSITIONS = new HashMap<Integer, Pose2d>();
    
    static {
      REEF_APRIL_TAG_POSITIONS.put(6, new Pose2d(13.474, 3.306, Rotation2d.fromDegrees(300)));
      REEF_APRIL_TAG_POSITIONS.put(7, new Pose2d(13.89, 4.0259, Rotation2d.fromDegrees(0)));
      REEF_APRIL_TAG_POSITIONS.put(8, new Pose2d(13.474, 4.745, Rotation2d.fromDegrees(60)));
      REEF_APRIL_TAG_POSITIONS.put(9, new Pose2d(12.643, 4.745, Rotation2d.fromDegrees(120)));
      REEF_APRIL_TAG_POSITIONS.put(10, new Pose2d(12.227, 4.0259, Rotation2d.fromDegrees(180)));
      REEF_APRIL_TAG_POSITIONS.put(11, new Pose2d(12.643, 3.306, Rotation2d.fromDegrees(240)));
    
      REEF_APRIL_TAG_POSITIONS.put(17, new Pose2d(4.074, 3.306, Rotation2d.fromDegrees(240)));
      REEF_APRIL_TAG_POSITIONS.put(18, new Pose2d(3.657, 4.025, Rotation2d.fromDegrees(180)));
      REEF_APRIL_TAG_POSITIONS.put(19, new Pose2d(4.074, 4.745, Rotation2d.fromDegrees(120)));
      REEF_APRIL_TAG_POSITIONS.put(20, new Pose2d(4.904, 4.745, Rotation2d.fromDegrees(60)));
      REEF_APRIL_TAG_POSITIONS.put(21, new Pose2d(5.321, 4.025, Rotation2d.fromDegrees(0)));
      REEF_APRIL_TAG_POSITIONS.put(22, new Pose2d(4.904, 3.306, Rotation2d.fromDegrees(300)));
    }
  }

  public static class SwerveSubsystemConstants {
    public static final double TRACK_WIDTH = 0.675;                 // distance between centers of right and left modules TODO: Validate measurment
    public static final double WHEEL_BASE = 0.675;                  // distance between centers of front and rear modules TODO: Validate measurment

    public static final double SLOW_INPUT_MULTIPLIER = 0.3;
    public static final double MEDIUM_INPUT_MULTIPLIER = 0.7;
    public static final double REGULAR_INPUT_MULTIPLIER = 1;
  
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
    );

    public static final double MAX_SPEED = 4.8;                     // Meters per second
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;     // Radians per second

    public static final double X_STATE_ANGLE = Math.PI / 4;         // Radians

    public static final int PIGEON_DEVICE_ID = 14;                   // TODO: Validate device id
  }

  public static final class NeoModuleConstants {
    public static final int MODULE_COUNT = 4;
    
    public static final int[] DRIVING_CAN_ID = new int[] {8, 6, 2, 4};        // TODO: Validate device id
    public static final int[] TURNING_CAN_ID = new int[] {7, 5, 1, 3};        // TODO: Validate device id
    public static final double[] ANGULAR_OFFSETS = new double[] {
      (-Math.PI / 2.0), 0, (Math.PI), (Math.PI / 2.0)
    };                                                                        // TODO: Validate device id

    public static final int DRIVING_MOTOR_PINION_TEETH = 16;                  // TODO: Validate value
    public static final int SPUR_GEAR_TEETH = 19;                             // TODO: Validate value
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * SPUR_GEAR_TEETH) / (DRIVING_MOTOR_PINION_TEETH * 15.0);

    public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI);           // Radians
    public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0;    // Radians per seconds

    public static final double DRIVING_FREE_SPEED_RPM = 5676;
    public static final double DRIVING_FREE_SPEED_RPS = DRIVING_FREE_SPEED_RPM / 60.0;

    public static final double WHEEL_DIAMETER_METERS = 0.0762;                // TODO: Validate value
    public static final double WHEEL_CIRCUFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVING_WHEEL_FREE_SPEED_RPS = (DRIVING_FREE_SPEED_RPS * WHEEL_CIRCUFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;
    public static final double DRIVING_ENCODER_POSITION_FACTOR = WHEEL_CIRCUFERENCE_METERS / DRIVING_MOTOR_REDUCTION;
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR = DRIVING_ENCODER_POSITION_FACTOR / 60.0;

    public static final int DRIVING_SMART_CURRENT_LIMIT = 50;                 // TODO: Validate value
    public static final int TURNING_SMART_CURRENT_LIMIT = 20;                 // TODO: Validate value

    public static final boolean TURNING_ENCODER_INVERTED = true;

    public static final double DRIVING_P_CONSTANT = 0.04;
    public static final double DRIVING_I_CONSTANT = 0;
    public static final double DRIVING_D_CONSTANT = 0;
    public static final double DRIVING_VELOCITY_FF = 1.0 / DRIVING_WHEEL_FREE_SPEED_RPS;

    public static final double TURNING_P_CONSTANT = 1;
    public static final double TURNING_I_CONSTANT = 0;
    public static final double TURNING_D_CONSTANT = 0;

    public static final double MAX_TURNING_ENCODER_VALUE = 2.0 * Math.PI;
  }
}

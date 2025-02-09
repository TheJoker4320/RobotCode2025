// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.PS4Controller;
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
    public static final int DRIVING_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // Swerve operator constants
    public static final int LOW_SPEED_SWERVE_BUTTON = XboxController.Button.kA.value;
    public static final int MEDIUM_SPEED_SWERVE_BUTTON = XboxController.Button.kB.value;
    public static final int REGULAR_SPEED_SWERVE_BUTTON = XboxController.Button.kY.value;
    public static final int RESET_HEADING_SWERVE_BUTTON = XboxController.Button.kLeftBumper.value;
    public static final int REFERENCE_FRAME_SWERVE_BUTTON = XboxController.Button.kRightBumper.value;

    public static final double DRIVE_DEADBAND = 0.05;
    
    // Elevator operator constants
    public static final int ELEVATOR_LOW_STATE = PS4Controller.Button.kSquare.value;
    public static final int ELEVATOR_HIGH_STATE = PS4Controller.Button.kTriangle.value;
  }

  public static class ElevatorConstants {

    public static final double SPROCKET_PITCH_DIAMETER = 0.0446193811;      // The pitch diameter of the sprocket in meters

    public static final double MOTOR_GEAR_RATIO_REDUCTION = 20;                

    public static final double ELV_SENSOR_TO_MECAHNISM_RATIO = (MOTOR_GEAR_RATIO_REDUCTION) / (2 * SPROCKET_PITCH_DIAMETER * Math.PI);

    public static final int ENCODER_CHANNEL = 3;
    public static final int RIGHT_MOTOR_DEVICE_ID = 12;
    public static final int LEFT_MOTOR_DEVICE_ID = 11;

    public static final boolean LEFT_OPPOSITE_OF_RIGHT = true;

    public static final double ELEVATOR_ENCODER_TOLERANCE = 0.01;
    public static final double ELEVATOR_POSITION_TOLERANCE = 0.005;

    public static final double LOW_POSITION_HEIGHT = 0.1;               // Example values - in meters
    public static final double HIGH_POSITION_HEIGHT = 1.26;             // Example values - in meters

    public static final double MINIMUM_ELEVATOR_HEIGHT = 0;             // This value is in meters
    public static final boolean MINIMUM_VALUE_ENABLED = true;
    public static final double MAXIMUM_ELEVATOR_HEIGHT = 1.3;           // This value is in meters
    public static final boolean MAXIMUM_VALUE_ENABLED = true;

    public static final double ELEVATOR_P_CONSTANT = 28.936;            // This is a calibrated value - do not lose it
    public static final double ELEVATOR_I_CONSTANT = 0;                 // This is a calibrated value - do not lose it
    public static final double ELEVATOR_D_CONSTANT = 1.655;             // This is a calibrated value - do not lose it
    public static final double ELEVATOR_G_CONSTANT = 0.10618;           // This is a calibrated value - do not lose it
    public static final double ELEVATOR_V_CONSTANT = 8.2604;            // This is a calibrated value - do not lose it
    public static final double ELEVATOR_A_CONSTANT = 0.19692;           // This is a calibrated value - do not lose it
    public static final double ELEVATOR_S_CONSTANT = 0.11453;           // This is a calibrated value - do not lose it

    public static final double MM_CRUISE_VELOCITY = 1.3;                // This is a calibrated value - do not lose it - meters per second
    public static final double MM_ACCELERATION = 20;                    // This is a calibrated value - do not lose it - meters per second^2
    public static final double MM_JERK = 140;                           // This is a calibrated value - do not lose it - meters per second^3

    public static final boolean CURRENT_LIMIT_ENABLED = true;
    public static final double CURRENT_LIMIT = 50;

    public static final boolean MOTIONMAGIC_ENABLED = true;

    public static final double REACHSTATE_TIMEOUT = 4;                  // Maximum time for a reachstate command (in seconds)

    public static final InvertedValue RIGHT_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;

    public static final Angle MOTOR_OFFSET = Rotation.of(0.0835);   // This is a measured value - might need to measure and validate it once in a while
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

    public static final int PIGEON_DEVICE_ID = 0;                   // TODO: Validate device id
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

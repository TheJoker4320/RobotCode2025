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
import static edu.wpi.first.units.Units.Rotation;
import com.ctre.phoenix6.signals.InvertedValue;

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
    // public static final int CLOSE_CLIMBER_BUTTON = 7; //The Minus Button
    // public static final int CLIMBER_BUTTON = 8; //The Plus button

    public static final int LOW_SPEED_SWERVE_BUTTON = XboxController.Button.kB.value;
    public static final int MEDIUM_SPEED_SWERVE_BUTTON = XboxController.Button.kA.value;
    public static final int REGULAR_SPEED_SWERVE_BUTTON = XboxController.Button.kX.value;
    public static final int RESET_HEADING_SWERVE_BUTTON = 7; //minus #TODO: Check if actually this button
    public static final int REFERENCE_FRAME_SWERVE_BUTTON = 8; //plus

    public static final double DRIVE_DEADBAND = 0.05;
    
    public static final int OPEN_BALL_POV_BUTTON = 0;         // This means UP on the pov button
    public static final int CLOSE_BALL_POV_BUTTON = 180;      // This means DOWN on the pov button
    public static final int COLLECT_POV_BUTTON = 90;          // This means RIGHT on the pov button
    public static final int EJECT_POV_BUTTOn = 270;           // This means LEFT on the pov button

    // Elevator operator constants
    public static final int L1_STATE_BUTTON = PS4Controller.Button.kL1.value;
    public static final int L2_STATE_BUTTON = PS4Controller.Button.kL2.value;
    public static final int L3_STATE_BUTTON = PS4Controller.Button.kR1.value;
    public static final int L4_STATE_BUTTON = PS4Controller.Button.kR2.value;
    public static final int INTAKE_PREPARE_BUTTON = PS4Controller.Button.kTriangle.value;
    public static final int PLACE_CORAL_BUTTON = PS4Controller.Button.kCircle.value;
    public static final int INTAKE_BUTTON = PS4Controller.Button.kCross.value;
    public static final int L2_BALL_STATE_BUTTON = PS4Controller.Button.kOptions.value;
    public static final int L3_BALL_STATE_BUTTON = PS4Controller.Button.kShare.value;
    public static final int COLLECT_BALL_BUTTON = PS4Controller.Button.kR3.value;
    public static final int EJECT_MANIPULATOR_BALL_BUTTON = PS4Controller.Button.kL3.value;
    
    public static final int CLIMBER_BUTTON = 0;
    public static final int CLOSE_CLIMBER_BUTTON = 180;
  }
  public static class BallCollectorConstants {
    public static final int ARM_MOTOR_PORT = 17;
    public static final int COLLECTOR_MOTOR_PORT = 16;

    public static final double POSITION_FACTOR = 360;
    public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60;

    public static final double P_CONSTANT = 0.005;
    public static final double I_CONSTANT = 0;
    public static final double D_CONSTANT = 0;

    public static final double MIN_POSITION = 307;
    public static final double MAX_POSITION = 268;
    
    public static final int ARM_CURRENT_LIMIT = 40;
    public static final int COLLECTOR_CURRENT_LIMIT = 20;

    public static final double OPEN_POSITION = 306;
    public static final double CLOSE_POSITION = 269;
    public static final double RELEASE_COLLECTION_SPEED = 290;
    public static final double POSITION_TOLERANCE = 0.5;

    public static final double COLLECT_SPEED = -0.8;
    public static final double EJECT_SPEED = -1 * COLLECT_SPEED;
  }

  public static class ClimberConstants {
    public static final int CLIMBER_MOTOR_PORT = 15;
    public static final double CLIMB_SPEED = 0.2; //TODO: Need to validate values
    public static final double CLOSE_CLIMBER_SPEED = -0.2; //TODO: Need to validate values
    public static final InvertedValue MOTOR_INVERTED = InvertedValue.Clockwise_Positive; 
    public static final boolean CURRENT_LIMIT_ENABLED = true; 
    public static final double CURRENT_LIMIT = 50; //TODO: Need to validate values
  }
  
  public static class ArmConstants {
	  public static final double MOTOR_TO_ARM_GEAR_RATIO = (4.0 * 4.0 * 3.0 * 42.0) / (18.0);
    public static final double ENCODER_TO_ARM_GEAR_RATIO = 1;
    
    public static final boolean SMART_CURRENT_LIMIT_ENABLED = true;
    public static final double SMART_CURRENT_LIMIT = 40;

    public static final int ENCODER_CHANNEL = 0;
    public static final int MOTOR_ID = 13;

    public static final double ARM_POSITION_TOLERANCE = 3;  // in degrees
    public static final double ARM_ENCODER_TOLERANCE = 0.5;   // in degrees
    public static final double ARM_ENCODER_OFFSET = -129.61616;   // in degrees

    //all PID values are in rotations, not degrees/radians
    public static final double ARM_KP = 30;
    public static final double ARM_KI = 0;
    public static final double ARM_KD = 11.88016;
    public static final double ARM_KG = 0.29506;  
    public static final double ARM_KV = 16.5144;  
    public static final double ARM_KA = 1.63256;  
    public static final double ARM_KS = 0.3766;

    public static final double ARM_KG_STAY = 0.1;

    public static final InvertedValue INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;

    public static final double MIN_ANGLE_L2_HEIGHT = -0;
    public static final double INTAKE_ANGLE = -85;
    public static final double OUT_OF_INTAKE_ANGLE = -65;
    public static final double L4_ANGLE = 67;
    public static final double L32_ANGLE = 69;
    public static final double L1_ANGLE = -31;
    public static final double L32_BALL_PRE_COLLECT_ANGLE = -15;
    public static final double L32_BALL_COLLECT_ANGLE = 0;
    public static final double L4_PLACED_ANGLE = 32;     // This angle isnt final - needs to be tested
    public static final double L32_PLACED_ANGLE = 43;    // This angle isnt final - needs to be tested
    public static final double ZERO_ANGLE = 0.0;

    public static final boolean MAXIMUM_VALUE_ENABLED = true;
    public static final boolean MINIMUM_VALUE_ENABLED = true;
    public static final double MINIMUM_ARM_ANGLE = -88;       // min - in degrees
    public static final double MAXIMUM_ARM_ANGLE = 73;        // max - in degrees

    public static final double MM_CRUISE_VELOCITY = 250;  // degrees per second
    public static final double MM_ACCELERATION = 500;     // degrees per second^2
    public static final double MM_JERK = 2500;            // degrees per second^3

    public static final boolean IS_MAGIC_MOTION_ENABLED = true;

  }

  public static class ManipulatorConstants {
    public static int BALL_MOTOR_ID = 9;
    public static int CORAL_MOTOR_ID = 10;

    /*This is the coral limit switch port */
    public static int CORAL_SWITCH_PORT = 1; //TODO: set port
    
    /*This is the ball limit switch port */
    public static int BALL_SWITCH_PORT = 2; //TODO: set port

    public static final boolean BALL_COLLECTOR_INVERTED = true;

    public static final int MANIPULATOR_BALL_SMART_CURRENT_LIMIT = 20; //TODO: validate value
    public static final int MANIPULATOR_CORAL_SMART_CURRENT_LIMIT = 20; //TODO: validate value

    //TODO: set correct speed for manipulator
    public static final double BALL_COLLECT_SPEED = 0.75; 
    public static final double CORAL_COLLECT_SPEED = 0.5;
    public static final double BALL_EJECT_SPEED = -0.5;
    public static final double CORAL_EJECT_SPEED = -0.2;
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

    public static final double PRE_INTAKE_POSITION = 0.59;
    public static final double INTAKE_POSITION = 0.4525;  //TODO: add to value +- 2 centimeters
    public static final double PRE_SCORING = 0.55;
    public static final double L2_BALL_POSITION = 0.475;
    public static final double L3_BALL_POSITION = 0.84;
    public static final double L4_POSITION = 1.257;
    public static final double L3_POSITION = 0.617;
    public static final double L2_POSITION = 0.185;
    public static final double L1_POSITION = 0.701;
    
    public static final double MINIMUM_ELEVATOR_HEIGHT = 0.15;          // This value is in meters
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

  public static class PoseEstimatorConstants {
    // The larger the following values are the less the pose estimator trusts the measurements - if we see 
    // large ambiguity increase the values, if we see high precision than decrease the values
    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));      // TODO: Calibrate values
    public static final Matrix<N3, N1> VISION_STANDARD_DEVIATIONS = VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(15));      // These values are most likely too high - should be tested, TODO: Calibrate values
  
    public static final double MAXIMUM_ANGULAR_VELOCITY = 720;

    public static final double FAR_REEF_X_OFFSET = -0.48;
    public static final double CLOSE_REEF_X_OFFSET = -0.32;          // The distance front edge of the robot to the center plus a few centimeters - depends on with/without bumpers
    public static final double REEF_Y_RIGHT_OFFSET = -0.24;   // The distance between the center of the april tag and reef branch
    public static final double REEF_Y_LEFT_OFFSET = 0.17;     // The distance between the center of the april tag and reef branch
    public static final double APRIL_TAG_ANGLE_OFFSET = Math.PI + Math.PI / 2;

    // It is by definition that 0 degree angle is towards the red alliance drivers - so for the blue alliance it is consistent
    // but for the red alliance we would expect 0 to face the blue alliance drivers so we must shift the gyro angle by 180 degrees;
    public static final double BLUE_GYRO_OFFSET = 0;          // Values is in degrees
    public static final double RED_GYRO_OFFSET = 180;         // Values is in degrees

    public static final HashMap<Integer, Pose2d> REEF_APRIL_TAG_POSITIONS = new HashMap<Integer, Pose2d>();

    public static final double MAX_SPEED = 2;
    public static final double MAX_ACCELERATION = 2;
    public static final double MAX_ANGULAR_SPEED = 1.5 * Math.PI;
    public static final double MAX_ANGULAR_ACCELERATION = 3 * Math.PI;
    
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

    public static final int PIGEON_DEVICE_ID = 14;
  }

  public static final class NeoModuleConstants {
    public static final int MODULE_COUNT = 4;
    
    public static final int[] DRIVING_CAN_ID = new int[] {8, 6, 2, 4};
    public static final int[] TURNING_CAN_ID = new int[] {7, 5, 1, 3};
    public static final double[] ANGULAR_OFFSETS = new double[] {
      (-Math.PI / 2.0), 0, (Math.PI), (Math.PI / 2.0)
    };

    public static final int DRIVING_MOTOR_PINION_TEETH = 14;
    public static final int SPUR_GEAR_TEETH = 22;
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * SPUR_GEAR_TEETH) / (DRIVING_MOTOR_PINION_TEETH * 15.0);

    public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI);           // Radians
    public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0;    // Radians per seconds

    public static final double DRIVING_FREE_SPEED_RPM = 5676;
    public static final double DRIVING_FREE_SPEED_RPS = DRIVING_FREE_SPEED_RPM / 60.0;

    public static final double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    public static final double DRIVING_WHEEL_FREE_SPEED_RPS = (DRIVING_FREE_SPEED_RPS * WHEEL_CIRCUFERENCE_METERS) / DRIVING_MOTOR_REDUCTION;
    public static final double DRIVING_ENCODER_POSITION_FACTOR = WHEEL_CIRCUFERENCE_METERS / DRIVING_MOTOR_REDUCTION;
    public static final double DRIVING_ENCODER_VELOCITY_FACTOR = DRIVING_ENCODER_POSITION_FACTOR / 60.0;

    public static final int DRIVING_SMART_CURRENT_LIMIT = 50;
    public static final int TURNING_SMART_CURRENT_LIMIT = 20;

    public static final boolean TURNING_ENCODER_INVERTED = true;

    public static final double DRIVING_P_CONSTANT = 0.026179521;
    public static final double DRIVING_I_CONSTANT = 0;
    public static final double DRIVING_D_CONSTANT = 0;
    public static final double DRIVING_VELOCITY_FF = 1.0 / DRIVING_WHEEL_FREE_SPEED_RPS;

    public static final double TURNING_P_CONSTANT = 1;
    public static final double TURNING_I_CONSTANT = 0;
    public static final double TURNING_D_CONSTANT = 0;

    public static final double MAX_TURNING_ENCODER_VALUE = 2.0 * Math.PI;
  }

  public static class AutonomousConstants {
    public static final double TRANSLATION_P_CONSTANT = 2.843395833;
    public static final double TRANSLATION_I_CONSTANT = 0;
    public static final double TRANSLATION_D_CONSTANT = 0.28391875;

    public static final double ROTATION_P_CONSTANT = 2.155166667;
    public static final double ROTATION_I_CONSTANT = 0;
    public static final double ROTATION_D_CONSTANT = 0.087583333;

    // These constants are for calculating MOI - there is no use in
    // the code for them
    public static final double TRANSLATION_KA = 0.26435;
    public static final double ANGULAR_KA = 0.20065;
    public static final double MOI_TRACK_WIDTH = Math.max(SwerveSubsystemConstants.TRACK_WIDTH, SwerveSubsystemConstants.WHEEL_BASE);
    public static final double ROBOT_WEIGHT = 56;   // This value may not be correct - this is a rough estimate
  
    public static final double ESTIMATED_MOI = ROBOT_WEIGHT * (MOI_TRACK_WIDTH / 2) * (ANGULAR_KA / TRANSLATION_KA);    // Its 14.34569699
  }
}
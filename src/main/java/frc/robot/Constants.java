// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final int BALL_COLLECT_BUTTON = XboxController.Button.kX.value;
    public static final int MOVE_BALL_COLLECTOR_BUTTON = XboxController.Button.kA.value;
  }

  public static class BallCollectorConstants{
    public static final int BALL_COLLECTOR_MOTOR_PORT = 1;
    public static final int BALL_COLLECTOR_ARM_MOTOR_PORT = 2;
    public static final int LIMIT_SWITCH_PORT = 3;
    public static final double BALL_COLLECTOR_MOTOR_START_SPEED = 1.0;
    public static final int BALL_COLLECTOR_MOTOR_FINISH_SPEED  = 0;
    public static final double BALL_COLLECTOR_MOTOR_ARM_START_POSITION = 1.0;
    public static final double BALL_COLLECTOR_MOTOR_ARM_FINISH_POSITION = 1.0;
    public static final double ENCODER_POSITION_FACTOR = 360;           // Degrees
    public static final double ARM_ENCODER_VELOCITY_FACTOR = 12.0;    // Degrees per seconds. TODO: Need to calibrate speed


    public static final int COLLECTOR_SMART_CURRENT_LIMIT = 20;
    public static final double COLLECTOR_ENCODER_VELOCITY_FACTOR = 12.0;
    public static final boolean ENCODER_INVERTED = false; // TODO: To check if encoder is inverted
    public static final int ARM_SMART_CURRENT_LIMIT = 20; // Default rev configuration for current limit
    public static final double P_CONSTANT = 1; //TODO: Calibrate PID values
    public static final double I_CONSTANT = 0;
    public static final double D_CONSTANT = 0;

    public static final double MAX_ENCODER_VALUE = 360;  // TODO: Calibrate Max Encoder Value

  }

}

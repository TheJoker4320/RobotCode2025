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
    public static final int kOperatorControllerPort = 0;

    public static final int ballCollectorButtonNumber = XboxController.Button.kX.value;
    public static final int MoveBallCollectorButtonNumber = XboxController.Button.kA.value;
  }

  public static class CollectorMotorPorts {
    public static final int motorNeoPort = 1;
    public static final int motorNeo550Port = 2;
    public static final int limitSwitchPort = 3;

  }

  public static final class BallCollectorPID {

    public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI);           // Radians
    public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0;    // Radians per seconds

    public static final boolean TURNING_ENCODER_INVERTED = true;
    public static final int TURNING_SMART_CURRENT_LIMIT = 20;
    public static final double TURNING_P_CONSTANT = 1;
    public static final double TURNING_I_CONSTANT = 0;
    public static final double TURNING_D_CONSTANT = 0;

    public static final double MAX_TURNING_ENCODER_VALUE = 2.0 * Math.PI;
  }
  

}

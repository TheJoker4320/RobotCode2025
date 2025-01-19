// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }
  public static class ArmConstants{
    public static int ENCODER_CHANNEL = 0;
    public static int MOTOR_ID = 0;

    public static double ARM_KP = 0;
    public static double ARM_KI = 0;
    public static double ARM_KD = 0;
    public static double ARM_KG = 0; 
    public static double ARM_KV = 0;
    public static double ARM_KA = 0;

    public static double ARM_LOW_ANGLE = 0;
    public static double ARM_HIGH_ANGLE = 90;

    public static double ARM_POSITION_TOLERANCE = 0;
    public static double ARM_ENCODER_TOLERANCE = 0;

    public static double ENCODER_TO_ARM_GEAR_RATIO = 0;
    public static double CONVERSION_RATE = ENCODER_TO_ARM_GEAR_RATIO * 2*Math.PI;

    public static boolean IS_MAGIC_MOTION_ENABLED = false;

    public static double MM_CRUISE_VELOCITY = 0;
    public static double MM_ACCELERATION = 0;
    public static double MM_JERK = 0;
    //TODO: set all constants
  }
}

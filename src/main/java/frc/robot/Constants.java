// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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

  public static class SwerveSubsystemConstants {
    public static final double TRACK_WIDTH = 0.675;                 // distance between centers of right and left modules TODO: Validate measurment
    public static final double WHEEL_BASE = 0.675;                  // distance between centers of front and rear modules TODO: Validate measurment
  
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
}

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

    public static final int ELEVATOR_LOW_STATE = XboxController.Button.kX.value;
    public static final int ELEVATOR_HIGH_STATE = XboxController.Button.kY.value;
  }

  public static class ElevatorConstants {
    public static final int ENCODER_CHANNEL = 1;                    // TODO: Validate encoder channel
    public static final int RIGHT_MOTOR_DEVICE_ID = 1;              // TODO: Validate device id
    public static final int LEFT_MOTOR_DEVICE_ID = 2;               // TODO: Validate device id

    public static final boolean LEFT_OPPOSITE_OF_RIGHT = false;     // TODO: Validate this value

    public static final double ELEVATOR_ENCODER_TOLERANCE = 0.1;
    public static final double ELEVATOR_POSITION_TOLERANCE = 0.1;

    // Notice that all the values in regard to positions/velocity/encoder/pid and so on are all based on rotations not meters
    // so if we want a height of 1 meter, instead of 1 we convert it to rotations and put that new value
    public static final double LOW_POSITION_HEIGHT = 50;            // Example values
    public static final double HIGH_POSITION_HEIGHT = 100;          // Example values

    public static final double ELEVATOR_P_CONSTANT = 1;             // TODO: Validate this value
    public static final double ELEVATOR_I_CONSTANT = 0;             // TODO: Validate this value
    public static final double ELEVATOR_D_CONSTANT = 0;             // TODO: Validate this value
    public static final double ELEVATOR_G_CONSTANT = 0;             // TODO: Calculate this value from the site ReCalc
    public static final double ELEVATOR_V_CONSTANT = 0;             // TODO: Calculate this value from the site ReCalc
    public static final double ELEVATOR_A_CONSTANT = 0;             // TODO: Calculate this value from the site ReCalc

    public static final double MM_CRUISE_VELOCITY = 1;              // TODO: Calculate this value from the site ReCalc
    public static final double MM_ACCELERATION = 10;                // TODO: Calculate this value from the site ReCalc
    public static final double MM_JERK = 100;                       // This value is optional, TODO: Calculate this value from the site ReCalc

    public static final boolean MOTIONMAGIC_ENABLED = false;
  }
}

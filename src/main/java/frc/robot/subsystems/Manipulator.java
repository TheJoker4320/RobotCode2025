// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//This is the ArmCollectors subsystem. It contains the methods to control the arm collectors.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class Manipulator extends SubsystemBase {
  private final SparkMax mCoralMotor;
  private final SparkMax mBallMotor;
  private final DigitalInput mBallSwitch;
  private final DigitalInput mCoralSwitch;
  private static Manipulator mInstance;

  public static Manipulator getInstance() {
    if (mInstance == null) {
      mInstance = new Manipulator();
    }
    return mInstance;
  }

  private Manipulator() {
      mCoralMotor = new SparkMax(ManipulatorConstants.CORAL_MOTOR_ID, MotorType.kBrushless);
      mBallMotor = new SparkMax(ManipulatorConstants.BALL_MOTOR_ID, MotorType.kBrushless);
      mBallSwitch = new DigitalInput(ManipulatorConstants.BALL_SWITCH_PORT);
      mCoralSwitch = new DigitalInput(ManipulatorConstants.CORAL_SWITCH_PORT);
  }

  public boolean getBallSwitchState() {
    return mBallSwitch.get();
  }

  public boolean getCoralSwitchState() {
    return mCoralSwitch.get();
  }

/**
 * This method sets the speed of the ball collector motor.
 * @param isForward - true if the motor should move forward, false if the motor should move backward.
 */
  public void setBallSpeed(Boolean isForward) {
    if (isForward) {
      mBallMotor.set(ManipulatorConstants.BALL_COLLECTOR_SPEED);
    } else {
      mBallMotor.set(-ManipulatorConstants.BALL_COLLECTOR_SPEED);
    }
  }

/**
 * This method sets the speed of the coral collector motor.
 * @param isForward - true if the motor should move forward, false if the motor should move backward.
 */
  public void setCoralSpeed(Boolean isForward) {
    if (isForward) {
      mCoralMotor.set(ManipulatorConstants.CORAL_COLLECTOR_SPEED);
    } else {
      mCoralMotor.set(-ManipulatorConstants.CORAL_COLLECTOR_SPEED);
    }
  }


  public void stopBallCollector() {
    mBallMotor.set(0);
  }

  public void stopCoralCollector() {
    mCoralMotor.set(0);
  }


  @Override
  public void periodic() {}
}
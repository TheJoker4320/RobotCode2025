// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//This is the ArmCollectors subsystem. It contains the methods to control the arm collectors.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmCollectorConstants;

public class ArmCollectors extends SubsystemBase {
  private final SparkMax mCoralMotor;
  private final SparkMax mBallMotor;
  private final DigitalInput mLimitSwitch;
  private static ArmCollectors mInstance;

  public static ArmCollectors getInstance() {
    if (mInstance == null) {
      mInstance = new ArmCollectors();
    }
    return mInstance;
  }

  private ArmCollectors() {
      mCoralMotor = new SparkMax(ArmCollectorConstants.CORAL_MOTOR_ID, MotorType.kBrushless);
      mBallMotor = new SparkMax(ArmCollectorConstants.BALL_MOTOR_ID, MotorType.kBrushless);
      mLimitSwitch = new DigitalInput(ArmCollectorConstants.LIMIT_SWITCH_PORT);
  }

  public boolean getLimitSwitchState() {
    return mLimitSwitch.get();
  }

/**
 * This method sets the speed of the ball collector motor.
 * @param isForward - true if the motor should move forward, false if the motor should move backward.
 */
  public void setBallSpeed(Boolean isForward) {
    if (isForward) {
      mBallMotor.set(ArmCollectorConstants.BALL_COLLECTOR_SPEED);
    } else {
      mBallMotor.set(-ArmCollectorConstants.BALL_COLLECTOR_SPEED);
    }
  }

/**
 * This method sets the speed of the coral collector motor.
 * @param isForward - true if the motor should move forward, false if the motor should move backward.
 */
  public void setCoralSpeed(Boolean isForward) {
    if (isForward) {
      mCoralMotor.set(ArmCollectorConstants.CORAL_COLLECTOR_SPEED);
    } else {
      mCoralMotor.set(-ArmCollectorConstants.CORAL_COLLECTOR_SPEED);
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
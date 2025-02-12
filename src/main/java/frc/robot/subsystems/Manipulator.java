// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//This is the ArmCollectors subsystem. It contains the methods to control the arm collectors.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.utils.Configs.ManipulatorConfigs;

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
      mCoralMotor.configure(ManipulatorConfigs.CORAL_COLLECTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      mBallMotor.configure(ManipulatorConfigs.BALL_COLLECTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public boolean getBallSwitchState() {
    return mBallSwitch.get();
  }

  public boolean getCoralSwitchState() {
    return mCoralSwitch.get();
  }

  public void collectBall(){
    mBallMotor.set(ManipulatorConstants.BALL_COLLECT_SPEED);
  }
  public void collectCoral(){
    mCoralMotor.set(ManipulatorConstants.CORAL_COLLECT_SPEED);
  }
  public void ejectBall(){
    mBallMotor.set(ManipulatorConstants.BALL_EJECT_SPEED);
  }
  public void ejectCoral(){
    mCoralMotor.set(ManipulatorConstants.CORAL_EJECT_SPEED);
  }

  public void stopBallCollector() {
    mBallMotor.set(0);
  }

  public void stopCoralCollector() {
    mCoralMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.getBoolean("ball collector state:", getBallSwitchState());
    SmartDashboard.putBoolean("coral collector state", getCoralSwitchState());
  }
}
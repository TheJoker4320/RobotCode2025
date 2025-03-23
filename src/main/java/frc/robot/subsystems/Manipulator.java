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
  private final SparkMax mPrimaryMotor;
  private final SparkMax mSecondaryMotor;
  
  //TODO: add back primary limit switch
  //private final DigitalInput mPrimarySwitch;
  private final DigitalInput mSecondarySwitch;

  private static Manipulator mInstance = null;

  public static Manipulator getInstance() {
    if (mInstance == null) {
      mInstance = new Manipulator();
    }
    return mInstance;
  }

  private Manipulator() {  
    mPrimaryMotor = new SparkMax(ManipulatorConstants.PRIMARY_MOTOR_ID, MotorType.kBrushless);
    mSecondaryMotor = new SparkMax(ManipulatorConstants.SECONDARY_MOTOR_ID, MotorType.kBrushless);

    //mPrimarySwitch = new DigitalInput(ManipulatorConstants.PRIMARY_SWITCH_PORT);
    mSecondarySwitch = new DigitalInput(ManipulatorConstants.SECONDARY_SWITCH_PORT);
      
    mPrimaryMotor.configure(ManipulatorConfigs.CORAL_COLLECTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mSecondaryMotor.configure(ManipulatorConfigs.BALL_COLLECTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public boolean getSwitchState() {
    return (!mSecondarySwitch.get());
  }

  public void collectCoral() {
    mSecondaryMotor.set(ManipulatorConstants.CORAL_COLLECT_SPEED);
    mPrimaryMotor.set(ManipulatorConstants.CORAL_COLLECT_SPEED);
  }

  public void collectBall() {
    mSecondaryMotor.set(ManipulatorConstants.BALL_COLLECT_SPEED);
    mPrimaryMotor.set(ManipulatorConstants.BALL_COLLECT_SPEED);
  }

  public void ejectBall() {
    mSecondaryMotor.set(ManipulatorConstants.BALL_EJECT_SPEED);
    mPrimaryMotor.set(ManipulatorConstants.BALL_EJECT_SPEED);
  }

  public void ejectCoral() {
    mSecondaryMotor.set(ManipulatorConstants.CORAL_EJECT_SPEED);
    mPrimaryMotor.set(ManipulatorConstants.CORAL_EJECT_SPEED);
  }

  public void stop(){
    mSecondaryMotor.set(0);
    mPrimaryMotor.set(0);
  }

  public void stopBall(){
    mSecondaryMotor.set(-0.3);
    mPrimaryMotor.set(-0.3);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("primary state:", mPrimarySwitch.get());
    SmartDashboard.putBoolean("secondary switch state:", mSecondarySwitch.get());
    SmartDashboard.putNumber("current", mPrimaryMotor.getOutputCurrent());
  }
}
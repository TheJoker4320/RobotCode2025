// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallCollectorConstants;
import frc.robot.utils.Configs;

public class BallCollector extends SubsystemBase {
  private final SparkMax mBallCollectorMotor;
  private final SparkMax mBallCollectorArmMotor;
  private final DigitalInput mLimitSwitch;
  private static BallCollector mInstance = null;

  private final SparkClosedLoopController mPIDController;
  private final AbsoluteEncoder mEncoder;

  

  private BallCollector() {
    //configuring 
    mBallCollectorMotor = new SparkMax(BallCollectorConstants.BALL_COLLECTOR_MOTOR_PORT, MotorType.kBrushless);
    mBallCollectorMotor.configure(Configs.BallCollectorConfig.COLLECTOR_CONFIGS, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mBallCollectorArmMotor = new SparkMax(BallCollectorConstants.BALL_COLLECTOR_ARM_MOTOR_PORT, MotorType.kBrushless);
    mBallCollectorArmMotor.configure(Configs.BallCollectorConfig.ARM_CONFIGS, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mLimitSwitch = new DigitalInput(BallCollectorConstants.LIMIT_SWITCH_PORT);
    
    mPIDController = mBallCollectorArmMotor.getClosedLoopController();
    mEncoder = mBallCollectorArmMotor.getAbsoluteEncoder();
    
    
  }

  public static BallCollector getInstance(){
    if(mInstance == null) {
      mInstance = new BallCollector();
    }
    return mInstance;

  }

  public boolean isAtState(double threshold , double setPoint) {
    if (Math.abs(mEncoder.getPosition() - setPoint) < threshold) {
      return true;
    }
    return false;
  }

  public void setSpeedCollectorBall(double speed) {
    mBallCollectorMotor.set(speed);
   
  }

  public void setReference(double position) {
    mPIDController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
  public boolean getLimitSwitch() {
    return mLimitSwitch.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("BallCollectorCollect", getLimitSwitch());
  }

}

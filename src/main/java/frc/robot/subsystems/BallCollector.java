// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallCollectorConstants;
import frc.robot.utils.Configs.BallCollectorConfis;

public class BallCollector extends SubsystemBase {
  /** Creates a new BallCollector. */
  private final SparkMax mArmMotor;
  private final SparkMax mCollectorMotor;

  private final SparkClosedLoopController mArmClosedLoopController;
  private final AbsoluteEncoder mEncoder;

  private static BallCollector instance = null;
  public static BallCollector getInstance() {
    if (instance == null)
      instance = new BallCollector();
    return instance;
  }
  
  private BallCollector() {
    mArmMotor = new SparkMax(BallCollectorConstants.ARM_MOTOR_PORT, MotorType.kBrushless);
    mCollectorMotor = new SparkMax(BallCollectorConstants.COLLECTOR_MOTOR_PORT, MotorType.kBrushless);

    mArmMotor.configure(BallCollectorConfis.ARM_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mArmMotor.configure(BallCollectorConfis.ARM_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    mEncoder = mArmMotor.getAbsoluteEncoder();
    mArmClosedLoopController = mArmMotor.getClosedLoopController();
  }

  public void collect() {
    mCollectorMotor.set(BallCollectorConstants.COLLECT_SPEED);
  }
  public void eject() {
    mCollectorMotor.set(BallCollectorConstants.EJECT_SPEED);
  }
  public void stopCollecting() {
    mCollectorMotor.set(0);
  }

  public void setArmReference(double reference) {
    mArmClosedLoopController.setReference(reference, ControlType.kPosition);
  }
  public void stopArm() {
    mArmMotor.set(0);
  }

  public boolean atPosition(double goal) {
    return (Math.abs(mEncoder.getPosition() - goal) < BallCollectorConstants.POSITION_TOLERANCE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

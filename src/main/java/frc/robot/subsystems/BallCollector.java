// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    mArmMotor = new SparkMax(17, MotorType.kBrushless);
    mCollectorMotor = new SparkMax(16, MotorType.kBrushless);

    // TODO: Add configs here

    mEncoder = mArmMotor.getAbsoluteEncoder();
    mArmClosedLoopController = mArmMotor.getClosedLoopController();
  }

  public void collect() {
    mCollectorMotor.set(-0.6);
  }
  public void eject() {
    mCollectorMotor.set(0.6);
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
    return (Math.abs(mEncoder.getPosition() - goal) < 0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

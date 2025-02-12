// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonFX mMotor;
  private static Climber mInstance;

  public static Climber getInstance() {
    if (mInstance == null)
      mInstance = new Climber();
    return mInstance;
  }

  private Climber() {
    mMotor = new TalonFX(1);
  } 

  public void setSpeed(double speed) {
    mMotor.set(speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

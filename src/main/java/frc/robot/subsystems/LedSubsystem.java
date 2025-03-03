// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {

  private final Spark mRightLedStrip;

  /** Creates a new LedSubsystem. */
  public LedSubsystem() {
    mRightLedStrip = new Spark(0);
  }

  public void setBlue() {
    mRightLedStrip.set(0.83);
  }

  public void setGreen() {
    mRightLedStrip.set(0.77);
  }

  public void setNone() {
    mRightLedStrip.set(0.99);
  }

  @Override
  public void periodic() {}

  public void setLavaPalette() {
    mRightLedStrip.set(-0.35);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LedSubsystemConstants;

public class LedSubsystem extends SubsystemBase {

  private final Spark mRightLedStrip;
  private final Spark mLeftLedStrip;

  private static LedSubsystem instance = null;
  public static LedSubsystem getInstance() {
    if (instance == null)
      instance = new LedSubsystem();
    return instance;
  }

  /** Creates a new LedSubsystem. */
  private LedSubsystem() {
    mRightLedStrip = new Spark(LedSubsystemConstants.RIGHT_LED_STRIP_PORT);
    mLeftLedStrip = new Spark(LedSubsystemConstants.LEFT_LED_STRIP_PORT);
  }

  public void setColor(double colorValue) {
    mRightLedStrip.set(colorValue);
    mLeftLedStrip.set(colorValue);
  }

  public void turnLedOff() {
    mRightLedStrip.set(LedSubsystemConstants.OFF_LED_COLOR);
    mLeftLedStrip.set(LedSubsystemConstants.OFF_LED_COLOR);
  }

  @Override
  public void periodic() {}
}

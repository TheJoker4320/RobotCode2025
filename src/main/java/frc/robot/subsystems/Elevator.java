// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ElevatorState;

public class Elevator extends SubsystemBase {

  private final TalonFX mRightMotorController;
  private final TalonFX mLeftMotorController;

  private final DutyCycleEncoder mElevatorEncoder;

  private double mSetpoint;

  /** Creates a new Elevator. */
  public Elevator() {
    super("Elevator");

    mElevatorEncoder = new DutyCycleEncoder(0);    // TODO: Get channel from constants

    mRightMotorController = new TalonFX(0);         // TODO: Get deviceId from constants
    mRightMotorController.setNeutralMode(NeutralModeValue.Brake);

    mLeftMotorController = new TalonFX(0);          // TODO: Get deviceId from constants
    mLeftMotorController.setControl(new Follower(mRightMotorController.getDeviceID(), false));
    mLeftMotorController.setNeutralMode(NeutralModeValue.Brake);

    syncEncoders();
  }

  public void setSetpoint(ElevatorState setpoint) {
    mSetpoint = setpoint.height();
  }

  private void verifyEncoderSync() {
    double krakenPosition = getCurrentHeight();
    double throughBorePosition = mElevatorEncoder.get();
    
    final double POSITION_TOLERANCE = 0.1;                                                  // TODO: Get value from constants
    if (Math.abs(krakenPosition - throughBorePosition) > POSITION_TOLERANCE) {
        System.out.println("WARNING: Encoder synchronization may be lost!");
        syncEncoders();
    }
  }

    private double getCurrentHeight() {
    return mRightMotorController.getPosition().getValue().magnitude();
  }

  public boolean isAtState(ElevatorState state) {
    final double POSITION_TOLERANCE = 0.1; // Adjust based on your needs
    if (Math.abs(state.height() - getCurrentHeight()) < POSITION_TOLERANCE) // Example threshold
      return true;
    return false; 
  }

  private void syncEncoders() {
    mRightMotorController.setPosition(mElevatorEncoder.get());
  }

  @Override
  public void periodic() {
    final PositionVoltage mRequest = new PositionVoltage (0);
    mRightMotorController.setControl(mRequest.withPosition(mSetpoint));

    verifyEncoderSync();
  }
}

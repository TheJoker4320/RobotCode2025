// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.ElevatorState;
import frc.robot.utils.Configs.ElevatorConfigs;

public class Elevator extends SubsystemBase {

  private final TalonFX mRightMotorController;
  private final TalonFX mLeftMotorController;

  private double mSetpoint;
  private boolean mSetpointInitiallied = false;

  private static Elevator mInstance = null;
  public static Elevator getInstance() {
    if (mInstance == null)
      mInstance = new Elevator();
    return mInstance;
  }

  /** Creates a new Elevator. */
  private Elevator() {
    super("Elevator");
    
    mRightMotorController = new TalonFX(ElevatorConstants.RIGHT_MOTOR_DEVICE_ID);
    mRightMotorController.getConfigurator().apply(ElevatorConfigs.ELEVATOR_TALONFX_CONFIG);
    mRightMotorController.setNeutralMode(NeutralModeValue.Brake);

    mRightMotorController.setPosition(ElevatorConstants.MOTOR_OFFSET);

    mLeftMotorController = new TalonFX(ElevatorConstants.LEFT_MOTOR_DEVICE_ID);
    mLeftMotorController.setControl(new Follower(mRightMotorController.getDeviceID(), ElevatorConstants.LEFT_OPPOSITE_OF_RIGHT));
    mLeftMotorController.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setSetpoint(ElevatorState setpoint) {
    mSetpoint = setpoint.height();
    mSetpointInitiallied = true;
  }

  private double getCurrentHeight() {
    return mRightMotorController.getPosition().getValueAsDouble();
  }

  public boolean isAtState(ElevatorState state) {
    if (Math.abs(state.height() - getCurrentHeight()) < ElevatorConstants.ELEVATOR_POSITION_TOLERANCE) {
      mSetpointInitiallied = false;
      stopMotorInPlace();
      return true;
    }
    return false; 
  }

  public void stopMotorInPlace() {
    mRightMotorController.set(0);
    mRightMotorController.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    if (mSetpointInitiallied) {
      // Note that we are using motion magic
      final MotionMagicVoltage mRequest = new MotionMagicVoltage(0);
      mRightMotorController.setControl(mRequest.withPosition(Rotations.of(mSetpoint)));
    }

    SmartDashboard.putNumber("ELEVATOR: Setpoint", mSetpoint);
    SmartDashboard.putNumber("ELEVATOR: Current position", getCurrentHeight());
    SmartDashboard.putNumber("velocity", mRightMotorController.getVelocity().getValue().in(RotationsPerSecond));
    SmartDashboard.putBoolean("initiallized", mSetpointInitiallied);
  }
}

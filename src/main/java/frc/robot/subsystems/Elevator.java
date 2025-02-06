// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.ElevatorState;
import frc.robot.utils.Configs.ElevatorConfigs;

public class Elevator extends SubsystemBase {

  private final TalonFX mRightMotorController;
  private final TalonFX mLeftMotorController;

  private final DutyCycleEncoder mElevatorEncoder;

  private double mSetpoint;

  private static Elevator mInstance = null;
  public static Elevator getInstance() {
    if (mInstance == null)
      mInstance = new Elevator();
    return mInstance;
  }

  /** Creates a new Elevator. */
  private Elevator() {
    super("Elevator");

    mElevatorEncoder = new DutyCycleEncoder(ElevatorConstants.ENCODER_CHANNEL);

    mRightMotorController = new TalonFX(ElevatorConstants.RIGHT_MOTOR_DEVICE_ID);
    mRightMotorController.getConfigurator().apply(ElevatorConfigs.ELEVATOR_TALONFX_CONFIG);
    mRightMotorController.setNeutralMode(NeutralModeValue.Brake);

    mLeftMotorController = new TalonFX(ElevatorConstants.LEFT_MOTOR_DEVICE_ID);
    mLeftMotorController.setControl(new Follower(mRightMotorController.getDeviceID(), ElevatorConstants.LEFT_OPPOSITE_OF_RIGHT));
    mLeftMotorController.setNeutralMode(NeutralModeValue.Brake);

    syncEncoders();
    DataLogManager.start();
  }

  private double getAbsoluteEncoderValue() {
    // In the current state of the elevator there are multiple gears between the motor and the absolute encoder
    // so in order to get the actual value of the absolute encoder from the point of view of the motor we must
    // multiply it by the ratio
    return (mElevatorEncoder.get() * ElevatorConstants.ABSOLUTE_ENCODER_TO_MOTOR_RATIO);
  }

  public void setSetpoint(ElevatorState setpoint) {
    mSetpoint = setpoint.height();
  }

  private void verifyEncoderSync() {
    double krakenPosition = getCurrentHeight();
    double throughBorePosition = getAbsoluteEncoderValue();
    
    if (Math.abs(krakenPosition - throughBorePosition) > ElevatorConstants.ELEVATOR_ENCODER_TOLERANCE) {
        DataLogManager.log("WARNING: Encoder synchronization may be lost!");
        syncEncoders();
    }
  }

    private double getCurrentHeight() {
    return mRightMotorController.getPosition().getValue().magnitude();
  }

  public boolean isAtState(ElevatorState state) {
    if (Math.abs(state.height() - getCurrentHeight()) < ElevatorConstants.ELEVATOR_POSITION_TOLERANCE)
      return true;
    return false; 
  }

  private void syncEncoders() {
    mRightMotorController.setPosition(getAbsoluteEncoderValue());
  }

  @Override
  public void periodic() {
    if (!ElevatorConstants.MOTIONMAGIC_ENABLED) {
      final PositionVoltage mRequest = new PositionVoltage(0);
      mRightMotorController.setControl(mRequest.withPosition(mSetpoint));
    } else {
      /*
      * This code is for when we want to add motion magic to the elevator
      * notice here that just as it is in the rest of the elevator code the setpoint should be
      * in rotations and not meters as many might think
      */
      final MotionMagicVoltage mRequest = new MotionMagicVoltage(0);
      mRightMotorController.setControl(mRequest.withPosition(mSetpoint));
    }

    // Use this in shuffleboard as a graph to calculate PID values
    SmartDashboard.putNumber("ELEVATOR: Setpoint", mSetpoint);
    SmartDashboard.putNumber("ELEVATOR: Current position", mRightMotorController.getPosition().getValue().magnitude());

    verifyEncoderSync();
  }
}
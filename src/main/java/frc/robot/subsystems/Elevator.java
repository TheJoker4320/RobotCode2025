// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.ElevatorState;
import frc.robot.utils.Configs.ElevatorConfigs;

public class Elevator extends SubsystemBase {

  private final TalonFX mRightMotorController;
  private final TalonFX mLeftMotorController;

  private double mSetpoint;
  private boolean mSetpointInitiallied = false;

  private DoublePublisher mElevatorPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("elevator/elevatorPose").publish();
  private DoublePublisher mElevatorRightOutputPublisher = NetworkTableInstance.getDefault().getDoubleTopic("elevator/elevatorRightOutput").publish();
  private DoublePublisher mElevatorSetpointPublisher = NetworkTableInstance.getDefault().getDoubleTopic("elevator/elevatorSetpoint").publish();
  private DoublePublisher mElevatorLeftOutputPublisher = NetworkTableInstance.getDefault().getDoubleTopic("elevator/elevatorLeftOutput").publish();
  private BooleanPublisher mElevatorSetpointInitializedPublisher = NetworkTableInstance.getDefault().getBooleanTopic("elevator/elevatorSetpointInitialized").publish();


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
    DataLogManager.log("ELEVATOR - SETPOINT INITIALIZED - " + String.valueOf(setpoint.height()));
    mSetpoint = setpoint.height();
    mSetpointInitiallied = true;
  }

  public double getCurrentHeight() {
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
    DataLogManager.log("ELEVATOR - STOPPING IN PLACE");

    mRightMotorController.set(0);
    mSetpointInitiallied = false;
  }

  public boolean isSetpointInitialled(){
    return mSetpointInitiallied;
  }

  @Override
  public void periodic() {
    if (mSetpointInitiallied) {
      // Note that we are using motion magic
      final MotionMagicVoltage mRequest = new MotionMagicVoltage(0);
      mRightMotorController.setControl(mRequest.withPosition(Rotations.of(mSetpoint)));
    }

    mElevatorPositionPublisher.set(mRightMotorController.getPosition().getValueAsDouble());
    mElevatorRightOutputPublisher.set(mRightMotorController.get());
    mElevatorLeftOutputPublisher.set(mLeftMotorController.get());
    mElevatorSetpointPublisher.set(mSetpoint);
    mElevatorSetpointInitializedPublisher.set(mSetpointInitiallied);
  }
}

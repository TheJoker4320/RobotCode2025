// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.ArmState;
import frc.robot.utils.Configs.ArmConfigs;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private TalonFX mMotor;
  private DutyCycleEncoder mEncoder;
  
  private double mSetpoint;
  private ArmState mSetpointState;
  private boolean mSetpointInitiallied;

  private DoubleLogEntry mArmPosLog;
  private DoubleLogEntry mArmAbsolutePoseLog;
  private DoubleLogEntry mArmSetpointLog;
  private BooleanLogEntry mSetpointInitialliedLog;
  private DoubleLogEntry mArmOutputLog;

  private static Arm mInstance;
  public static Arm getInstance() {
    if (mInstance == null)
      mInstance = new Arm();
    return mInstance;
  }

  private Arm() {
    mMotor = new TalonFX(ArmConstants.MOTOR_ID);
    mEncoder = new DutyCycleEncoder(ArmConstants.ENCODER_CHANNEL);
    mMotor.getConfigurator().apply(ArmConfigs.ARM_TALONFX_CONFIG);
    mMotor.setNeutralMode(NeutralModeValue.Brake);
    syncEncoders();

    mSetpointInitiallied = false;
    mSetpointState = null;

    DataLog log = DataLogManager.getLog();
    mArmPosLog = new DoubleLogEntry(log, "/joker/arm/position");
    mArmSetpointLog = new DoubleLogEntry(log, "/joker/arm/setpoint");
    mArmOutputLog = new DoubleLogEntry(log, "/joker/arm/output");
    mSetpointInitialliedLog = new BooleanLogEntry(log, "/joker/arm/setpointInitialized");
    mArmAbsolutePoseLog = new DoubleLogEntry(log, "/joker/arm/absPosition");
  }

  public double getAbsoluteEncoderValue(){
    return Math.IEEEremainder(mEncoder.get() * 360 + ArmConstants.ARM_ENCODER_OFFSET, 360);
  }

  public void setSetpoint(ArmState setpoint) {
    DataLogManager.log("ARM - SETPOINT INITIALIZED - " + String.valueOf(setpoint.angle()));

    mSetpointInitiallied = true;
    mSetpointState = setpoint;
    mSetpoint = setpoint.angle();
  }
  public ArmState setPlaceCoralSetpoint() {
    if (mSetpointState == ArmState.L32) {
      setSetpoint(ArmState.L32_PLACED);
      return ArmState.L32_PLACED;
    }
    else if (mSetpointState == ArmState.L4) {
      setSetpoint(ArmState.L4_PLACED);
      return ArmState.L4_PLACED;
    }
    return null; 
  }

  public double getCurrentAngle() {
    return mMotor.getPosition().getValue().in(Degree);
  }
  public void syncEncoders() { 
    mMotor.setPosition(Degree.of(getAbsoluteEncoderValue()));
  }
  private boolean verifyEncoderSync() {
    if (Math.abs(getCurrentAngle() - getAbsoluteEncoderValue()) > ArmConstants.ARM_ENCODER_TOLERANCE){
      syncEncoders();
      return true;
    }
    return false;
  }

  public boolean isAtState(ArmState state) {
    if (Math.abs(state.angle() - getCurrentAngle()) < ArmConstants.ARM_POSITION_TOLERANCE) {
      mSetpointInitiallied = false;
      return true;
    }
    return false;
  }

  public void stopMotorInPlace() {
    DataLogManager.log("ARM - STOPPING IN PLACE");
    mSetpointInitiallied = false;
    double vol = ArmConstants.ARM_KG_STAY * Math.cos(Degrees.of(getCurrentAngle()).in(Degrees));
    if (vol >= 0 && vol <= ArmConstants.ARM_KG_STAY)
      mMotor.setVoltage(vol);
    else
      mMotor.set(0);
  }

  public ArmState getSetpointState(){
    return mSetpointState;
  }

  public boolean isSetpointInitialled(){
    return mSetpointInitiallied;
  }

  @Override
  public void periodic() {
    if (mSetpointInitiallied) {
      // This method will be called once per scheduler run
      if (!ArmConstants.IS_MAGIC_MOTION_ENABLED) {
        PositionVoltage m_request = new PositionVoltage(0);
        mMotor.setControl(m_request.withPosition(Degree.of(mSetpoint)));
      }
      
      //this code uses motion magic for the arm
      else {  
        MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        mMotor.setControl(m_request.withPosition(Degree.of(mSetpoint)));
      }
    }

    mArmAbsolutePoseLog.append(getAbsoluteEncoderValue());
    mArmSetpointLog.append(mSetpoint);
    mArmPosLog.append(mMotor.getPosition().getValue().in(Degrees));
    mArmOutputLog.append(mMotor.get());
    mSetpointInitialliedLog.append(mSetpointInitiallied);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  
  private static Arm mInstance;
  public static Arm getInstance() {
    if (mInstance == null)
      mInstance = new Arm();
    return mInstance;
  }

  private Arm() {
    SmartDashboard.putBoolean("Reached state", false);
    mMotor = new TalonFX(ArmConstants.MOTOR_ID);
    mEncoder = new DutyCycleEncoder(ArmConstants.ENCODER_CHANNEL);
    mMotor.getConfigurator().apply(ArmConfigs.ARM_TALONFX_CONFIG);
    mMotor.setNeutralMode(NeutralModeValue.Brake);
    syncEncoders();

    mSetpointInitiallied = false;
  }
  public double getAbsoluteEncoderValue(){
    return mEncoder.get() * 360 < 200 ? mEncoder.get() * 360 * ArmConstants.ENCODER_TO_ARM_GEAR_RATIO + ArmConstants.ARM_ENCODER_OFFSET : mEncoder.get() * 360 * ArmConstants.ENCODER_TO_ARM_GEAR_RATIO + ArmConstants.ARM_ENCODER_OFFSET - 360;
  }

  public void setSetpoint(ArmState setpoint) {
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
  private void syncEncoders() { 
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
      SmartDashboard.putBoolean("Reached state", true);
      mSetpointInitiallied = false;
      return true;
    }
    SmartDashboard.putBoolean("Reached state", false);
    return false;
  }

  public void stopMotorInPlace() {
    //TODO: check if arm stays in place
    mSetpointInitiallied = false;
    mMotor.setVoltage(ArmConstants.ARM_KG_STAY * Math.cos(Degrees.of(getCurrentAngle()).in(Radians)));
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

    //sends to smart dashboard if encoders are out of sync
    //Alert encoderDesyncAlert = new Alert("WARNING: ENCODER VALUES ARE OUT OF SYNC", AlertType.kWarning);
    //encoderDesyncAlert.set(verifyEncoderSync()); //TODO: check if SmartDashBoard puts this value
    
    //if (mSetpointInitiallied && isAtState(mSetpointState))
    //  mSetpointInitiallied = false;
    verifyEncoderSync();
  }
}

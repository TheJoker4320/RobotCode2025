// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.ArmState;
import frc.robot.utils.Configs.ArmConfigs;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private TalonFX mMotor;
  private DutyCycleEncoder mEncoder;
  
  private double mSetpoint;
  
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
  }
  public double getAbsoluteEncoderValue(){
    return mEncoder.get() * 360 * ArmConstants.ENCODER_TO_ARM_GEAR_RATIO + ArmConstants.ARM_ENCODER_OFFSET;
  }
  public void setSetpoint(ArmState setpoint) {
    mSetpoint = setpoint.angle();
  }
  private double getCurrentAngle() {
    return mMotor.getPosition().getValueAsDouble();
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
    if (Math.abs(state.angle() - getCurrentAngle()) < ArmConstants.ARM_POSITION_TOLERANCE)
      return true;
    return false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!ArmConstants.IS_MAGIC_MOTION_ENABLED) {
      PositionVoltage m_request = new PositionVoltage(0);
      mMotor.setControl(m_request.withPosition(Degree.of(mSetpoint)));
    }
    /*
     * this code is for when we want to add magic motion to the arm
     * notice here that just as it is in the rest of the arm code the setpoint should be
     * in degrees and not radians
     */
    else {  
      MotionMagicVoltage m_request = new MotionMagicVoltage(0);
      mMotor.setControl(m_request.withPosition(Degree.of(mSetpoint)));
    }

    //sends to smart dashboard if encoders are out of sync
    Alert encoderDesyncAlert = new Alert("WARNING: ENCODER VALUES ARE OUT OF SYNC", AlertType.kWarning);
    encoderDesyncAlert.set(verifyEncoderSync()); //TODO: check if SmartDashBoard puts this value
  }
}

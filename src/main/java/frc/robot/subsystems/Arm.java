// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.ArmState;
import frc.robot.utils.Configs.ArmConfigs;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private TalonFX m_motor;
  private DutyCycleEncoder m_encoder;
  
  private double m_setpoint;
  
  private Alert m_encoderDesyncAlert = new Alert("WARNING: Encoder values are not in sync!", Alert.AlertType.kWarning);
  private static Arm m_instance;
  public static Arm getInstance() {
    if (m_instance == null) m_instance = new Arm();
    return m_instance;
  }

  private Arm() {
    m_motor = new TalonFX(ArmConstants.MOTOR_ID);
    m_encoder = new DutyCycleEncoder(ArmConstants.ENCODER_CHANNEL);
    m_motor.getConfigurator().apply(ArmConfigs.ARM_TALONFX_CONFIG);
    m_motor.setNeutralMode(NeutralModeValue.Brake);
    syncEncoders();
  }
  public void setSetpoint(ArmState setpoint) {
    m_setpoint = setpoint.angle();
  }
  private double getCurrentAngle() {
    return m_motor.getPosition().getValue().magnitude();
  }
  private void syncEncoders() {
    m_motor.setPosition(m_encoder.get());
  }
  private boolean verifyEncoderSync() {
    if (Math.abs(getCurrentAngle() - m_encoder.get()) > ArmConstants.ARM_ENCODER_TOLERANCE){
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
    if (!ArmConstants.IS_MAGIC_MOTION_ENABLED){
      PositionVoltage m_request = new PositionVoltage(0);
      m_motor.setControl(m_request.withPosition(m_setpoint));
    }
    /*
     * this code is for when we want to add magic motion to the arm
     * notice here that just as it is in the rest of the arm code the setpoint should be
     * in rotations and not radians
     */
    else{  
      MotionMagicVoltage m_request = new MotionMagicVoltage(0);
      m_motor.setControl(m_request.withPosition(m_setpoint));
    }

    //sends to smart dashboard if encoders are out of sync
    m_encoderDesyncAlert.set(verifyEncoderSync()); //TODO: check if SmartDashBoard puts this value
  }
}

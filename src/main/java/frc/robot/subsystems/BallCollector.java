// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CollectorMotorPorts;
import frc.robot.utils.Configs;

public class BallCollector extends SubsystemBase {
  private final SparkMax m_ballCollectorMotor;
  private final SparkMax m_moveBallCollectorMotor;
  private final DigitalInput m_limitSwitch;
  private static BallCollector instance = null;

  private final SparkClosedLoopController m_PIDController;
  

  private BallCollector() {
    //configuring 
    m_ballCollectorMotor = new SparkMax(CollectorMotorPorts.BALL_Collector_MOTOR_MOTOR_PORT , MotorType.kBrushless);
    m_ballCollectorMotor.configure(Configs.BallCollectorConfig.config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_PIDController = m_ballCollectorMotor.getClosedLoopController();

    m_moveBallCollectorMotor = new SparkMax(CollectorMotorPorts.MOVE_BALL_COLLECTOR_MOTOR_PORT, MotorType.kBrushless);
    
    m_limitSwitch = new DigitalInput(CollectorMotorPorts.LIMIT_SWITCH_PORT);
    
  }

  public static BallCollector getInstance(){
    if(instance == null) {
        instance = new BallCollector();
    }
    return instance;

  }

  public void setSpeedCollectorBall(double speed) {
    m_moveBallCollectorMotor.set(speed);
   
  }

  public void setReference(double position) {
    m_PIDController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
  public boolean getLimitSwitch() {
    return m_limitSwitch.get();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}

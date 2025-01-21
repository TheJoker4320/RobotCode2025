// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.CollectorMotorSpeeds;
import frc.robot.subsystems.BallCollector;

import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class MoveBallCollector extends Command {
  private final BallCollector m_ballCollector;
  /**
   * Creates a new EampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveBallCollector(BallCollector ballCollector) {
    m_ballCollector = ballCollector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ballCollector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ballCollector.setReference(CollectorMotorSpeeds.MOTOR_NEO550_START_SPEED);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ballCollector.setReference(CollectorMotorSpeeds.MOTOR_NEO550_FINISH_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }

}

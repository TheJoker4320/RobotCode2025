// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.CollectorMotors;
import frc.robot.subsystems.BallCollector;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class BallCollect extends Command {
  private final BallCollector m_collectorBall;

  /**
   * Creates a new EampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BallCollect(BallCollector collectorBall) {
    m_collectorBall = collectorBall;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_collectorBall);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_collectorBall.setSpeedCollectorBall(CollectorMotors.BALL_Collector_MOTOR_START_SPEED);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_collectorBall.setSpeedCollectorBall(CollectorMotors.BALL_Collector_MOTOR_FINISH_SPEED);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_collectorBall.getLimitSwitch();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.BallCollectorConstants;
import frc.robot.subsystems.BallCollector;

import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class MoveBallCollector extends Command {
  private final BallCollector mBallCollector;
  /**
   * Creates a new EampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveBallCollector(BallCollector ballCollector) {
    mBallCollector = ballCollector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mBallCollector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mBallCollector.setReference(BallCollectorConstants.BALL_COLLECTOR_MOTOR_ARM_START_POSITION);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mBallCollector.setReference(BallCollectorConstants.BALL_COLLECTOR_MOTOR_ARM_FINISH_POSITION);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;

  }

}

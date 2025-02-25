// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BallCollectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BallCollectorConstants;
import frc.robot.subsystems.BallCollector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CloseBallCollector extends Command {
  private final BallCollector mBallCollector;
  /** Creates a new OpenBallCollector. */
  public CloseBallCollector(BallCollector ballCollector) {
    mBallCollector = ballCollector;

    addRequirements(mBallCollector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mBallCollector.setArmReference(BallCollectorConstants.CLOSE_POSITION);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mBallCollector.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mBallCollector.atPosition(BallCollectorConstants.CLOSE_POSITION);
  }
}

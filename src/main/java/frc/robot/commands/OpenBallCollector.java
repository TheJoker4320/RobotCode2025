// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.BallCollectorConstants;
import frc.robot.subsystems.BallCollector;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;


/** An example command that uses an example subsystem. */
public class OpenBallCollector extends Command {
  private final BallCollector mBallCollector;
  private final Timer mTimer;
  /**
   * Creates a new EampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public OpenBallCollector(BallCollector ballCollector) {
    mBallCollector = ballCollector;
    mTimer = new Timer();  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mBallCollector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTimer.start();
    mBallCollector.setReference(BallCollectorConstants.OPEN_POSITION);

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTimer.stop();
    mTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mBallCollector.isAtState(BallCollectorConstants.OPEN_POSITION) || mTimer.get() >= BallCollectorConstants.OPEN_TIME_OUT) {
      return true;
    }
    return false;
  }

}

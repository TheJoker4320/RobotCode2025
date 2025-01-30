// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//This is the ArmCollectBall command. It is used to collect balls with the arm collector.


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmCollectors;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCollectBall extends Command {
  /** Creates a new ArmCollectBall. */
  private ArmCollectors mCollector;
  public ArmCollectBall(ArmCollectors collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mCollector = collector;
    addRequirements(mCollector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mCollector.setBallSpeed(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mCollector.stopBallCollector();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mCollector.getBallSwitchState();
  }
}

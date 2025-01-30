// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//This is the ArmCollectBall command. It is used to collect balls with the arm collector.


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManipulatorCollectBall extends Command {
  /** Creates a new ArmCollectBall. */
  private Manipulator mManipulator;
  public ManipulatorCollectBall(Manipulator manipulator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mManipulator = manipulator;
    addRequirements(mManipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mManipulator.setBallSpeed(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mManipulator.stopBallCollector();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mManipulator.getBallSwitchState();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// This is the command for the coral collector on the arm to collect the coral

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmCollectors;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCollectCoral extends Command {
  /** Creates a new ArmCollectCoral. */
  private ArmCollectors mCollector;
  public ArmCollectCoral(ArmCollectors collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mCollector = collector;
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mCollector.setCoralSpeed(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mCollector.stopCoralCollector();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

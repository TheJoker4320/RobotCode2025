// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// This is the command for the coral collector on the arm to collect the coral

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManipulatorCollectCoral extends Command {
  /** Creates a new ArmCollectCoral. */
  private Manipulator mManipulator;
  private int line;
  public ManipulatorCollectCoral(Manipulator manipulator, int line) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mManipulator = manipulator;
    this.line = line;
    DataLogManager.log("MANIPULATOR_COLLECT_CORAL");
    addRequirements(mManipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log("MANIPULATOR_COLLECT_CORAL INITIALIZED CALLED BY: " + line);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mManipulator.collectCoral();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      DataLogManager.log("MANIPULATOR_COLLECT_CORAL FINISHED - INTERRUPTED CALLED BY: " + line);
    else
      DataLogManager.log("MANIPULATOR_COLLECT_CORAL FINISHED CALLED BY: " + line);
    mManipulator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mManipulator.getSwitchState();
  }
}

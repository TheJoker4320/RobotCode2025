// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.utils.ArmState;
import frc.robot.utils.ElevatorState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManipulatorCoralEjectTeleop extends Command {
  /** Creates a new ManipulatorCoralEjectTeleop. */
  private Elevator mElevator;
  private Arm mArm;
  private Manipulator mManipulator;
  public ManipulatorCoralEjectTeleop(Elevator elevator, Arm arm, Manipulator manipulator) {
    mElevator = elevator;
    mManipulator = manipulator;
    mArm = arm;
    
    addRequirements(mManipulator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!mElevator.isAtState(ElevatorState.L4)) {
      mManipulator.ejectCoral();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mArm.isAtState(ArmState.L4_PLACED)){
      mManipulator.ejectCoral();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mManipulator.stopCoralCollector();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

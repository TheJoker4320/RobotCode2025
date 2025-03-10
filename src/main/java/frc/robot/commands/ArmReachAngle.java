// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.utils.ArmState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmReachAngle extends Command {
  /** Creates a new ArmReachAngle. */
  private Arm mArm;
  private ArmState mDesieredState;
  public ArmReachAngle(Arm arm, ArmState desiredState) {
    mArm = arm;
    mDesieredState = desiredState; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mArm.setSetpoint(mDesieredState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted)
      mArm.stopMotorInPlace();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mArm.isAtState(mDesieredState);
  }
}

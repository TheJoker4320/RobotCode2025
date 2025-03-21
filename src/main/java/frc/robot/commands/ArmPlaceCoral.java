// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
// import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.utils.ArmState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmPlaceCoral extends Command {
  /** Creates a new ArmReachAngle. */
  private Arm mArm;
  private ArmState mDesiredState;
  public ArmPlaceCoral(Arm arm) {
    mArm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mArm);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDesiredState = mArm.setPlaceCoralSetpoint();
    if (mDesiredState != null)
      DataLogManager.log("ARM_PLACE_CORAL_" + mDesiredState.angle() + " INITIALIZED");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted)
      mArm.stopMotorInPlace();

    if (mDesiredState != null)
      if (interrupted)
        DataLogManager.log("ARM_PLACE_CORAL_" + mDesiredState.angle() + " FINISHED INTERRUPTED - REACHED: " + mArm.getCurrentAngle());
      else
        DataLogManager.log("ARM_PLACE_CORAL_" + mDesiredState.angle() + " FINISHED - REACHED: " + mArm.getCurrentAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (mDesiredState == null)
      return true;
    return mArm.isAtState(mDesiredState);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.ElevatorState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorReachL2 extends Command {
  
  private final Timer mTimer;
  private final Elevator mElevator;
  private final Arm mArm;

  private boolean mSetpointInitiallied;
  private final ElevatorState mDesiredState;

  public ElevatorReachL2(Elevator elevator, Arm arm) {
    mElevator = elevator;
    mDesiredState = ElevatorState.L2;
    mArm = arm;

    mTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mElevator);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mSetpointInitiallied = false;
    mTimer.start();
    mTimer.reset();

    DataLogManager.log("ELEVATOR_REACH_STATE_" + mDesiredState.height() + " INITIALIZED");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!mSetpointInitiallied) {
      if (mArm.getCurrentAngle() >= ArmConstants.MIN_ANGLE_L2_HEIGHT) {
        mSetpointInitiallied = true;
        mElevator.setSetpoint(mDesiredState);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mElevator.stopMotorInPlace();
    mTimer.stop();

    if (interrupted)
      DataLogManager.log("ELEVATOR_REACH_STATE_" + mDesiredState.height() + " FINISHED INTERRUPTED - REACHED: " + mElevator.getCurrentHeight());
    else
      DataLogManager.log("ELEVATOR_REACH_STATE_" + mDesiredState.height() + " FINISHED - REACHED: " + mElevator.getCurrentHeight());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (mElevator.isAtState(mDesiredState) || (mTimer.get() > ElevatorConstants.REACHSTATE_TIMEOUT));
  }
}

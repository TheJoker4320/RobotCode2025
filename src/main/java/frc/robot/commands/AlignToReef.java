// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.PoseEstimatorConstants;
import frc.robot.Constants.SwerveSubsystemConstants;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {

  private final Swerve mSwerve;
  private final PoseEstimatorSubsystem mPoseEstimator;
  private final Pose2d mGoal;
  private final HolonomicDriveController mDriveController;

  private final StructPublisher<Pose2d> mGoalPublisher;

  /** Creates a new AlignToReef. */
  public AlignToReef(Swerve swerve, Pose2d goal) {
    mSwerve = swerve;
    mGoal = goal;
    mPoseEstimator = PoseEstimatorSubsystem.getInstance(swerve);

    mDriveController = new HolonomicDriveController(
        new PIDController(
          AutonomousConstants.TRANSLATION_P_CONSTANT, 
          AutonomousConstants.TRANSLATION_I_CONSTANT, 
          AutonomousConstants.TRANSLATION_D_CONSTANT
        ), 
        new PIDController(
          AutonomousConstants.TRANSLATION_P_CONSTANT, 
          AutonomousConstants.TRANSLATION_I_CONSTANT, 
          AutonomousConstants.TRANSLATION_D_CONSTANT
        ), 
        new ProfiledPIDController(
          AutonomousConstants.ROTATION_P_CONSTANT, 
          AutonomousConstants.ROTATION_I_CONSTANT, 
          AutonomousConstants.ROTATION_D_CONSTANT, 
          new Constraints(PoseEstimatorConstants.MAX_ANGULAR_SPEED, PoseEstimatorConstants.MAX_ANGULAR_ACCELERATION)
        )
    );

    mGoalPublisher = NetworkTableInstance.getDefault().getStructTopic("GoalPose", Pose2d.struct).publish();

    SmartDashboard.putNumber("goal x", goal.getX());
    SmartDashboard.putNumber("goal y", goal.getY());
    SmartDashboard.putNumber("goal rotation", goal.getRotation().getDegrees());

    addRequirements(mSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDriveController.setTolerance(new Pose2d(0.01, 0.01, Rotation2d.fromDegrees(1)));
    mGoalPublisher.set(mGoal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = mDriveController.calculate(mPoseEstimator.getPose(), mGoal, 0, mGoal.getRotation());
    mSwerve.setModuleStates(SwerveSubsystemConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Might be needed to put mSwerve.setX() here but im not sure - requires testing
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mDriveController.atReference();
  }
}

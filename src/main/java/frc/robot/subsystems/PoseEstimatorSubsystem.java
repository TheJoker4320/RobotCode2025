// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.PoseEstimatorConstants;
import frc.robot.Constants.SwerveSubsystemConstants;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.commands.AlignToReef;

public class PoseEstimatorSubsystem extends SubsystemBase {  
  private final SwerveDrivePoseEstimator mPoseEstimator;
  private final Swerve mSwerve;

  private double mGyroOffset;   // In degrees

  private final AprilTagFieldLayout mAprilTagFieldLayout;

  private final DoubleEntry mXOffsetTunable;  // TODO: REMOVE THIS
  private final DoubleEntry mYOffsetTunable;  // TODO: REMOVE THIS

  private final StructLogEntry<Pose2d> mPoseLog;
  private final StructArrayLogEntry<Pose3d> mAprilTagsLog1;
  private final StructArrayLogEntry<Pose3d> mAprilTagsLog2;
  
  private static PoseEstimatorSubsystem mInstance = null;
  public static PoseEstimatorSubsystem getInstance(Swerve swerve) {
    if (mInstance == null)
      mInstance = new PoseEstimatorSubsystem(swerve);
    return mInstance;
  }

  /** Creates a new PoseEstimator. */
  private PoseEstimatorSubsystem(Swerve swerve) {
    mSwerve = swerve;
    mPoseEstimator = new SwerveDrivePoseEstimator(
      SwerveSubsystemConstants.DRIVE_KINEMATICS, 
      getRobotRotation(), 
      mSwerve.getModulePositions(), 
      new Pose2d(),
      PoseEstimatorConstants.STATE_STANDARD_DEVIATIONS,
      PoseEstimatorConstants.VISION_STANDARD_DEVIATIONS
    );

    mXOffsetTunable = DogLog.tunable("Alignment/XOffset", PoseEstimatorConstants.FAR_REEF_X_OFFSET);      // TODO: REMOVE THIS
    mYOffsetTunable = DogLog.tunable("Alignment/YOffset", PoseEstimatorConstants.REEF_Y_RIGHT_OFFSET);    // TODO: REMOVE THIS

    mAprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    DataLog log = DataLogManager.getLog();
    mPoseLog = StructLogEntry.create(log, "/joker/robot/pose", Pose2d.struct);
    mAprilTagsLog1 = StructArrayLogEntry.create(log, "/joker/robot/aprilTagsRight", Pose3d.struct);
    mAprilTagsLog2 = StructArrayLogEntry.create(log, "/joker/robot/aprilTagsLeft", Pose3d.struct);
  }

  public void resetGyroOffset(double newOffset) {
    mGyroOffset = newOffset;
  }

  private Rotation2d getRobotRotation() {
    Rotation2d robotRotation = mSwerve.getRotation();
    robotRotation.rotateBy(Rotation2d.fromDegrees(mGyroOffset));

    return robotRotation;
  }

  public Pose2d getPose() {
    return mPoseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    mPoseEstimator.resetPosition(
      getRobotRotation(),
      mSwerve.getModulePositions(),
      pose
    );
  }

  public void updateVisionMeasurement(String cameraName, StructArrayLogEntry<Pose3d> aprilTagLogEntry) {
    LimelightHelpers.SetRobotOrientation(
      cameraName, 
      mSwerve.getRotation().getDegrees() + mGyroOffset, 
      0, 
      0, 
      0, 
      0, 
      0
    );

    LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    boolean rejectVisionUpdate = false;

    Pose3d[] aprilTags = null;
    // If the angular velocity is over 720, the estimated pose is going to be very innacurate so we "reject" the vision update
    if (poseEstimate != null) {
      if (Math.abs(mSwerve.getAngularVelocity())  > PoseEstimatorConstants.MAXIMUM_ANGULAR_VELOCITY) {
        rejectVisionUpdate = true;
      }
      if (poseEstimate.tagCount == 0) {
        rejectVisionUpdate = true;
      }

      if (!rejectVisionUpdate) {
        mPoseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
      }

      aprilTags = new Pose3d[poseEstimate.rawFiducials.length];
      for (int i = 0; i < poseEstimate.rawFiducials.length; i++)
        aprilTags[i] = mAprilTagFieldLayout.getTagPose(poseEstimate.rawFiducials[i].id).get();
    } else {
      aprilTags = new Pose3d[0];
    }

    aprilTagLogEntry.append(aprilTags);
  }

  @Override
  public void periodic() {
    updateVisionMeasurement("limelight-right", mAprilTagsLog1);
    updateVisionMeasurement("limelight-left", mAprilTagsLog2);

    mPoseEstimator.update(getRobotRotation(), mSwerve.getModulePositions());

    mPoseLog.append(mPoseEstimator.getEstimatedPosition());
  }

  public Pose2d getReefToAlign(final double xOffset, final double yOffset) {
    Pose2d robotPose = mPoseEstimator.getEstimatedPosition();
    List<Pose2d> mReefAprilTags = new ArrayList<Pose2d>(PoseEstimatorConstants.REEF_APRIL_TAG_POSITIONS.values());
    Pose2d reefToAlign = robotPose.nearest(mReefAprilTags);

    final double angleOffset = PoseEstimatorConstants.APRIL_TAG_ANGLE_OFFSET;
    
    double deltaX = mXOffsetTunable.get() * Math.sin(reefToAlign.getRotation().getRadians() + angleOffset) + mYOffsetTunable.get() * Math.cos(reefToAlign.getRotation().getRadians() + angleOffset);      // TODO: REMOVE THIS
    double deltaY = -mXOffsetTunable.get() * Math.cos(reefToAlign.getRotation().getRadians() + angleOffset) + mYOffsetTunable.get() * Math.sin(reefToAlign.getRotation().getRadians() + angleOffset);     // TODO: REMOVE THIS
    //double deltaX = xOffset * Math.sin(reefToAlign.getRotation().getRadians() + angleOffset) + yOffset * Math.cos(reefToAlign.getRotation().getRadians() + angleOffset);
    //double deltaY = -xOffset * Math.cos(reefToAlign.getRotation().getRadians() + angleOffset) + yOffset * Math.sin(reefToAlign.getRotation().getRadians() + angleOffset);

    return new Pose2d(reefToAlign.getX() + deltaX, reefToAlign.getY() + deltaY, reefToAlign.getRotation().plus(Rotation2d.k180deg));
  }

  private boolean isInTolerance(Pose2d pose, Pose2d goal, Pose2d tolerance) {
    return
      Math.abs(pose.getX() - goal.getX()) < tolerance.getX() &&
      Math.abs(pose.getY() - goal.getY()) < tolerance.getY() &&
      Math.abs(pose.getRotation().minus(goal.getRotation()).getDegrees()) < tolerance.getRotation().getDegrees();
  }

  public Command getReefAlignmentCommand(final double xOffset, final double yOffset, Command ledCommand) {
    return new ParallelDeadlineGroup(
      new DeferredCommand(
        () -> {
          return new AlignToReef(mSwerve, getReefToAlign(xOffset, yOffset));
        },
        Set.of(mSwerve)
      ),
      new SequentialCommandGroup(
        new DeferredCommand(
          () -> {
            Pose2d goal = getReefToAlign(xOffset, yOffset);
            return new WaitUntilCommand(() -> isInTolerance(getPose(), goal, new Pose2d(0.035, 0.035, Rotation2d.fromDegrees(1))));
          }, 
          Set.of()
        ),
        ledCommand
      )
    );
  }
}
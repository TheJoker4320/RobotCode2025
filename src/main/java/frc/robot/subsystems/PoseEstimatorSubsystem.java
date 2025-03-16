// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.PoseEstimatorConstants;
import frc.robot.Constants.SwerveSubsystemConstants;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.commands.AlignToReef;

public class PoseEstimatorSubsystem extends SubsystemBase {
  private final Field2d mField;
  
  private final SwerveDrivePoseEstimator mPoseEstimator;
  private final Swerve mSwerve;

  private double mGyroOffset;   // In degrees

  private final StructPublisher<Pose2d> mPosePublisher;

  private static PoseEstimatorSubsystem mInstance = null;
  public static PoseEstimatorSubsystem getInstance(Swerve swerve) {
    if (mInstance == null)
      mInstance = new PoseEstimatorSubsystem(swerve);
    return mInstance;
  }

  /** Creates a new PoseEstimator. */
  private PoseEstimatorSubsystem(Swerve swerve) {
    mField = new Field2d();
    SmartDashboard.putData(mField);

    mSwerve = swerve;
    mPoseEstimator = new SwerveDrivePoseEstimator(
      SwerveSubsystemConstants.DRIVE_KINEMATICS, 
      getRobotRotation(), 
      mSwerve.getModulePositions(), 
      new Pose2d(),
      PoseEstimatorConstants.STATE_STANDARD_DEVIATIONS,
      PoseEstimatorConstants.VISION_STANDARD_DEVIATIONS
    );
    mPosePublisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();
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

  public void updateVisionMeasurement(String cameraName) {
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
    }
  }

  @Override
  public void periodic() {
    updateVisionMeasurement("limelight2");
    updateVisionMeasurement("limelight2+");

    mPoseEstimator.update(getRobotRotation(), mSwerve.getModulePositions());

    mField.setRobotPose(mPoseEstimator.getEstimatedPosition());

    mPosePublisher.set(mPoseEstimator.getEstimatedPosition());
  }

  public Pose2d getReefToAlign(final double xOffset, final double yOffset) {
    Pose2d robotPose = mPoseEstimator.getEstimatedPosition();
    List<Pose2d> mReefAprilTags = new ArrayList<Pose2d>(PoseEstimatorConstants.REEF_APRIL_TAG_POSITIONS.values());
    Pose2d reefToAlign = robotPose.nearest(mReefAprilTags);

    final double angleOffset = PoseEstimatorConstants.APRIL_TAG_ANGLE_OFFSET;

    double deltaX = xOffset * Math.sin(reefToAlign.getRotation().getRadians() + angleOffset) + yOffset * Math.cos(reefToAlign.getRotation().getRadians() + angleOffset);
    double deltaY = -xOffset * Math.cos(reefToAlign.getRotation().getRadians() + angleOffset) + yOffset * Math.sin(reefToAlign.getRotation().getRadians() + angleOffset);

    return new Pose2d(reefToAlign.getX() + deltaX, reefToAlign.getY() + deltaY, reefToAlign.getRotation().plus(Rotation2d.k180deg));
  }

  public Command getReefAlignmentCommand(final double xOffset, final double yOffset) {
    return new DeferredCommand(
      () -> {
        return new AlignToReef(mSwerve, getReefToAlign(xOffset, yOffset));
      },
      Set.of(mSwerve)
    );
  }
}
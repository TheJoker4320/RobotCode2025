// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.PoseEstimatorConstants;
import frc.robot.Constants.SwerveSubsystemConstants;
import frc.robot.subsystems.Swerve.Swerve;

public class PoseEstimatorSubsystem extends SubsystemBase {

  private final Field2d mField;
  
  private final SwerveDrivePoseEstimator mPoseEstimator;
  private final Swerve mSwerve;

  private final double mGyroOffset;   // In degrees

  private static PoseEstimatorSubsystem mInstance = null;
  public static PoseEstimatorSubsystem getInstance(Swerve swerve, Alliance alliance) {
    if (mInstance == null)
      mInstance = new PoseEstimatorSubsystem(swerve, alliance);
    return mInstance;
  }

  /** Creates a new PoseEstimator. */
  private PoseEstimatorSubsystem(Swerve swerve, Alliance alliance) {
    mField = new Field2d();
    SmartDashboard.putData(mField);

    if (alliance == Alliance.Red)
      mGyroOffset = PoseEstimatorConstants.RED_GYRO_OFFSET;
    else
      mGyroOffset = PoseEstimatorConstants.BLUE_GYRO_OFFSET;

    mSwerve = swerve;
    mPoseEstimator = new SwerveDrivePoseEstimator(
      SwerveSubsystemConstants.DRIVE_KINEMATICS, 
      getRobotRotation(), 
      mSwerve.getModulePositions(), 
      new Pose2d(),
      PoseEstimatorConstants.STATE_STANDARD_DEVIATIONS,
      PoseEstimatorConstants.VISION_STANDARD_DEVIATIONS
    );
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

  @Override
  public void periodic() {
    boolean rejectVisionUpdate = false;
    LimelightHelpers.SetRobotOrientation(
      "limelight", 
      mPoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 
      0, 
      0, 
      0, 
      0, 
      0
    );

    LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

    // If the angular velocity is over 720, the estimated pose is going to be very innacurate so we "reject" the vision update
    if (Math.abs(mSwerve.getAngularVelocity())  > PoseEstimatorConstants.MAXIMUM_ANGULAR_VELOCITY) {
      rejectVisionUpdate = true;
    }
    if (poseEstimate.tagCount == 0) {
      rejectVisionUpdate = true;
    }

    if (!rejectVisionUpdate) {
      SmartDashboard.putNumber("Limelight reported x", poseEstimate.pose.getX());                                       // Specifically for debugging, should be removed later
      SmartDashboard.putNumber("Limelight reported y", poseEstimate.pose.getY());                                       // Specifically for debugging, should be removed later
      SmartDashboard.putNumber("Limelight reported angle", poseEstimate.pose.getRotation().getDegrees());               // Specifically for debugging, should be removed later

      mPoseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
    }

    mPoseEstimator.update(getRobotRotation(), mSwerve.getModulePositions());

    mField.setRobotPose(mPoseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("Estimated x", mPoseEstimator.getEstimatedPosition().getX());                              // Specifically for debugging, should be removed later
    SmartDashboard.putNumber("Estimated y", mPoseEstimator.getEstimatedPosition().getY());                              // Specifically for debugging, should be removed later
    SmartDashboard.putNumber("Estimated angle", mPoseEstimator.getEstimatedPosition().getRotation().getDegrees());      // Specifically for debugging, should be removed later
  }
}

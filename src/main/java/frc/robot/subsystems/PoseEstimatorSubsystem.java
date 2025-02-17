// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.OperatorConstants;
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
      mPoseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
    }

    mPoseEstimator.update(getRobotRotation(), mSwerve.getModulePositions());

    mField.setRobotPose(mPoseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("Estimated x", mPoseEstimator.getEstimatedPosition().getX());                              // Specifically for debugging, should be removed later
    SmartDashboard.putNumber("Estimated y", mPoseEstimator.getEstimatedPosition().getY());                              // Specifically for debugging, should be removed later
    SmartDashboard.putNumber("Estimated angle", mPoseEstimator.getEstimatedPosition().getRotation().getDegrees());      // Specifically for debugging, should be removed later
  
    Pose2d alignTo = getReefToAlignRight();
    SmartDashboard.putNumber("AlignTo x", alignTo.getX());
    SmartDashboard.putNumber("AlignTo y", alignTo.getY());
    SmartDashboard.putNumber("AlignTo angle", alignTo.getRotation().getDegrees());
  }

  public Pose2d getReefToAlignRight() {
    Pose2d robotPose = mPoseEstimator.getEstimatedPosition();
    List<Pose2d> mReefAprilTags = new ArrayList<Pose2d>(PoseEstimatorConstants.REEF_APRIL_TAG_POSITIONS.values());
    Pose2d reefToAlign = robotPose.nearest(mReefAprilTags);

    final double xOffset = PoseEstimatorConstants.REEF_X_OFFSET;
    final double yOffset = PoseEstimatorConstants.REEF_Y_RIGHT_OFFSET;
    final double angleOffset = PoseEstimatorConstants.APRIL_TAG_ANGLE_OFFSET;

    double deltaX = xOffset * Math.sin(reefToAlign.getRotation().getRadians() + angleOffset) + yOffset * Math.cos(reefToAlign.getRotation().getRadians() + angleOffset);
    double deltaY = -xOffset * Math.cos(reefToAlign.getRotation().getRadians() + angleOffset) + yOffset * Math.sin(reefToAlign.getRotation().getRadians() + angleOffset);

    return reefToAlign.plus(new Transform2d(-1 * deltaX, -1 * deltaY, Rotation2d.fromDegrees(180)));
  }

  public Command getAlignRightReef() {
    Pose2d curPose = mPoseEstimator.getEstimatedPosition();
    Pose2d goalPose = getReefToAlignRight();

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(curPose.getX(), curPose.getY(), Rotation2d.fromDegrees(0)),
      new Pose2d(goalPose.getX(), goalPose.getY(), Rotation2d.fromDegrees(0))
    );

    PathConstraints constraints = new PathConstraints(0.5, 1, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

    PathPlannerPath alignmentPath = new PathPlannerPath(
      waypoints,
      constraints,
      null,
      new GoalEndState(0, goalPose.getRotation())
    );
    
    try {
      mSwerve.resetHeading(curPose.getRotation().getDegrees() + mGyroOffset);
      mSwerve.resetOdometry(mPoseEstimator.getEstimatedPosition());

      RobotConfig config = null;
      try{
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
      }

      AutoBuilder.configure(
        mPoseEstimator::getEstimatedPosition, 
        mPoseEstimator::resetPose, 
        mSwerve::getRobotRelativeSpeeds, 
        (speeds, feedforwards) -> mSwerve.setModuleStates(SwerveSubsystemConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds)), 
        new PPHolonomicDriveController(
          new PIDConstants(AutonomousConstants.TRANSLATION_P_CONSTANT, AutonomousConstants.TRANSLATION_I_CONSTANT, AutonomousConstants.TRANSLATION_D_CONSTANT), 
          new PIDConstants(AutonomousConstants.ROTATION_P_CONSTANT, AutonomousConstants.ROTATION_I_CONSTANT, AutonomousConstants.ROTATION_D_CONSTANT)
        ), 
        config, 
        () -> { return false; }, 
        mSwerve
      );

      return AutoBuilder.followPath(alignmentPath);
    } catch (Exception e) {
      return new WaitCommand(0);
    }
  }
}

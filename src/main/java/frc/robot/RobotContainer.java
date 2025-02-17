// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveSubsystemConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Swerve.SwerveModuleType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Swerve mSwerveSubsystem = Swerve.getInstance(SwerveModuleType.NEO);
  private PoseEstimatorSubsystem mPoseEstimatorSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    JoystickButton slowSwerveButton = new JoystickButton(m_driverController, OperatorConstants.LOW_SPEED_SWERVE_BUTTON);        // Artificially slows down the robot by multiplying the drivers input (*0.3)
    slowSwerveButton.onTrue(new InstantCommand(() -> mSwerveSubsystem.setInputMultiplier(SwerveSubsystemConstants.SLOW_INPUT_MULTIPLIER), mSwerveSubsystem));
    JoystickButton mediumSwerveButton = new JoystickButton(m_driverController, OperatorConstants.MEDIUM_SPEED_SWERVE_BUTTON);   // Artifically slows down the robot (not too much) by multiplying the drivers input (*0.7)
    mediumSwerveButton.onTrue(new InstantCommand(() -> mSwerveSubsystem.setInputMultiplier(SwerveSubsystemConstants.MEDIUM_INPUT_MULTIPLIER), mSwerveSubsystem));
    JoystickButton regularSwerveButton = new JoystickButton(m_driverController, OperatorConstants.REGULAR_SPEED_SWERVE_BUTTON); // Sets the robots speed to regular (i.e. resets the input multiplier)
    regularSwerveButton.onTrue(new InstantCommand(() -> mSwerveSubsystem.setInputMultiplier(SwerveSubsystemConstants.REGULAR_INPUT_MULTIPLIER), mSwerveSubsystem));

    JoystickButton resetHeadingButton = new JoystickButton(m_driverController, OperatorConstants.RESET_HEADING_SWERVE_BUTTON); // Resets the robot heading - use when the gyro reports incorrect values
    resetHeadingButton.onTrue(new InstantCommand(() -> mSwerveSubsystem.resetHeading(0), mSwerveSubsystem));

    JoystickButton switchReferenceFrameButton = new JoystickButton(m_driverController, OperatorConstants.REFERENCE_FRAME_SWERVE_BUTTON); // Switches between driving field-relative and robot-relative
    switchReferenceFrameButton.onChange(new InstantCommand(() -> mSwerveSubsystem.switchReferenceFrame(), mSwerveSubsystem));

    mSwerveSubsystem.setDefaultCommand(
      new RunCommand(
        () -> mSwerveSubsystem.drive(
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.DRIVE_DEADBAND),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.DRIVE_DEADBAND),
          -MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.DRIVE_DEADBAND)
        ), 
        mSwerveSubsystem
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // When you load here the autonomous command you must firstly reset the poseEstimator and swerve's odometry
    // with the first pose of the autonomous command.
    // use: mPoseEstimator.resetPose(...);
    // use: mSwerveSubsystem.resetOdometry(...);

    // An example command will be run in autonomous
    mPoseEstimatorSubsystem = PoseEstimatorSubsystem.getInstance(mSwerveSubsystem, Alliance.Red); // Make this modulor (get from driver station)
    // For blue: mSwerveSubsystem.resetHeading(autonomous.getStartingHolonomicPose().getRotation().getDegrees());
    // For red: mSwerveSubsystem.resetHeading(autonomous.getStartingHolonomicPose().getRotation().getDegrees() + 180);
    mSwerveSubsystem.resetHeading(180);
    return new WaitCommand(0);
  }
}

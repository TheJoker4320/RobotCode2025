// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutonomousConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPlaceCoral;
import frc.robot.commands.ManipulatorCollectBall;
import frc.robot.commands.ManipulatorCollectCoral;
import frc.robot.commands.ManipulatorBallEject;
import frc.robot.commands.ManipulatorCoralEject;
import frc.robot.subsystems.Manipulator;
import frc.robot.commands.ArmReachAngle;
import frc.robot.commands.ElevatorReachL2;
import frc.robot.subsystems.Arm;
import frc.robot.utils.ArmState;
import frc.robot.commands.ElevatorReachState;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.ElevatorState;
import frc.robot.Constants.SwerveSubsystemConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Swerve.SwerveModuleType;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  /**
   * This class is where the bulk of the robot should be declared. Since Command-based is a
   * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
   * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
   * subsystems, commands, and trigger mappings) should be declared here.
   */
  private final Manipulator mManipulator = Manipulator.getInstance();
  // The robot's subsystems and commands are defined here...
  private Arm mArm = Arm.getInstance();
  private final Swerve mSwerveSubsystem = Swerve.getInstance(SwerveModuleType.NEO);
  private PoseEstimatorSubsystem mPoseEstimatorSubsystem;
  private final Elevator mElevatorSubsystem = Elevator.getInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController = new XboxController(OperatorConstants.DRIVING_CONTROLLER_PORT);
  private final PS4Controller m_operatorController = new PS4Controller(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    mSwerveSubsystem.resetHeading(180);

    RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    
    AutoBuilder.configure(
      mPoseEstimatorSubsystem::getPose, 
      mPoseEstimatorSubsystem::resetPose, 
      mSwerveSubsystem::getRobotRelativeSpeeds, 
      (speeds, feedforwards) -> mSwerveSubsystem.setModuleStates(SwerveSubsystemConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds)), 
      new PPHolonomicDriveController(
        new PIDConstants(AutonomousConstants.TRANSLATION_P_CONSTANT, AutonomousConstants.TRANSLATION_I_CONSTANT, AutonomousConstants.TRANSLATION_D_CONSTANT), 
        new PIDConstants(AutonomousConstants.ROTATION_P_CONSTANT, AutonomousConstants.ROTATION_I_CONSTANT, AutonomousConstants.ROTATION_D_CONSTANT)
      ), 
      config, 
      () -> { return false; }, 
      mSwerveSubsystem
    );

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
    
    // -------------- OPERATOR BUTTONS --------------

    //temporary button of collecting
    Command tempCollect = new ManipulatorCollectCoral(mManipulator);

    // command for preparing to collect a coral
    Command prepareIntakeSequenceCommand = new SequentialCommandGroup(new ElevatorReachState(mElevatorSubsystem, ElevatorState.PRE_INTAKE), new ArmReachAngle(mArm, ArmState.INTAKE));
    // command for collecting a coral
    Command intakeSequenceCommand = new SequentialCommandGroup(new ParallelCommandGroup(new ElevatorReachState(mElevatorSubsystem, ElevatorState.INTAKE), new ManipulatorCollectCoral(mManipulator)), new ElevatorReachState(mElevatorSubsystem, ElevatorState.PRE_INTAKE), new ArmReachAngle(mArm, ArmState.OUT_OF_INTAKE));
    // command for adjusting elevator and arm for l1
    Command l1Command = new ParallelCommandGroup(new ElevatorReachState(mElevatorSubsystem, ElevatorState.L1), new ArmReachAngle(mArm, ArmState.L1));
    // command for adjusting elevator and arm for l2
    //Command l2Command = new ParallelCommandGroup(new ElevatorReachL2(mElevatorSubsystem, mArm), new ArmReachAngle(mArm, ArmState.L32));
    // command for adjusting elevator and arm for l3
    Command l3Command = new ParallelCommandGroup(new ElevatorReachState(mElevatorSubsystem, ElevatorState.L3), new ArmReachAngle(mArm, ArmState.L32));
    // command for adjusting elevator and arm for l4
    Command l4Command = new ParallelCommandGroup(new ElevatorReachState(mElevatorSubsystem, ElevatorState.L4), new ArmReachAngle(mArm, ArmState.L4));
    // command for ejecting a coral
    Command placeCoralCommand = new ParallelCommandGroup(new ArmPlaceCoral(mArm), new ManipulatorCoralEject(mManipulator));
    // command for reaching a ball placed on l2
    Command reachL2BallCommand = new ParallelCommandGroup(new ElevatorReachL2(mElevatorSubsystem, mArm), new ArmReachAngle(mArm, ArmState.L32_BALL));
    // command for reaching a ball placed on l3
    Command reachL3BallCommand = new ParallelRaceGroup(new ElevatorReachState(mElevatorSubsystem, ElevatorState.L3_BALL), new ArmReachAngle(mArm, ArmState.L32_BALL));
    // command for collecting a ball
    Command collectBallCommand = new ManipulatorCollectBall(mManipulator);

    JoystickButton intakePrepareButton = new JoystickButton(m_operatorController, OperatorConstants.INTAKE_PREPARE_BUTTON);
    JoystickButton intakeButton = new JoystickButton(m_operatorController, OperatorConstants.INTAKE_BUTTON);
    JoystickButton l1Button = new JoystickButton(m_operatorController, OperatorConstants.L1_STATE_BUTTON);
    //JoystickButton l2Button = new JoystickButton(m_operatorController, OperatorConstants.L2_STATE_BUTTON);
    JoystickButton l3Button = new JoystickButton(m_operatorController, OperatorConstants.L3_STATE_BUTTON);
    JoystickButton l4Button = new JoystickButton(m_operatorController, OperatorConstants.L4_STATE_BUTTON);
    JoystickButton placeCoralButton = new JoystickButton(m_operatorController, OperatorConstants.PLACE_CORAL_BUTTON);
    JoystickButton reachL2BallButton = new JoystickButton(m_operatorController, OperatorConstants.L2_BALL_STATE_BUTTON); //TODO: check constants before running
    JoystickButton reachL3BallButton = new JoystickButton(m_operatorController, OperatorConstants.L3_BALL_STATE_BUTTON); //TODO: check constants before running
    JoystickButton collectBallButton = new JoystickButton(m_operatorController, OperatorConstants.COLLECT_BALL_BUTTON); //TODO: check constants before running


    intakePrepareButton.onTrue(prepareIntakeSequenceCommand);
    intakeButton.onTrue(intakeSequenceCommand);
    l1Button.onTrue(l1Command);
    //l2Button.onTrue(l2Command);
    l3Button.onTrue(l3Command);
    l4Button.onTrue(l4Command);
    placeCoralButton.toggleOnTrue(placeCoralCommand);
    reachL2BallButton.onTrue(reachL2BallCommand);
    reachL3BallButton.onTrue(reachL3BallCommand);
    collectBallButton.onTrue(collectBallCommand);

    NamedCommands.registerCommand("release", new ManipulatorCoralEject(mManipulator));
    NamedCommands.registerCommand("armPlaceCoral", new ArmPlaceCoral(mArm));
    NamedCommands.registerCommand("reachL4", l4Command);
    NamedCommands.registerCommand("prepareIntake", prepareIntakeSequenceCommand);

    // -------------- DRIVER BUTTONS -------------

    // Swerve buttons
    JoystickButton slowSwerveButton = new JoystickButton(m_driverController, OperatorConstants.LOW_SPEED_SWERVE_BUTTON);        // Artificially slows down the robot by multiplying the drivers input (*0.3)
    slowSwerveButton.onTrue(new InstantCommand(() -> mSwerveSubsystem.setInputMultiplier(SwerveSubsystemConstants.SLOW_INPUT_MULTIPLIER), mSwerveSubsystem));
    JoystickButton mediumSwerveButton = new JoystickButton(m_driverController, OperatorConstants.MEDIUM_SPEED_SWERVE_BUTTON);   // Artifically slows down the robot (not too much) by multiplying the drivers input (*0.7)
    mediumSwerveButton.onTrue(new InstantCommand(() -> mSwerveSubsystem.setInputMultiplier(SwerveSubsystemConstants.MEDIUM_INPUT_MULTIPLIER), mSwerveSubsystem));
    JoystickButton regularSwerveButton = new JoystickButton(m_driverController, OperatorConstants.REGULAR_SPEED_SWERVE_BUTTON); // Sets the robots speed to regular (i.e. resets the input multiplier)
    regularSwerveButton.onTrue(new InstantCommand(() -> mSwerveSubsystem.setInputMultiplier(SwerveSubsystemConstants.REGULAR_INPUT_MULTIPLIER), mSwerveSubsystem));

    JoystickButton resetHeadingButton = new JoystickButton(m_driverController, OperatorConstants.RESET_HEADING_SWERVE_BUTTON); // Resets the robot heading - use when the gyro reports incorrect values
    resetHeadingButton.onTrue(new InstantCommand(() -> mSwerveSubsystem.resetHeading(0), mSwerveSubsystem));

    JoystickButton switchReferenceFrameButton = new JoystickButton(m_driverController, OperatorConstants.REFERENCE_FRAME_SWERVE_BUTTON); // Switches between driving field-relative and robot-relative
    switchReferenceFrameButton.onTrue(new InstantCommand(() -> mSwerveSubsystem.switchReferenceFrame(), mSwerveSubsystem));

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
    mPoseEstimatorSubsystem = PoseEstimatorSubsystem.getInstance(mSwerveSubsystem, DriverStation.getAlliance().get());
    // // For blue: mSwerveSubsystem.resetHeading(autonomous.getStartingHolonomicPose().getRotation().getDegrees());
    // // For red: mSwerveSubsystem.resetHeading(autonomous.getStartingHolonomicPose().getRotation().getDegrees() + 180);

    Trigger rightCloseAlignTrigger = new Trigger(() -> { return m_driverController.getRightTriggerAxis() > OperatorConstants.DRIVE_DEADBAND; } );
    rightCloseAlignTrigger.whileTrue(mPoseEstimatorSubsystem.alignToCloseRightReef());
    Trigger leftCloseAlignTrigger = new Trigger(() -> { return m_driverController.getLeftTriggerAxis() > OperatorConstants.DRIVE_DEADBAND; } );
    leftCloseAlignTrigger.whileTrue(mPoseEstimatorSubsystem.alignToCloseLeftReef());

    JoystickButton rightFarAlignButton = new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value);
    rightFarAlignButton.whileTrue(mPoseEstimatorSubsystem.alignToFarRightReef());
    JoystickButton leftFarAlignButton = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);
    leftFarAlignButton.whileTrue(mPoseEstimatorSubsystem.alignToFarLeftReef());
    
    // PathPlannerPath path;
    // try {
    //   path = PathPlannerPath.fromPathFile("1MeterPath");
    //   mSwerveSubsystem.resetOdometry(path.getStartingHolonomicPose().get());
    //   mSwerveSubsystem.resetHeading(path.getStartingHolonomicPose().get().getRotation().getDegrees());
    //   return AutoBuilder.followPath(path);
    // } catch (Exception e) {
    //   return null;
    // }
    PathPlannerAuto auto = new PathPlannerAuto("testAuto");
    mPoseEstimatorSubsystem.resetPose(auto.getStartingPose());

    double degreeOffset = DriverStation.getAlliance().get().equals(Alliance.Blue) ? 0 : 180;
    mSwerveSubsystem.resetHeading(auto.getStartingPose().getRotation().getDegrees() + degreeOffset);
    return auto;
  }
}

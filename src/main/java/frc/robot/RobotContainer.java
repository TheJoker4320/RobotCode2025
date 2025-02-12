// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmPlaceCoral;
import frc.robot.commands.ManipulatorCollectBall;
import frc.robot.commands.ManipulatorCollectCoral;
import frc.robot.commands.ManipulatorBallEject;
import frc.robot.commands.ManipulatorCoralEject;
import frc.robot.subsystems.Manipulator;
import frc.robot.commands.ArmReachAngle;
import frc.robot.subsystems.Arm;
import frc.robot.utils.ArmState;
import frc.robot.commands.ElevatorReachState;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.ElevatorState;
import frc.robot.Constants.SwerveSubsystemConstants;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.subsystems.Swerve.SwerveModuleType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
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
  private final Elevator mElevatorSubsystem = Elevator.getInstance();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController = new XboxController(OperatorConstants.DRIVING_CONTROLLER_PORT);
  private final PS4Controller m_operatorController = new PS4Controller(OperatorConstants.OPERATOR_CONTROLLER_PORT);

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
    
    // -------------- OPERATOR BUTTONS --------------

    Command prepareIntakeSequenceCommand = new SequentialCommandGroup(new ElevatorReachState(mElevatorSubsystem, ElevatorState.PRE_INTAKE), new ArmReachAngle(mArm, ArmState.INTAKE));
    Command intakeSequenceCommand = new SequentialCommandGroup(new ParallelRaceGroup(new ElevatorReachState(mElevatorSubsystem, ElevatorState.INTAKE), new ManipulatorCollectCoral(mManipulator)), new ParallelCommandGroup(new ElevatorReachState(mElevatorSubsystem, ElevatorState.L2), new ArmReachAngle(mArm, ArmState.L32)));
    Command l1Command = new ParallelCommandGroup(new ElevatorReachState(mElevatorSubsystem, ElevatorState.L1), new ArmReachAngle(mArm, ArmState.L1));
    Command l2Command = new ParallelCommandGroup(new ElevatorReachState(mElevatorSubsystem, ElevatorState.L2), new ArmReachAngle(mArm, ArmState.L32));
    Command l3Command = new ParallelCommandGroup(new ElevatorReachState(mElevatorSubsystem, ElevatorState.L3), new ArmReachAngle(mArm, ArmState.L32));
    Command l4Command = new ParallelCommandGroup(new ElevatorReachState(mElevatorSubsystem, ElevatorState.L4), new ArmReachAngle(mArm, ArmState.L4));
    Command placeCoralCommand = new ParallelCommandGroup(new ArmPlaceCoral(mArm), new ManipulatorCoralEject(mManipulator));

    JoystickButton intakePrepareButton = new JoystickButton(m_operatorController, OperatorConstants.INTAKE_PREPARE_BUTTON);
    JoystickButton intakeButton = new JoystickButton(m_operatorController, OperatorConstants.INTAKE_BUTTON);
    JoystickButton l1Button = new JoystickButton(m_operatorController, OperatorConstants.L1_STATE_BUTTON);
    JoystickButton l2Button = new JoystickButton(m_operatorController, OperatorConstants.L2_STATE_BUTTON);
    JoystickButton l3Button = new JoystickButton(m_operatorController, OperatorConstants.L3_STATE_BUTTON);
    JoystickButton l4Button = new JoystickButton(m_operatorController, OperatorConstants.L4_STATE_BUTTON);
    JoystickButton placeCoralButton = new JoystickButton(m_operatorController, OperatorConstants.PLACE_CORAL_BUTTON);

    intakePrepareButton.onTrue(prepareIntakeSequenceCommand);
    intakeButton.onTrue(intakeSequenceCommand);
    l1Button.onTrue(l1Command);
    l2Button.onTrue(l2Command);
    l3Button.onTrue(l3Command);
    l4Button.onTrue(l4Command);
    placeCoralButton.whileTrue(placeCoralCommand);

    // -------------- DRIVER BUTTONS -------------

    // Swerve buttons
    JoystickButton slowSwerveButton = new JoystickButton(m_driverController, OperatorConstants.LOW_SPEED_SWERVE_BUTTON);        // Artificially slows down the robot by multiplying the drivers input (*0.3)
    slowSwerveButton.onTrue(new InstantCommand(() -> mSwerveSubsystem.setInputMultiplier(SwerveSubsystemConstants.SLOW_INPUT_MULTIPLIER), mSwerveSubsystem));
    JoystickButton mediumSwerveButton = new JoystickButton(m_driverController, OperatorConstants.MEDIUM_SPEED_SWERVE_BUTTON);   // Artifically slows down the robot (not too much) by multiplying the drivers input (*0.7)
    mediumSwerveButton.onTrue(new InstantCommand(() -> mSwerveSubsystem.setInputMultiplier(SwerveSubsystemConstants.MEDIUM_INPUT_MULTIPLIER), mSwerveSubsystem));
    JoystickButton regularSwerveButton = new JoystickButton(m_driverController, OperatorConstants.REGULAR_SPEED_SWERVE_BUTTON); // Sets the robots speed to regular (i.e. resets the input multiplier)
    regularSwerveButton.onTrue(new InstantCommand(() -> mSwerveSubsystem.setInputMultiplier(SwerveSubsystemConstants.REGULAR_INPUT_MULTIPLIER), mSwerveSubsystem));

    JoystickButton resetHeadingButton = new JoystickButton(m_driverController, OperatorConstants.RESET_HEADING_SWERVE_BUTTON); // Resets the robot heading - use when the gyro reports incorrect values
    resetHeadingButton.onTrue(new InstantCommand(() -> mSwerveSubsystem.zeroHeading(), mSwerveSubsystem));

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
    // An example command will be run in autonomous
    return new WaitCommand(0);
  }
}

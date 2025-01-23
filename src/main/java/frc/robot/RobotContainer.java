// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmCollectBall;
import frc.robot.commands.ArmCollectCoral;
import frc.robot.commands.BallEject;
import frc.robot.commands.CoralEject;
import frc.robot.subsystems.ArmCollectors;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final ArmCollectors m_armCollectors = ArmCollectors.getInstance();
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

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
    //button to collect balls 
    JoystickButton armCollectBallButton = new JoystickButton(m_driverController, OperatorConstants.ARM_COLLECT_BALL_BUTTON);
    armCollectBallButton.toggleOnTrue(new ArmCollectBall(m_armCollectors));
    //button to collect coral 
    JoystickButton armCollectCoralButton = new JoystickButton(m_driverController, OperatorConstants.ARM_COLLECT_CORAL_BUTTON);
    armCollectCoralButton.whileTrue(new ArmCollectCoral(m_armCollectors));

    //button to eject balls 
    JoystickButton armEjectBallButton = new JoystickButton(m_driverController, OperatorConstants.ARM_EJECT_BALL_BUTTON);
    armEjectBallButton.whileTrue(new BallEject(m_armCollectors));
    //button to eject coral
    JoystickButton armEjectCoralButton = new JoystickButton(m_driverController, OperatorConstants.ARM_EJECT_CORAL_BUTTON);
    armEjectCoralButton.whileTrue(new CoralEject(m_armCollectors));

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

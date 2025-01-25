// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BallCollect;
import frc.robot.commands.CloseBallCollector;
import frc.robot.commands.MoveBallCollector;
import frc.robot.subsystems.BallCollector;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final BallCollector mBallCollector = BallCollector.getInstance();

  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);

   
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the trigger bindings
    configureBindings();
  }
  
  private void configureBindings() {

    //Button for turn on the Ball Collector
    JoystickButton BallcollectorButton = new JoystickButton(m_driverController , OperatorConstants.BALL_COLLECT_BUTTON);
    BallcollectorButton.whileTrue(new BallCollect(mBallCollector));

    //Button for moving the Ball Collector
    JoystickButton MoveBallCollectorButton = new JoystickButton(m_driverController , OperatorConstants.MOVE_BALL_COLLECTOR_BUTTON);
    MoveBallCollectorButton.onTrue(new MoveBallCollector(mBallCollector , 5));

    //Button for closing the Ball Collector
    JoystickButton CloseBallCollectorButton = new JoystickButton(m_driverController, 0);
    CloseBallCollectorButton.onTrue(new CloseBallCollector(mBallCollector, 5));
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

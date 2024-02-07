// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.driveCommands.driveByController;
import frc.robot.commands.driveCommands.positionCommand;
import frc.robot.commands.pneumatics.movePiston;
import frc.robot.commands.pneumatics.stopPiston;
import frc.robot.subsystems.falcon;
import frc.robot.subsystems.pneumatics;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  
  private final XboxController xc = new XboxController(Constants.XC1);

  //private final pneumatics piston = new pneumatics();
  private final falcon m_motor = new falcon();

  /*private final movePiston m_actuate = new movePiston(piston);
  private final stopPiston m_StopPiston = new stopPiston(piston);
  private final positionCommand m_pose = new positionCommand(m_motor, 1000);*/
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_motor.setDefaultCommand(new driveByController(() -> xc.getLeftY(), m_motor));
  // new JoystickButton(xc, Constants.xcA).toggleOnTrue(m_actuate);

   // new JoystickButton(xc, Constants.xcB).onTrue(m_StopPiston);
    
    //new JoystickButton(xc, Constants.xcY).toggleOnTrue(m_pose);
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
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}

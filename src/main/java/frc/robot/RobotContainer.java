/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DashboardPID_Drive;
import frc.robot.commands.DriveJoystick;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.PID_Drive;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  final Drive drive = new Drive();
  final Joystick leftStick = new Joystick(Constants.USB_LEFT_STICK);
  final Joystick rightStick = new Joystick(Constants.USB_RIGHT_STICK);

  final DriveJoystick driveJoysticks = new DriveJoystick(drive, leftStick, rightStick);
  
  final PID_Drive pid_Drive = new PID_Drive(drive, Constants.DriveConstants.DISTANCE,
      Constants.DriveConstants.MARGIN);
  final DashboardPID_Drive dashboard_PID_Drive = new DashboardPID_Drive(drive, Constants.DriveConstants.DISTANCE, Constants.DriveConstants.MARGIN);
 
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
  
    final DriveJoystick driveJoysticks = new DriveJoystick(drive, leftStick, rightStick);
    
    JoystickButton trigger = new JoystickButton(leftStick, Constants.TRIGGER_LEFT_STICK);
    trigger.whenPressed(pid_Drive);

    JoystickButton middle = new JoystickButton(leftStick, Constants.MIDDLE_BUTTON_LEFT_STICK);
    middle.whenPressed(dashboard_PID_Drive);
    ;

     
    // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    drive.setDefaultCommand(driveJoysticks);

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.Joysti  ckButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton trigger = new JoystickButton(leftStick, Constants.TRIGGER_LEFT_STICK);
    trigger.whenPressed(pid_Drive); 
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
   // return m_autoCommand;
  //}
}

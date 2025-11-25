// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.IndexOff;
import frc.robot.commands.IndexOn;
import frc.robot.commands.SetShooterSpeed;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> m_chooser; //creates a place to pick which auto routine we want to run from shuffleboard

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(new SetShooterSpeed(m_shooter, 5250), new IndexOn(m_shooter)));
    NamedCommands.registerCommand("ShootOff", new SequentialCommandGroup(new SetShooterSpeed(m_shooter, 0), new IndexOff(m_shooter)));
    configureBindings(); //this is a method created below, we are running it here to define the triggers for commands
    m_chooser = AutoBuilder.buildAutoChooser(); //the chooser created above gets defined to select between pathplanner autos
    SmartDashboard.putData("Auto Chooser", m_chooser); //adding the chooser to shuffleboard
  }

  //method to configure trigger bindings any thing we want the robot to do based on some state (button press, or sensor value) should go in here
  private void configureBindings() {
    
    m_driverController.rightTrigger().onTrue(new SequentialCommandGroup(new SetShooterSpeed(m_shooter, 5250), new IndexOn(m_shooter))); 
    m_driverController.rightTrigger().onFalse(new SequentialCommandGroup(new SetShooterSpeed(m_shooter, 0), new IndexOff(m_shooter)));

    m_drive.setDefaultCommand(
        new ArcadeDrive(m_drive,() -> -m_driverController.getLeftY(),() -> -m_driverController.getRightX())); //using arcade drive command. telling which controller buttons are inputs
      }
  

  //making a command to call autonomous in Robot.java nothing else needs to be done here if we are just using pathplanner
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}

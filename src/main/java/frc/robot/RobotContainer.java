// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.robot.autos.AUTO_ForwardWaitBack;
import frc.robot.autos.AUTO_Trajectory;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_controller = new XboxController(0);
  // private final XboxController m_secondController = new XboxController(1);
  public final SwerveDrivetrain m_drivetrain = new SwerveDrivetrain();
  final SUB_Intake m_intake = new SUB_Intake();
  public final SUB_Navx m_NavxGyro = new SUB_Navx();
  public final AUTO_Trajectory trajectory = new AUTO_Trajectory(m_drivetrain);
  // public final SUB_Shooter m_shooter = new SUB_Shooter();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData("SyncAngles", new CMD_SyncSwerveEncoders(m_drivetrain));
    SmartDashboard.putData("ResetAngles", new CMD_ResetSwerve(m_drivetrain));
    // Configure the button bindings
    configureButtonBindings();
    // m_FRCGyro.calibrateFRCGyro();
    // m_shooter.setDefaultCommand(new CMD_ShooterOn(m_shooter,m_secondController));
    m_drivetrain.setDefaultCommand(new SwerveDriveCommand(m_drivetrain,m_controller));
    // m_drivetrain.setDefaultCommand(new SwerveTestCommand(m_drivetrain,m_controller));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(m_controller, XboxController.Button.kB.value)
    // .whenPressed(new CMD_ShooterReady(m_shooter));
    // new JoystickButton(m_controller, XboxController.Button.kA.value)
    // .whenPressed(new CMD_ShooterOff(m_shooter));
    new JoystickButton(m_controller, XboxController.Button.kX.value)
    .whenPressed(new CMD_IntakeForward(m_intake));
    new JoystickButton(m_controller, XboxController.Button.kY.value)
    .whenPressed(new CMD_IntakeOff(m_intake));
    new JoystickButton(m_controller, XboxController.Button.kA.value)
    .whenPressed(new CMD_IntakeReverse(m_intake));


  }

  public XboxController getDriveController(){
    return m_controller;
  }





 
  
}

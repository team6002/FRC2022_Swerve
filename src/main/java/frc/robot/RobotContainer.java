// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.commands.*;
import frc.robot.subsystems.SUB_Navx;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_controller = new XboxController(0);
  private final SwerveDrivetrain m_swerve = new SwerveDrivetrain();

  public final SUB_Navx m_NavxGyro = new SUB_Navx();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // SmartDashboard.putData(m_FRCGyro);
    SmartDashboard.putData("ResetOffsets", new CMD_ResetSwerveOffset(m_swerve));
    // Configure the button bindings
    configureButtonBindings();
    // m_FRCGyro.calibrateFRCGyro();
  
    m_swerve.setDefaultCommand(new SwerveDriveCommand(m_swerve,m_controller));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value)
      .whenPressed(new CMD_LeftEvasive(m_swerve))
      .whenReleased(new CMD_NonEvasive(m_swerve));
    
    new JoystickButton(m_controller, XboxController.Button.kRightBumper.value)
      .whenPressed(new CMD_RightEvasive(m_swerve))
      .whenReleased(new CMD_NonEvasive(m_swerve));
    // new JoystickButton(m_controller, XboxController.Axis.kLeftTrigger.value)
    //   .whenHeld(new CMD_LeftEvasive(m_swerve));
  }
    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_autoCommand;
  // }

  public void updateOdometry() {
    m_swerve.updateOdometry();
  }

}

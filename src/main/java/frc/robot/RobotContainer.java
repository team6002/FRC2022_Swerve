// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.autos.*;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_secondController = new XboxController(1);
  public final SwerveDrivetrain m_drivetrain = new SwerveDrivetrain();
  private final SUB_Intake m_intake = new SUB_Intake();
  public final SUB_Navx m_NavxGyro = new SUB_Navx();
  // public final SUB_Shooter m_shooter = new SUB_Shooter();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData("SyncAngles", new CMD_SyncSwerveEncoders(m_drivetrain));
    SmartDashboard.putData("ResetAngles", new CMD_ResetSwerveEncoders(m_drivetrain));
    // Configure the button bindings
    configureButtonBindings();
    // m_FRCGyro.calibrateFRCGyro();
    // m_shooter.setDefaultCommand(new CMD_ShooterOn(m_shooter,m_secondController));
    m_drivetrain.setDefaultCommand(new SwerveDriveCommand(m_drivetrain,m_controller));
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
    // new JoystickButton(m_controller, XboxController.Button.kX.value)
    // .whenPressed(new CMD_IntakeForward(m_intake));
    // new JoystickButton(m_controller, XboxController.Button.kY.value)
    // .whenPressed(new CMD_IntakeOff(m_intake));


  }

  public XboxController getDriveController(){
    return m_controller;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
     
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                    0.5,
                    0.5)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(m_drivetrain.m_kinematics);
    
        // An example trajectory to follow.  All units in meters.
        // Trajectory exampleTrajectory =
        //     TrajectoryGenerator.generateTrajectory(
        //         // Start at the origin facing the +X direction
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         // Pass through these two interior waypoints, making an 's' curve path
        //         List.of(new Translation2d(1, 0), new Translation2d(2,0)),/* new Translation2d(0,1)),*/
        //         // End 3 meters straight ahead of where we started, facing forward
        //         new Pose2d(2, 0, new Rotation2d(0)),
        //         config);
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 0)),/* new Translation2d(0,1)),*/
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(1, 0, new Rotation2d(0)),
                config);
        

        var thetaController =
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                m_drivetrain::getPose, // Functional interface to feed supplier
                m_drivetrain.m_kinematics,
    
                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_drivetrain::setModuleStates,
                m_drivetrain);
    
        // Reset odometry to the starting pose of the trajectory.
        m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
    
        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_drivetrain.drive(0, 0, 0, false));
  }

  public void updateOdometry() {
    m_drivetrain.updateOdometry();
  }
  
}

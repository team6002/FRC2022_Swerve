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
  private final SUB_Intake m_intake = new SUB_Intake();
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





  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // return AUTO_ForwardWaitBack;
  // }
//     // Create config for trajectory
//     TrajectoryConfig config =
//         new TrajectoryConfig(
//             AutoConstants.kMaxSpeedMetersPerSecond,
//             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//             // Add kinematics to ensure max speed is actually obeyed
//             .setKinematics(m_drivetrain.m_kinematics);

//     // An example trajectory to follow.  All units in meters.
//     // Trajectory exampleTrajectory =
//     //     TrajectoryGenerator.generateTrajectory(
//     //         // Start at the origin facing the +X direction
//     //         new Pose2d(0, 0, new Rotation2d(0)),
//     //         // Pass through these two interior waypoints, making an 's' curve path
//     //         List.of(new Translation2d(1, 0), new Translation2d(2,0)),/* new Translation2d(0,1)),*/
//     //         // End 3 meters straight ahead of where we started, facing forward
//     //         new Pose2d(2, 0, new Rotation2d(0)),
//     //         config);

//     // three meters and stop
//     Trajectory exampleTrajectory =
//         TrajectoryGenerator.generateTrajectory(
//             // Start at the origin facing the +X direction
//             new Pose2d(0, 0, new Rotation2d(0)),
//             // Pass through these two interior waypoints, making an 's' curve path
//             List.of(new Translation2d(1,0)),/* new Translation2d(0,1)),*/
//             // End 3 meters straight ahead of where we started, facing forward
//             new Pose2d(3, 0, new Rotation2d(0)),
//             config);
            
//     Trajectory exampleTrajectory2 =
//         TrajectoryGenerator.generateTrajectory(
//             // Start at the origin facing the +X direction
//             new Pose2d(3, 0, new Rotation2d(0)),
//             // Pass through these two interior waypoints, making an 's' curve path
//             List.of(new Translation2d(2,0)),/* new Translation2d(0,1)),*/
//             // End 3 meters straight ahead of where we started, facing forward
//             new Pose2d(0, 0, new Rotation2d(0)),
//             config);

//     //square mode
//     // Trajectory exampleTrajectory =
//     // TrajectoryGenerator.generateTrajectory(
//     //     // Start at the origin facing the +X direction
//     //     new Pose2d(0, 0, new Rotation2d(0)),
//     //     // Pass through these two interior waypoints, making an 's' curve path
//     //     List.of(new Translation2d(1,0),new Translation2d(1,1), new Translation2d(0,1)),/* new Translation2d(0,1)),*/
//     //     // End 3 meters straight ahead of where we started, facing forward
//     //     new Pose2d(0, 0, new Rotation2d(0)),
//     //     config);
    

//     var thetaController =
//         new ProfiledPIDController(
//             AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);
//     // thetaController.setTolerance(1);
//     SwerveControllerCommand swerveControllerCommand =
//         new SwerveControllerCommand(
//             exampleTrajectory,
//             m_drivetrain::getPose, // Functional interface to feed supplier
//             m_drivetrain.m_kinematics,

//             // Position controllers
//             new PIDController(AutoConstants.kPXController, 0, 0),
//             new PIDController(AutoConstants.kPYController, 0, 0),
//             thetaController,
//             m_drivetrain::setModuleStates,
//             m_drivetrain);


//     // Reset odometry to the starting pose of the trajectory.
//     m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

//     // Run path following command, then stop at the end.
//     return swerveControllerCommand.andThen(() -> m_drivetrain.drive(0, 0, 0, false));
// }

// public void updateOdometry() {
// m_drivetrain.updateOdometry();
// }
  
}

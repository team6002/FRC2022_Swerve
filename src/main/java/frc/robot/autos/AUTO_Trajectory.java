// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveDrivetrain;
/** Add your docs here. */

public class AUTO_Trajectory {
    private SwerveDrivetrain m_drivetrain;
    public Trajectory threeMetersForwardTrajectory;
    public Trajectory threeMetersBackwardTrajectory;
    // public Trajectory exampleTrajectory;
    // public Trajectory FirstBallTrajectory;
    // public Trajectory SecondBallTrajectory;
    // public Trajectory ThirdBallTrajectory;
    // public Trajectory ReturnTrajectory;
    public AUTO_Trajectory(SwerveDrivetrain drivetrain){
        m_drivetrain = drivetrain;

        TrajectoryConfig config =
            new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(m_drivetrain.m_kinematics);

        // FirstBallTrajectory = 
        // TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(8.2,2.8, new Rotation2d(0)),
        //      List.of(new Translation2d(7,0.2))
        //     , new Pose2d(7.5,0.2, new Rotation2d(0)), 
        //     config);
        // SecondBallTrajectory =
        // TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(7.5,0.2, new Rotation2d(0)),
        //      List.of(new Translation2d(6,1.8))
        //     , new Pose2d(5,1.8, new Rotation2d(0)), 
        //     config);
        // ThirdBallTrajectory =
        // TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(5,1.8, new Rotation2d(0)),
        //      List.of(new Translation2d(4,1))
        //     , new Pose2d(1,1, new Rotation2d(0)), 
        //     config);
         // three meters and stop
        threeMetersForwardTrajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1,0),
                new Translation2d (2,0)),
                new Pose2d(3, 0, new Rotation2d(0)),
                config);
           
        threeMetersBackwardTrajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(3, 0, new Rotation2d(0)),
                List.of(new Translation2d(2, 0.1),
                new Translation2d(1,0.1)),
                new Pose2d(0, 0, new Rotation2d(0)),
                config.setReversed(true));
        
        //   ReturnTrajectory = 
        // TrajectoryGenerator.generateTrajectory(
        //     drivetrain.getPose(),
        //      List.of(new Translation2d(5,1.8))
        //     , new Pose2d(5,1.8, new Rotation2d(0)), 
        //     config);        
        // exampleTrajectory = 
        // TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         List.of(
        //                 new Translation2d(1, 0),
        //                 new Translation2d(1, -1)),
        //         new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
        //         config);

        
    }
  
    public Command driveTrajectory(Trajectory trajectory) {
     
        // Create config for trajectory
      
        
    
        var thetaController =
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        // thetaController.setTolerance(1);
        
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                trajectory,
                m_drivetrain::getPose, // Functional interface to feed supplier
                m_drivetrain.m_kinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_drivetrain::setModuleStates,
                m_drivetrain);

        // Reset odometry to the starting pose of the trajectory.
        m_drivetrain.resetOdometry(trajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_drivetrain.drive(0, 0, 0, false));
  }

 


}
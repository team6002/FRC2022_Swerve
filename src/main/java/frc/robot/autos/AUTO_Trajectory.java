// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveDrivetrain;
/** Add your docs here. */

public class AUTO_Trajectory {
    
    // String trajectoryJSON = "paths/FirstBall.wpilib.json";
    Trajectory trajectory = new Trajectory();
    private SwerveDrivetrain m_drivetrain;
    // public Trajectory threeMetersForwardTrajectory;
    // public Trajectory threeMetersBackwardTrajectory;
    // public Trajectory exampleTrajectory;
    public Trajectory FirstBallTrajectory;
    public Trajectory SecondBallTrajectory;
    // public Trajectory FluidThreeBallTrajectory;
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

        // Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        //     try {
        //         trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        //     } catch (IOException e) {
        //         e.printStackTrace();
        // }           
        
        //Rotation2d uses RADIANS NOT DEGREES!
        //Use Rotation2d.fromDegrees(desiredDegree) instead
        FirstBallTrajectory = 
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d(0)),
             List.of(
                //  new Translation2d(0.5, 0)
            ), 
            new Pose2d(1.1,0, new Rotation2d(0)), 
            config
        );

        SecondBallTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                // new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                // new Pose2d(-0.25, -1, Rotation2d.fromDegrees(0)),
            ),
            new Pose2d(-0.8, -2.371, Rotation2d.fromDegrees(45)),
            config.setReversed(true)
        );    
            
            // ThirdBallTrajectory =
        // TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(5,1.8, new Rotation2d(0)),
        //      List.of(new Translation2d(4,1))
        //     , new Pose2d(1,1, new Rotation2d(0)), 
        //     config);
         // three meters and stop
            
           
        // threeMetersBackwardTrajectory =
        //     TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         List.of(new Translation2d(2, 0.1),
        //         new Translation2d(1,0.1)),
        //         new Pose2d(0, 0, new Rotation2d(0)),
        //         config.setReversed(true));
        
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
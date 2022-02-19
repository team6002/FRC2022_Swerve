// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */


public final class Constants {

    public static final class DriveConstants {
    public static double k_frontLeftOffset =0.0;   // 15;
    public static double k_backLeftOffset = 0.0;//40;
    public static double k_frontRightOffset = 0.0;
    public static double k_backRightOffset = 0.0;// 164;
    public static double k_LeftEvasiveX = 0.0635;
    public static double k_LeftEvasiveY = 0.6477;
    public static double k_RightEvasiveX = 0.6477;
    public static double k_RightEvasiveY = -0.0635;
    }
        /**
     * Gear Ratios for Swerve
     * Turning: 16-1
     * Drive  : 5-1
     * 
     * Sparkmax Turning Position Conversion Factor
     * 360/16 = 22.5
     */

    public static final class ShooterConstants {
    
        //shooter
        public static final int kShooterMaster = 8;
        public static final int kShooterSlave = 9;
    
        public static final double kShooterFF = 0.000230;
        public static final double kShooterP = 0.000080;
        public static final double kShooterI = 0; 
        public static final double kShooterD = 0.000;
    
        //Max free velocity ~5000
        public static final double kMaxShooterVelocity = 2750;//max working velocity 
        public static final double kShootingVelocity = 2075;    //2650;
        public static final double kShootingAccel = 1200;
    
        public static final double kMinOutput = -1;
        public static final double kMaxOutput = 1;
    
        public static final double kShooterSpeed = 0.30;
    
        //turret
        public static final int kTurretMotor = 1;
    

        
    }
    public static final class TurretConstants {
        //turret
        public static final int kTurretMotor = 2;
    
        //turret PID
        public static final double kTurretFF = 0;
        public static final double kTurretP = 0.0049;
        public static final double kTurretI = 0.00003;
        public static final double kTurretD = 0.00003;
    
        //turret speeds
        public static final double kMinTurretOutput = -1;
        public static final double kMaxTurretOutput = 1;
    
        public static final double kTurretVoltage = 10;
        public static final double kTurretHuntVoltage = 1;
      }
    public static final class IndexerConstants{
        // Intake stuff
        public static final int kFrontIntake = 12;
        public static final int kBackIntake = 1;
        public static final int kFrontIntakeIR = 0;
        public static final int kBackIntakeIR=2 ;
        
        public static final double IntakeForward = 0.5;
        public static final double IntakeOff = 0;
        public static final double IntakeReverse = -0.4;

        //Indexer stuff
        public static final int kIndexer = 7;
        public static final int kHopper  = 17;
        public static final int kHopperIR = 1;

        public static final double HopperForward = 0.3;
        public static final double HopperOff = 0;
        public static final double HopperBackward = -0.3;

        public static final double IndexerForward = 0.5;
        public static final double IndexerOff = 0;
        public static final double IndexerBackward = -0.3;


    }
     
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI/4;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI/4;
    
        public static final double kPXController = 6;
        public static final double kPYController = 6;
        public static final double kPThetaController = 2*Math.PI;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }
}

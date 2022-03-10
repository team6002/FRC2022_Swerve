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
        public static final double kShooterP = 0.00012;
        public static final double kShooterI = 0; 
        public static final double kShooterD = 0.000;
    
        //Max free velocity ~5000
        public static final double kMaxShooterVelocity = 2750;//max working velocity 
        public static final double kShootingVelocity =  2475;    
        public static final double kShootingAccel = 1200;
    
        public static final double kMinOutput = -1;
        public static final double kMaxOutput = 1;
    
        public static final double kShooterSpeed = 0.30;
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
    
        public static final double kTurretVoltage = 3;
        public static final double kTurretHuntVoltage = 1;
        public static final double kTurretMannualVoltage = 3;
        public static final double kTurretResetVoltage = 1;
      }

    public static final class IndexerConstants{
        //INTAKE PORTS
        public static final int kFrontIntake = 12;
        public static final int kBackIntake = 1;
        public static final int kFrontIntakeIR = 0;
        public static final int kBackIntakeIR=5 ;
        public static final int kIndexer = 7;
        public static final int kHopper  = 17;
        public static final int kHopperIR = 1;
        public static final int kFrontIntakeSolonoid = 0;
        public static final int kBackIntakeSolonoid = 1;

        //INDEXER (pre-stager)
        public static final double kIndexerFF = 0.00031;
        public static final double kIndexerP = 0.0003;
        public static final double kIndexerI = 0;  
        public static final double kIndexerD = 0.0001;
        public static final double kIndexerMaxVelocity = 2000;
        public static final double kIndexerAccel = 2000;

        //HOPPER
        public static final double kHopperFF = 0.0003;
        public static final double kHopperP = 0.000052;
        public static final double kHopperI = 0.00000;  
        public static final double kHopperD = 0.00001;
        public static final double kHopperMaxVelocity = 4500;//1500;
        public static final double kHopperAccel = 2000;

        //INTAKE
        public static final double kIntakeFF = 0.00026; 
        public static final double kIntakeP = 0.000032;
        public static final double kIntakeI = 0;  
        public static final double kIntakeD = 0.0000;
        public static final double kIntakeMaxVelocity = 2000;
        public static final double kIntakeAccel = 2000;

        //SPEEDS
        public static final double kIndexerVelocity = 1100;
        public static final double kHopperVelocity = 1200; 
        public static final double kIntakeVelocity = 900;
        public static final double HopperOff = 0;
        public static final double IndexerOff = 0;
        public static final double IntakeOff = 0;

       
        public static final double kMinOutput = -1;
        public static final double kMaxOutput = 1; 

    }

    public static final class ClimberConstants{
        public static final int kSecondaryClimberMotor1 = 10;
        public static final int kSecondaryClimberMotor2 = 11;
        public static final int kPrimaryClimberMotor1 = 18;
        public static final int kPrimaryClimberMotor2 = 19;
        public static final int kMainSolonoid = 3;
        public static final int kSecondSolonoid = 2;
        public static final double ThroughBoreEncoderPort1 = 3;
        public static final double ThroughBoreEncoderPort2 = 4;
        //PID VALUES
        //Primary
        public static final double kPrimaryClimberFF = 0.0016;
        public static final double kPrimaryClimberP = 0.000001;
        public static final double kPrimaryClimberI = 0;
        public static final double kPrimaryClimberD = 0;
        public static final double kPrimaryClimberIz = 0;

        public static final double kPrimaryClimberMaxVelocity = 250;
        public static final double kPrimaryClimberMaxAccel = 250;
        public static final double kPrimaryClimberAllowedError = 0.5;
        public static final double kPrimaryClimberMinOutput = -1;
        public static final double kPrimaryClimberMaxOutput = 1;

        //Secondary
        public static final double kSecondaryClimberFF = 0.0016;
        public static final double kSecondaryClimberP = 0.000003;
        public static final double kSecondaryClimberI = 0;
        public static final double kSecondaryClimberD = 0;
        public static final double kSecondaryClimberIz = 0;

        public static final double kSecondaryClimberMaxVelocity = 300;
        public static final double kSecondaryClimberMaxAccel = 300;
        public static final double kSecondaryClimberAllowedError = 0.5;
        public static final double kSecondaryClimberMinOutput = -1;
        public static final double kSecondaryClimberMaxOutput = 1;

        //Climb Positions
        public static final int PrimaryClimberFullyExtended = 116; 
        public static final int PrimaryClimberDeploy = 116;
        public static final int PrimaryClimberLevitate = 94;// pid test height
        public static final int PrimaryClimberLift = 0;
        public static final int PrimaryClimberFivePointsLower = 5;
        public static final int PrimaryClimberRelease = 10;
         

        public static final int SecondaryClimberFullyExtended = 111;
        public static final int SecondaryClimberDeploy = 115;
        public static final int SecondaryClimberPreStage = 98;
        public static final int SecondaryClimberLift = 80; //temp value
        
    }
     

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
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

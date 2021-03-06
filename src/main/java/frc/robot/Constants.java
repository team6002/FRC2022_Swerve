// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;



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
        
    
        public static double kMaxSpeedMetersPerSecond = 6380.0 / 60.0 *
                (16.0 / 32.0) * ( 15.0 / 45.0) * Units.inchesToMeters(3) * Math.PI;
        public static int kNavXAdjustment = 0;

        public static final int kFrontLeftTurningMotorPort = 16;    
        public static final int kFrontLeftDriveMotorPort = 15;
        public static final int kRearLeftTurningMotorPort = 4;
        public static final int kRearLeftDriveMotorPort = 3;

        public static final boolean kFrontLeftDriveMotorInverted = false;
        public static final boolean kFrontRightDriveMotorInverted = true;
        public static final boolean kRearLeftDriveMotorInverted = false;
        public static final boolean kRearRightDriveMotorInverted = true;
        
        public static final int kFrontRightTurningMotorPort = 14;
        public static final int kFrontRightDriveMotorPort = 13;
        public static final int kRearRightTurningMotorPort = 6;
        public static final int kRearRightDriveMotorPort = 5;

        public static final int kFrontLeftDriveAnalogPort = 0;
        public static final int kFrontRightDriveAnalogPort = 1;
        public static final int kRearLeftDriveAnalogPort = 2;
        public static final int kRearRightDriveAnalogPort = 3;

        public static final int kFrontLeftTurningDutyCycleEncoder = 6;
        public static final int kFrontRightTurningDutyCycleEncoder = 7;
        public static final int kRearLeftTurningDutyCycleEncoder = 8;
        public static final int kRearRightTurningDutyCycleEncoder = 9;
        

        public static final double kTrackWidth = Units.inchesToMeters(23.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(23.5);
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); 
    
    
    
    
    }
    /*
     * Gear Ratios for Swerve
     * Turning: 16-1
     * Drive  : 6-1
     * 
     * Sparkmax Turning Position Conversion Factor
     * 360/16 = 22.5
    */

    public static final class ShooterConstants {

        //shooter
        public static final int kShooterMaster = 8;
        public static final int kShooterSlave = 9;
        public static final int kShooterHoodSolonoid = 3;
       
    
        public static final double kShooterFF = 0.00023;
        public static final double kShooterP = 0.0000;//8;//0.0004;
        public static final double kShooterI = 0.00000001; 
        public static final double kShooterD = 0.000;
    
        //Shooter is 1:1
        public static final double kMaxShooterVelocity = 7000; //2750;//max working velocity 
        public static final double kShootingVelocity = 2000;    
        public static final double kShootingAccel = 5000;
        public static final double kCloseShootingVelocityFirstShot = 2150;
        public static final double kCloseShootingVelocitySecondShot = 2275;
        public static final double kReverseShooterVelocity = -1000;
        public static final double kLowShootingVelocity = 1750;
        public static final double kMinOutput = -1;
        public static final double kMaxOutput = 1;
    
        public static final double[][] kShooterArray = {
            // // Twisted Devil field
            // {-5, 3000}
            // ,{0, 2600}
            // ,{7, 2250}
            // ,{14, 2100}
            // //lake view tournamet
            // {-2, 3000}, // bumper against the safe zone
	        // {2, 2600}, // center of the robot on O ring and the  front of the robot is 11 feet 9 inches 
            // {11, 2250}, // front frame on the tarmac and about 9 feet from center
            // {17, 2100}  // the front of the robot is 7ft and 10 inches away from center and back bumper on the tarmac
            
            {-2, 3200}, // bumper against the safe zone
	        {2, 2800}, // center of the robot on O ring and the  front of the robot is 11 feet 9 inches 
            {7, 2600}, // front frame on the tarmac and about 9 feet from center
            {10, 2550}  // the front of the robot is 7ft and 10 inches away from center and back bumper on the tarmac
            
            // {-5, 3000}, // bumper against the safe zone
	        // {-1, 2700}, // center of the robot on O ring and the  front of the robot is 11 feet 9 inches 
            // {5, 2250}, // front frame on the tarmac and about 9 feet from center
            // {8, 2100}  // the front of the robot is 7ft and 10 inches away from center and back bumper on the tarmac
            
		};
    }
    
    public static final class TurretConstants {
        //turret
        public static final int kTurretMotor = 2;
    
        //turret PID
        public static final double kTurretFF = 0.00035; //0.0007;
        public static final double kTurretP = 0.000; //0.60; //0.3; (testing P)
        public static final double kTurretI = 0.0;
        public static final double kTurretD = 0.0;
    
        //turret speeds
        public static final double kMaxTurretVelocity = 900; //32 * Math.PI * 60;
        public static final double kMaxTurretAccel = 4000;

        public static final double kMinTurretOutput = -1;
        public static final double kMaxTurretOutput = 1;
    
        //turret voltage
        public static final double kTurretVoltage = 5;
        // public static final double kTurretHuntVoltage = 1;
        public static final double kTurretMannualVoltage = 3;
        public static final double kTurretResetVoltage = 1;
        public static final double kTurretJoystickVoltage = 0.5;
      }

    public static final class IndexerConstants{
        //INTAKE PORTS
        public static final int kFrontIntake = 12;
        public static final int kBackIntake = 1;
        public static final int kFrontIntakeIR = 2;
        public static final int kBackIntakeIR= 3;
        public static final int kIndexer = 7;
        public static final int kHopper  = 17;
        public static final int kHopperIR = 1;
        public static final int kFrontIntakeSolonoid = 0;
        public static final int kBackIntakeSolonoid = 1;

        //INDEXER (pre-stager)
        public static final double kIndexerFF = 0.000220;
        public static final double kIndexerP = 0.0003;
        public static final double kIndexerI = 0;  
        public static final double kIndexerD = 0.0001;
        public static final double kIndexerMaxVelocity = 4000;
        public static final double kIndexerAccel = 2000;

        //HOPPER
        public static final double kHopperFF = 0.00021;
        public static final double kHopperP = 0.000052;
        public static final double kHopperI = 0.00000;  
        public static final double kHopperD = 0.00001;
        public static final double kHopperMaxVelocity = 4500;//1500;
        public static final double kHopperAccel = 3000;
        public static final int kHopperCurrentLimit = 50;

        //INTAKE
        public static final double kIntakeFF = 0.0002; 
        public static final double kIntakeP = 0;  //0.000032;
        public static final double kIntakeI = 0;  
        public static final double kIntakeD = 0.0000;
        public static final double kIntakeMaxVelocity = 5000;
        public static final double kIntakeAccel = 3000;
        public static final int kIntakeCurrentLimit = 40;

        //SPEEDS
        public static final double kIndexerVelocity = 4000;
        public static final double kHopperVelocity = 2000; 
        public static final double kIntakeVelocity = 4000;
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
        public static final int kSecondHookSolonoid = 2;

        //PID VALUES
        //Primary
        // first set for ascending
        public static final double kPrimaryClimberFF = 0.005;
        public static final double kPrimaryClimberP = 0.000001;
        public static final double kPrimaryClimberI = 0;
        public static final double kPrimaryClimberD = 0;
        public static final double kPrimaryClimberIz = 0;

        public static final double kPrimaryClimberMaxVelocity = 300;
        public static final double kPrimaryClimberMaxAccel = 150;
        public static final double kPrimaryClimberAllowedError = 0.5;
        public static final double kPrimaryClimberMinOutput = -1;
        public static final double kPrimaryClimberMaxOutput = 1;
       
        //Second set of PID for descending
        public static final double kPrimaryClimberFF2 = 0.0004;
        public static final double kPrimaryClimberP2 = 0.000001;
        public static final double kPrimaryClimberI2 = 0;
        public static final double kPrimaryClimberD2 = 0;
        public static final double kPrimaryClimberIz2 = 0;

        public static final double kPrimaryClimberMaxVelocity2 = 500;
        public static final double kPrimaryClimberMaxAccel2 = 300;
        public static final double kPrimaryClimberAllowedError2 = 0.5;
        public static final double kPrimaryClimberMinOutput2 = -1;
        public static final double kPrimaryClimberMaxOutput2 = 1;

        //Secondary 
        // First set of PID for ascending
        public static final double kSecondaryClimberFF = 0.002;
        public static final double kSecondaryClimberP = 0.000003;
        public static final double kSecondaryClimberI = 0;
        public static final double kSecondaryClimberD = 0;
        public static final double kSecondaryClimberIz = 0;

        public static final double kSecondaryClimberMaxVelocity = 400;
        public static final double kSecondaryClimberMaxAccel = 200;
        public static final double kSecondaryClimberAllowedError = 0.5;
        public static final double kSecondaryClimberMinOutput = -1;
        public static final double kSecondaryClimberMaxOutput = 1;
      
        // second set of PID for Desecending
        public static final double kSecondaryClimberFF2 = 0.001;
        public static final double kSecondaryClimberP2 = 0.000003;
        public static final double kSecondaryClimberI2 = 0;
        public static final double kSecondaryClimberD2 = 0;
        public static final double kSecondaryClimberIz2 = 0;

        public static final double kSecondaryClimberMaxVelocity2 = 400;
        public static final double kSecondaryClimberMaxAccel2 = 150;
        public static final double kSecondaryClimberAllowedError2 = 0.5;
        public static final double kSecondaryClimberMinOutput2 = -1;
        public static final double kSecondaryClimberMaxOutput2 = 1;

        //Climb Positions
        public static final double PrimaryClimberFullyExtended = 34;//116; 
        public static final double PrimaryClimberDeploy = 34; //116;
        public static final double PrimaryClimberLevitate = 25.85;//94;// pid test height
        public static final double PrimaryClimberLift = 0;
        public static final double PrimaryClimberFivePointsLower = 1.375;
        public static final double PrimaryClimberRelease = 2.75;
         

        public static final double SecondaryClimberFullyExtended = 33.5;//111;
        public static final double SecondaryClimberDeploy = 33.5; //111;
        public static final double SecondaryClimberPreStage = 29.106;
        public static final double SecondaryClimberLift = 23.76; //temp value
        
    }
     
    public static final class AutoConstants {
        // public static final double kMaxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond;
        // public static final double kMaxAccelerationMetersPerSecondSquared = DriveConstants.kMaxSpeedMetersPerSecond;
        public static final double kMaxSpeedMetersPerSecond = 4.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
     
        public static final double kMaxAngularSpeedRadiansPerSecond = 4*Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 4*Math.PI;
    
        public static final double kPXController = 2;
        public static final double kPYController = 2;
        public static final double kPThetaController = 2;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }
}

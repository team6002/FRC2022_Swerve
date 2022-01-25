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
    public static double k_frontLeftOffset = 0;   // 15;
    public static double k_backLeftOffset = 0;//40;
    public static double k_frontRightOffset = 0;
    public static double k_backRightOffset = 0;// 164;
    public static double k_LeftEvasiveX = 0.0635;
    public static double k_LeftEvasiveY = 0.6477;
    public static double k_RightEvasiveX = 0.6477;
    public static double k_RightEvasiveY = -0.0635;
    
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
        public static final int kShooterMaster = 14;
        public static final int kShooterSlave = 15;
    
        public static final double kShooterFF = 0.000230;
        public static final double kShooterP = 0.000080;
        public static final double kShooterI = 0; 
        public static final double kShooterD = 0.000;
    
        //Max free velocity ~5000
        public static final double kMaxShooterVelocity = 2750;//max working velocity 
        public static final double kShootingVelocity = 2650;
        public static final double kShootingAccel = 1200;
    
        public static final double kMinOutput = -1;
        public static final double kMaxOutput = 1;
    
        public static final double kShooterSpeed = 0.30;
    
        //turret
        public static final int kTurretMotor = 11;
    
        //indexer
        public static final int kIndexerTop = 12;
        public static final int kIndexerBottom = 13;
        public static final int kIndexerBack = 10;
    
        public static final double kIndexerFSpeed = 0.5;
        public static final double kIndexerRSpeed = -0.3;
    
        public static final double kStabilizationP = 1;
        public static final double kStabilizationI = 0.5;
        public static final double kStabilizationD = 0;
    
        public static final double kTurnP = 1;
        public static final double kTurnI = 0;
        public static final double kTurnD = 0;
    
        public static final double kMaxTurnRateDegPerS = 100;
        public static final double kMaxTurnAccelerationDegPerSSquared = 300;
    
        public static final double kTurnToleranceDeg = 5;
        public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
    }
     
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }
}

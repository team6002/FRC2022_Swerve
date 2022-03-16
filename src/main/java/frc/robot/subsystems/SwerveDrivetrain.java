// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;



public class SwerveDrivetrain extends SubsystemBase {

  public static final double kMaxSpeed = Units.feetToMeters(10); // 13.6 feet per second/ 10 feet per second now
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  /**
   * TODO: These are example values and will need to be adjusted for your robot!
   * Modules are in the order of -
   * Front Left
   * Front Right
   * Back Left
   * Back Right
   * 
   * Positive x values represent moving toward the front of the robot whereas
   * positive y values represent moving toward the left of the robot
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */

  private final Translation2d m_frontLeftLocation = new Translation2d(Units.inchesToMeters(11.75), Units.inchesToMeters(11.75));
  private final Translation2d m_frontRightLocation = new Translation2d(Units.inchesToMeters(11.75), Units.inchesToMeters(-11.75));
  private final Translation2d m_backLeftLocation = new Translation2d(Units.inchesToMeters(-11.75), Units.inchesToMeters(11.75));
  private final Translation2d m_backRightLocation = new Translation2d(Units.inchesToMeters(-11.75), Units.inchesToMeters(-11.75));

  private final SwerveModule m_frontLeft = new SwerveModule(25, 16, true);//(15,16)
  private final SwerveModule m_frontRight = new SwerveModule(13, 14, false);
  private final SwerveModule m_backLeft = new SwerveModule(3, 4, true);
  private final SwerveModule m_backRight = new SwerveModule(5, 6, false);

  private final AHRS m_Navx = new AHRS(Port.kMXP);
  // private double EvasiveX = 0;
  // private double EvasiveY = 0;
  private boolean fieldMode = true;

  public final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, m_Navx.getRotation2d());//, new Pose2d(0, 0, new Rotation2d()

  public SwerveDrivetrain() {
    syncAllAngles();
    // resetDriveEncoder();
  }

  //sets the gyro angle to 0
  public void zeroGyroscope() {
    m_Navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
    if (m_Navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_Navx.getFusedHeading());
    }

    //counter-clockwise = positive angle
    return Rotation2d.fromDegrees(360.0 - m_Navx.getYaw());
  }

  public void syncAllAngles() {
    m_frontLeft.syncAngle();
    m_frontRight.syncAngle();
    m_backLeft.syncAngle();
    m_backRight.syncAngle();
  }
  
  public void resetAllAngles(){
    m_frontLeft.resetDriveEnc();
    m_frontRight.resetDriveEnc();
    m_backLeft.resetDriveEnc();
    m_backRight.resetDriveEnc();
  }
  // public void stopModules() {
  //   m_frontLeft.stop();
  //   m_frontRight.stop();
  //   m_backLeft.stop();
  //   m_backRight.stop();
  // }
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
  //  * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_Navx.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, kMaxSpeed);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }
  
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public double getOdometryX(){
    return m_odometry.getPoseMeters().getX();
  }
  public double getOdometryY(){
    return m_odometry.getPoseMeters().getY();
  }
  public double getOdometryRotate(){
    return m_odometry.getPoseMeters().getRotation().getDegrees();
  }
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_Navx.getRotation2d());
  }
  public double getDegrees(){
    return m_Navx.getRotation2d().getDegrees();
  }

  // public void LeftEvasive(){
  //   EvasiveX = DriveConstants.k_LeftEvasiveX;
  //   EvasiveY = DriveConstants.k_LeftEvasiveY;
  // }
  // public void RightEvasive(){
  //   EvasiveX = DriveConstants.k_RightEvasiveX;
  //   EvasiveY = DriveConstants.k_RightEvasiveY;
  // }
  // public void NonEvasive(){
  //   EvasiveX = 0;
  //   EvasiveY = 0;
  // }
  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    // var gyroAngle = Rotation2d.fromDegrees(-m_Navx.getAngle());
    m_odometry.update(
        // gyroAngle,
        m_Navx.getRotation2d(),
        // new Rotation2d(0),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

  public void resetDriveEncoder(){
    m_frontLeft.resetDriveEnc();
    m_frontRight.resetDriveEnc();
    m_backLeft.resetDriveEnc();
    m_backRight.resetDriveEnc();
  }

  public void fieldModeChange(){
    fieldMode = !fieldMode;
  }

  public boolean getFieldMode(){
    return fieldMode;
  }
  
  @Override
  public void periodic() {
    updateOdometry();
    // SmartDashboard.putBoolean("FieldRelative", fieldMode);
    SmartDashboard.putNumber("OdoX", getOdometryX());
    SmartDashboard.putNumber("OdoY", getOdometryY());
    SmartDashboard.putNumber("OdoRotate", getOdometryRotate());
    m_frontLeft.updateSmartDashboard();
    m_frontRight.updateSmartDashboard();
    m_backLeft.updateSmartDashboard();
    m_backRight.updateSmartDashboard();
    
    SmartDashboard.putNumber("NavxDegrees", m_Navx.getRotation2d().getDegrees());
    // SmartDashboard.putNumber("NavxRadians", m_Navx.getRotation2d().getRadians());

    // This method will be called once per scheduler run
  }



  @Override       
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
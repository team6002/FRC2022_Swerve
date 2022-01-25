// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;



public class SwerveDrivetrain extends SubsystemBase {

  public static final double kMaxSpeed = Units.feetToMeters(13.6); // 13.6 feet per second
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

  private final SwerveModule m_frontLeft = new SwerveModule(1, 2, Constants.k_frontLeftOffset, false);
  private final SwerveModule m_frontRight = new SwerveModule(3, 4, Constants.k_frontRightOffset, true);
  private final SwerveModule m_backLeft = new SwerveModule(5, 6, Constants.k_backLeftOffset, true);
  private final SwerveModule m_backRight = new SwerveModule(7, 8, Constants.k_backRightOffset, false);

  private final SUB_Navx m_Navx = new SUB_Navx();   
  private double EvasiveX = 0;
  private double EvasiveY = 0;
  private boolean fieldMode = false;

  public final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, m_Navx.getRotation2d());

  public SwerveDrivetrain() {
    m_Navx.resetNavx(); 
    syncAllAngles();
  }
  
  public void syncAllAngles() {
    m_frontLeft.syncAngle();
    m_frontRight.syncAngle();
    m_backLeft.syncAngle();
    m_backRight.syncAngle();
  }

    /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_Navx.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot), new Translation2d(EvasiveX,EvasiveY));
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

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_Navx.getRotation2d());
  }

  public void LeftEvasive(){
    EvasiveX = Constants.k_LeftEvasiveX;
    EvasiveY = Constants.k_LeftEvasiveY;
  }
  public void RightEvasive(){
    EvasiveX = Constants.k_RightEvasiveX;
    EvasiveY = Constants.k_RightEvasiveY;
  }
  public void NonEvasive(){
    EvasiveX = 0;
    EvasiveY = 0;
  }
  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_Navx.getRotation2d(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

  public void fieldModeChange(){
    fieldMode = !fieldMode;
  }

  public boolean getFieldMode(){
    return fieldMode;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("FieldRelative", fieldMode);

    // This method will be called once per scheduler run
  }

  @Override       
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.math.util.Units;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;

// import com.revrobotics.AnalogInput;
import com.revrobotics.SparkMaxPIDController;

public class SwerveModule {

  // TODO: Tune these PID values for your robot
  private static final double kDriveP = 0.005;  //15.0;
  private static final double kDriveI = 0;//0;  //0.01;
  private static final double kDriveD = 0.1;  //0.1;
  private static final double kDriveF = 0.2;  //0.2;

  private static final double kAngleP = 0.006;
  private static final double kAngleI = 0.0;
  private static final double kAngleD = 0.0;
  private static final double kAngleFF = 0.0000;
  // private static final double kModuleMaxAngularVelocity = SwerveDrivetrain.kMaxAngularSpeed;
  // private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private final double m_turningMotorChannel;
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkMaxPIDController m_drivePIDController;
  private final SparkMaxPIDController m_turningPIDController;
 
  private final  SparkMaxAnalogSensor m_analogSensor;
  // private final boolean m_driveDirection;
  private double m_turningEncoderOffset = 0.0;//15;
  
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      double turningOffset,
      boolean driveDirection) {

    m_turningMotorChannel = turningMotorChannel;    
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.setInverted(driveDirection);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_turningEncoderOffset = turningOffset;
     /**
     * The restoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */

     
    // m_driveMotor.restoreFactoryDefaults();
    // m_turningMotor.restoreFactoryDefaults();

    m_driveEncoder = m_driveMotor.getEncoder();
    // m_driveEncoder.setPositionConversionFactor(1); //* 0.0381);
    // // 0.0381
    m_driveEncoder.setVelocityConversionFactor(((2.0*Math.PI*0.0381)/6)/60.0);
    m_driveEncoder.setPositionConversionFactor((2.0 * Math.PI * 0.0381)/6.0);
    m_driveEncoder.setPosition(0);
  

    m_turningEncoder = m_turningMotor.getEncoder();
    m_turningEncoder.setPositionConversionFactor(22.5);

    m_analogSensor = m_turningMotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
    m_analogSensor.setInverted(true);
    m_analogSensor.setPositionConversionFactor(109.091);

    m_turningPIDController = m_turningMotor.getPIDController();
    // m_turningPIDController.enableContinuousInput(-Math.PI,Math.PI);
    // m_turningPIDController.setFeedbackDevice(m_analogSensor);
    m_turningPIDController.setP(kAngleP);
    m_turningPIDController.setI(kAngleI);
    m_turningPIDController.setD(kAngleD);
    m_turningPIDController.setFF(kAngleFF);

    m_drivePIDController = m_driveMotor.getPIDController();
    m_drivePIDController.setFeedbackDevice(m_driveEncoder);
    m_drivePIDController.setP(kDriveP);
    m_drivePIDController.setI(kDriveI);
    m_drivePIDController.setD(kDriveD);
    m_drivePIDController.setFF(kDriveF);
}

  
  /**
   * Gets the relative rotational position of the module
   * @return The relative rotational position of the angle motor in degrees
   */
  public double getAbsAngle() {
    // set the lamprey sensor to zero degree pointing to the back
    // the analog signal is zero to 3.3v, representing 0 to 360 degree, CCW positive
    double absoluteAngle = m_analogSensor.getPosition();
    if (absoluteAngle > 360) absoluteAngle = absoluteAngle % 360;
    // we need to convert it to 180 degree (left) to -180 (right), zero degree pointing forward
    // we can do this by subtracting 180 from the value
    absoluteAngle-=180;

    return absoluteAngle;
  }

    /**
   * Gets the relative rotational position of the module
   * @return The relative rotational position of the angle motor in degrees
   */
  public double getAngle() {
    // Note: This assumes the CANCoders are setup with the default feedback coefficient
    // and the sensor value reports degrees.
    double angle = ((m_turningEncoder.getPosition() - m_turningEncoderOffset) % 360);
    // double angle = (m_analogSensor.getPosition() - m_turningEncoderOffset) % 360;
    if (angle > 180) angle-=360;

    return getAbsAngle();
  }

  // public void updateSmartDashboard() {
  //       double angle = getAbsAngle();
  //       angle -= m_turningEncoderOffset;
  //       if (angle > 180) angle -= 360;

        // // SmartDashboard.putNumber("turnRaw:"+m_turningMotorChannel, m_turningEncoder.getPosition());
        // SmartDashboard.putNumber("turnEnc:"+m_turningMotorChannel, getAngle());
        // SmartDashboard.putNumber("driveEnc:"+m_turningMotorChannel, m_driveEncoder.getPosition());
        // SmartDashboard.putNumber("rawAnalogEnc:"+m_turningMotorChannel, m_analogSensor.getPosition());
        // SmartDashboard.putNumber("turnAbs:"+m_turningMotorChannel, angle);
        // SmartDashboard.putNumber("driveVelocity"+m_turningMotorChannel, m_driveEncoder.getVelocity());
        // SmartDashboard.putNumber("turnOff:"+m_turningMotorChannel, m_turningEncoderOffset);
  // }
  /**
   * Resets the relative encoder to the absolute encoder.
   */
  public void syncAngle() {
    m_turningEncoder.setPosition(getAbsAngle());
  }
  public void resetDriveEnc(){
    m_driveEncoder.setPosition(0);
  }
  public void setAngle(Rotation2d rotation, Rotation2d currentRotation) {
    Rotation2d rotationDelta = rotation.minus(currentRotation);
    double position = rotationDelta.getDegrees() + m_turningEncoder.getPosition();
    m_turningPIDController.setReference(position, ControlType.kPosition);
  }


  // public void setAngle(Rotation2d rotation) {
  //   double angle = rotation.getDegrees();
  //   angle+=180;
  //   m_turningPIDController.setReference(angle, ControlType.kPosition);
    
  // }
 
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // double fakeSpeed = -0.16666;
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(getAngle() * (Math.PI / 180.0)));
    // return new SwerveModuleState(fakeSpeed, new Rotation2d(Math.PI / 2));
  }
  /**
   * 
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentRotation = Rotation2d.fromDegrees(getAngle());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, currentRotation);

      // if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      //     stop();
      //     return;
      // }
    setAngle(state.angle, currentRotation);
    // setAngle(state.angle);

    // SmartDashboard.putNumber("TurnC:"+m_turningMotorChannel, currentRotation.getDegrees());
    // SmartDashboard.putNumber("TurnD:"+m_turningMotorChannel, state.angle.getDegrees());


    // double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);
    double speed = state.speedMetersPerSecond / SwerveDrivetrain.kMaxSpeed;
    m_driveMotor.set(speed);
        
  }  
  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
} 
}
 
 
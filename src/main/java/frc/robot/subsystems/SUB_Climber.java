// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
public class SUB_Climber extends SubsystemBase {
  private final Solenoid m_SecondSolenoid;
  private final Solenoid m_MainSolenoid;
 
  private CANSparkMax m_SecondaryClimberMotor2;
  private CANSparkMax m_SecondaryClimberMotor1;
  private CANSparkMax m_PrimaryClimberMotor2;
  private CANSparkMax m_PrimaryClimberMotor1;
 
  private SparkMaxLimitSwitch m_PrimaryHomeLimitSwitch;
  private SparkMaxLimitSwitch m_SecondaryHomeLimitSwitch;
  public boolean MainSolonoidState = false;
  public boolean SecondSolonoidState = false;
  
  private RelativeEncoder m_PrimaryEncoder;
  private RelativeEncoder m_SecondaryEncoder;

  private SparkMaxPIDController m_PrimaryClimberPID;
  private SparkMaxPIDController m_SecondaryClimberPID;

  private double PrimarySetpoint = 0;
  private double SecondarySetpoint = 0;
  private boolean climbing = false;
  /** Creates a new SUB_Climber. */
  public SUB_Climber() {
    m_SecondSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.kSecondSolonoid);
    m_MainSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.kMainSolonoid);
 
    m_SecondaryClimberMotor2 = new CANSparkMax(ClimberConstants.kSecondaryClimberMotor2,MotorType.kBrushless);
    m_SecondaryClimberMotor1 = new CANSparkMax(ClimberConstants.kSecondaryClimberMotor1,MotorType.kBrushless);
    m_PrimaryClimberMotor2 = new CANSparkMax(ClimberConstants.kPrimaryClimberMotor2,MotorType.kBrushless);
    m_PrimaryClimberMotor1 = new CANSparkMax(ClimberConstants.kPrimaryClimberMotor1,MotorType.kBrushless);
    
    m_PrimaryHomeLimitSwitch = m_PrimaryClimberMotor1.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    m_SecondaryHomeLimitSwitch = m_SecondaryClimberMotor1.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  
    m_PrimaryEncoder = m_PrimaryClimberMotor1.getEncoder();
    m_SecondaryEncoder = m_SecondaryClimberMotor1.getEncoder();

    m_PrimaryClimberPID = m_PrimaryClimberMotor1.getPIDController();
    m_SecondaryClimberPID = m_SecondaryClimberMotor1.getPIDController();
   
    m_PrimaryClimberMotor1.restoreFactoryDefaults();
    m_PrimaryClimberMotor2.restoreFactoryDefaults();
    m_SecondaryClimberMotor1.restoreFactoryDefaults();
    m_SecondaryClimberMotor2.restoreFactoryDefaults();

    m_PrimaryHomeLimitSwitch.enableLimitSwitch(true);
    m_SecondaryHomeLimitSwitch.enableLimitSwitch(true);
    
    m_SecondaryClimberMotor2.follow(m_SecondaryClimberMotor1);
    m_PrimaryClimberMotor2.follow(m_PrimaryClimberMotor1);

    m_SecondaryClimberMotor1.setInverted(true);

    m_SecondaryClimberMotor1.setIdleMode(IdleMode.kBrake);
    m_SecondaryClimberMotor2.setIdleMode(IdleMode.kBrake);
    m_PrimaryClimberMotor1.setIdleMode(IdleMode.kBrake);
    m_PrimaryClimberMotor2.setIdleMode(IdleMode.kBrake);

    m_PrimaryClimberPID.setFF(ClimberConstants.kPrimaryClimberFF);
    m_PrimaryClimberPID.setP(ClimberConstants.kPrimaryClimberP);
    m_PrimaryClimberPID.setI(ClimberConstants.kPrimaryClimberI);
    m_PrimaryClimberPID.setIZone(ClimberConstants.kPrimaryClimberIz);
    m_PrimaryClimberPID.setD(ClimberConstants.kPrimaryClimberD);
    m_PrimaryClimberPID.setSmartMotionMaxVelocity(ClimberConstants.kPrimaryClimberMaxVelocity, 0);
    m_PrimaryClimberPID.setSmartMotionMinOutputVelocity(0, 0);
    m_PrimaryClimberPID.setSmartMotionMaxAccel(ClimberConstants.kPrimaryClimberMaxAccel, 0);
    m_PrimaryClimberPID.setSmartMotionAllowedClosedLoopError(ClimberConstants.kPrimaryClimberAllowedError, 0);
    m_PrimaryClimberPID.setOutputRange(ClimberConstants.kPrimaryClimberMinOutput,
                                       ClimberConstants.kPrimaryClimberMaxOutput);

    m_SecondaryClimberPID.setFF(ClimberConstants.kSecondaryClimberFF);
    m_SecondaryClimberPID.setP(ClimberConstants.kSecondaryClimberP);
    m_SecondaryClimberPID.setI(ClimberConstants.kSecondaryClimberI);
    m_SecondaryClimberPID.setIZone(ClimberConstants.kSecondaryClimberIz);
    m_SecondaryClimberPID.setD(ClimberConstants.kSecondaryClimberD);
    m_SecondaryClimberPID.setSmartMotionMaxVelocity(ClimberConstants.kSecondaryClimberMaxVelocity, 0);
    m_SecondaryClimberPID.setSmartMotionMinOutputVelocity(0, 0);
    m_SecondaryClimberPID.setSmartMotionMaxAccel(ClimberConstants.kSecondaryClimberMaxAccel, 0);
    m_SecondaryClimberPID.setSmartMotionAllowedClosedLoopError(ClimberConstants.kSecondaryClimberAllowedError, 0);
    m_PrimaryClimberPID.setOutputRange(ClimberConstants.kSecondaryClimberMinOutput,
                                       ClimberConstants.kSecondaryClimberMaxOutput);

  }
  
  public void setClimbing(boolean isClimbing){
    climbing = isClimbing;
  }

  public boolean getClimbing(){
    return climbing;
  }

  public void setPrimaryEncoder(int pos){
    m_PrimaryEncoder.setPosition(pos);
  }

  public void setSecondaryEncoder(int pos){
    m_SecondaryEncoder.setPosition(pos);
  }

  public void setPrimaryGearDisengage(){
    m_MainSolenoid.set(false);
    MainSolonoidState = false;
  }

  public void setPrimaryGearEngage(){
    m_MainSolenoid.set(true);
    MainSolonoidState = true;
  }

  public void setSecondaryGearDisengage(){
    m_SecondSolenoid.set(false);
    SecondSolonoidState = false;
  }

  public void setSecondaryGearEngage(){
    m_SecondSolenoid.set(true);
    SecondSolonoidState = true;
  }

  public void moveSecondaryClimber(double value){
    m_SecondaryClimberMotor1.set(value);
  }

  public void movePrimaryClimber(double value){
    m_PrimaryClimberMotor1.set(value);
  }

  public boolean getPrimaryHomeLimitSwitch(){
    return m_PrimaryHomeLimitSwitch.isPressed();
  }
  public boolean getSecondaryHomeLimitSwitch(){
    return m_SecondaryHomeLimitSwitch.isPressed();
  }

  public void setPrimaryPosition(double pos){
    setPrimaryGearEngage();
    m_PrimaryClimberPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);
    PrimarySetpoint = pos;
  }

  public void setSecondaryPosition(double pos){
    setSecondaryGearEngage();
    m_SecondaryClimberPID.setReference(pos, CANSparkMax.ControlType.kSmartMotion);
    SecondarySetpoint = pos;
  }

  //enableLimitSwitches and disableLimitSwitches should be done in pairs
  public void enableLimitSwitches(){
    m_PrimaryHomeLimitSwitch.enableLimitSwitch(true);
    m_SecondaryHomeLimitSwitch.enableLimitSwitch(true);
  }
  public void disableLimitSwitches(){
    m_PrimaryHomeLimitSwitch.enableLimitSwitch(false);
    m_SecondaryHomeLimitSwitch.enableLimitSwitch(false);
  }

  //ONLY USE FOR STARTUP, skips engaging the gears
  public void setPositionsOverride(double pos){
    PrimarySetpoint = pos;
    SecondarySetpoint = pos;
    m_PrimaryClimberPID.setReference(pos, ControlType.kSmartMotion);
    m_SecondaryClimberPID.setReference(pos, ControlType.kSmartMotion);
  }

  public double getPrimarySetpoint(){
    return PrimarySetpoint;
  }
  public double getSecondarySetpoint(){
    return SecondarySetpoint;
  }

  public double getPrimaryPosition(){
    return m_PrimaryEncoder.getPosition();
  }
  public double getSecondaryPosition(){
    return m_SecondaryEncoder.getPosition();
  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("SecondaryClutch", SecondSolonoidState);
    SmartDashboard.putBoolean("PrimaryClutch", MainSolonoidState);
    // SmartDashboard.putBoolean("isClimbing", getClimbing());
    // SmartDashboard.putBoolean("Primaryhomelimitswitch", getPrimaryHomeLimitSwitch());
    // SmartDashboard.putNumber("PrimaryEncoder", getPrimaryPosition());
    // SmartDashboard.putNumber("PrimaryAppliedOutput", m_PrimaryClimberMotor1.getAppliedOutput());
    
    // SmartDashboard.putBoolean("Secondaryhomelimitswitch", getSecondaryHomeLimitSwitch());
    // SmartDashboard.putNumber("SecondaryEncoder", getSecondaryPosition());
    // SmartDashboard.putNumber("SecondaryAppliedOutput", m_SecondaryClimberMotor1.getAppliedOutput());

    // int Ticks = ThroughBore.get(); // 2000 ticks are about a rotation.
    // SmartDashboard.putNumber("ThroughBore", Ticks);
    // moveClimber(0.1);
    // // This method will be called once per schedule;r run
  }
}

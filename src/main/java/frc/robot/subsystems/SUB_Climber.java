// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
public class SUB_Climber extends SubsystemBase {
 
  private final Solenoid m_SecondSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.kSecondSolonoid);
  private final Solenoid m_MainSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.kMainSolonoid);
 
  public CANSparkMax m_SecondaryClimberMotor2 = new CANSparkMax(ClimberConstants.kSecondaryClimberMotor2,MotorType.kBrushless);
  public CANSparkMax m_SecondaryClimberMotor1 = new CANSparkMax(ClimberConstants.kSecondaryClimberMotor1,MotorType.kBrushless);
  public CANSparkMax m_PrimaryClimberMotor2 = new CANSparkMax(ClimberConstants.kPrimaryClimberMotor2,MotorType.kBrushless);
  public CANSparkMax m_PrimaryClimberMotor1 = new CANSparkMax(ClimberConstants.kPrimaryClimberMotor1,MotorType.kBrushless);
 
  public SparkMaxLimitSwitch m_PrimaryHomeLimitSwitch = m_PrimaryClimberMotor1.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  
  public boolean MainSolonoidState = false;
  public boolean SecondSolonoidState = false;
  
  private SparkMaxPIDController m_PrimaryClimberPID = m_PrimaryClimberMotor1.getPIDController();
  private SparkMaxPIDController m_SecondaryClimberPID = m_SecondaryClimberMotor1.getPIDController();
  /** Creates a new SUB_Climber. */
  public SUB_Climber() {
    m_PrimaryHomeLimitSwitch.enableLimitSwitch(true);

    m_SecondaryClimberMotor2.follow(m_SecondaryClimberMotor1);
    m_PrimaryClimberMotor2.follow(m_PrimaryClimberMotor1);

    m_SecondaryClimberMotor1.setIdleMode(IdleMode.kBrake);
    m_SecondaryClimberMotor2.setIdleMode(IdleMode.kBrake);
    m_PrimaryClimberMotor1.setIdleMode(IdleMode.kBrake);
    m_PrimaryClimberMotor2.setIdleMode(IdleMode.kBrake);

    m_PrimaryClimberPID.setFF(0.00001);
    m_PrimaryClimberPID.setP(0);
    m_PrimaryClimberPID.setI(0);
    m_PrimaryClimberPID.setD(0);

    m_SecondaryClimberPID.setFF(0);
    m_SecondaryClimberPID.setP(0);
    m_SecondaryClimberPID.setI(0);
    m_SecondaryClimberPID.setD(0);
  }

  public void setPrimaryEncoder(int pos){
    m_PrimaryClimberMotor1.getEncoder().setPosition(pos);
  }

  public void setPrimaryGearDisengage(){
    m_MainSolenoid.set(false);
    MainSolonoidState = false;
  }

  public void setPrimaryGearEngage(){
    m_MainSolenoid.set(true);
    MainSolonoidState = true;
  }

  public void setRetractSecond(){
    m_SecondSolenoid.set(false);
    SecondSolonoidState = false;
  }

  public void setExtendSecond(){
    m_SecondSolenoid.set(true);
    SecondSolonoidState = true;
  }

  public void moveSecondaryClimber(double value){
    m_SecondaryClimberMotor1.set(value);
  }
  public void movePrimaryClimber(double value){
    m_PrimaryClimberMotor1.set(value);
  }
  public void setPrimaryPosition(double value){
    
  }

  public boolean getPrimaryHomeLimitSwitch(){
    return m_PrimaryHomeLimitSwitch.isPressed();
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("SecondaryClutch", SecondSolonoidState);
    SmartDashboard.putBoolean("PrimaryClutch", MainSolonoidState);
    SmartDashboard.putBoolean("primaryhomelimitswitch", getPrimaryHomeLimitSwitch());
    SmartDashboard.putNumber("Climber Voltage", m_PrimaryClimberMotor1.getAppliedOutput());
    // int Ticks = ThroughBore.get(); // 2000 ticks are about a rotation.
    // SmartDashboard.putNumber("ThroughBore", Ticks);
    // moveClimber(0.1);
    // // This method will be called once per schedule;r run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CLimberConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
public class SUB_Climber extends SubsystemBase {
  Encoder ThroughBore1;
  Encoder ThroughBore2;
  private final Solenoid m_SecondSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, CLimberConstants.kSecondSolonoid);
  private final Solenoid m_MainSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, CLimberConstants.kMainSolonoid);
  public CANSparkMax m_SecondaryClimberMotor2= new CANSparkMax(CLimberConstants.kSecondaryClimberMotor2,MotorType.kBrushless);
  public CANSparkMax m_SecondaryClimberMotor1= new CANSparkMax(CLimberConstants.kSecondaryClimberMotor1,MotorType.kBrushless);
  public CANSparkMax m_PrimaryClimberMotor2= new CANSparkMax(CLimberConstants.kPrimaryClimberMotor2,MotorType.kBrushless);
  public CANSparkMax m_PrimaryClimberMotor1= new CANSparkMax(CLimberConstants.kPrimaryClimberMotor1,MotorType.kBrushless);
  public boolean MainSolonoidState = false;
  public boolean SecondSolonoidState = false;
  /** Creates a new SUB_Climber. */
  public SUB_Climber() {
    ThroughBore1 = new Encoder(2,3);
    m_SecondaryClimberMotor2.follow(m_SecondaryClimberMotor1);
    m_PrimaryClimberMotor2.follow(m_PrimaryClimberMotor1);
    m_SecondaryClimberMotor1.setIdleMode(IdleMode.kBrake);
    m_SecondaryClimberMotor2.setIdleMode(IdleMode.kBrake);
  }
  public void setRetractMain(){
    m_MainSolenoid.set(false);
    MainSolonoidState = false;
  }
  public void setExtendMain(){
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


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("SecondaryClutch", SecondSolonoidState);
    SmartDashboard.putBoolean("PrimaryClutch", MainSolonoidState);
    // int Ticks = ThroughBore.get(); // 2000 ticks are about a rotation.
    // SmartDashboard.putNumber("ThroughBore", Ticks);
    // moveClimber(0.1);
    // // This method will be called once per schedule;r run
  }
}

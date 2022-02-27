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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
public class SUB_Climber extends SubsystemBase {
  Encoder ThroughBore;
  private final Solenoid m_SecondSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, CLimberConstants.kSecondSolonoid);
  private final Solenoid m_MainSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, CLimberConstants.kMainSolonoid);
  public CANSparkMax m_ClimberMotor2= new CANSparkMax(CLimberConstants.kClimberMotor2,MotorType.kBrushless);
  public CANSparkMax m_ClimberMotor1= new CANSparkMax(CLimberConstants.kClimberMotor1,MotorType.kBrushless);
  
  
  /** Creates a new SUB_Climber. */
  public SUB_Climber() {
    ThroughBore = new Encoder(2,3);
    m_ClimberMotor2.follow(m_ClimberMotor1, false);
    m_ClimberMotor1.setIdleMode(IdleMode.kBrake);
    m_ClimberMotor2.setIdleMode(IdleMode.kBrake);
  }
  public void setRetractMain(){
    m_MainSolenoid.set(false);
  }
  public void setExtendMain(){
    m_MainSolenoid.set(true);
  }
  public void setRetractSecond(){
    m_SecondSolenoid.set(false);
  }
  public void setExtendSecond(){
    m_SecondSolenoid.set(true);
  }
  public void setClimberForward(){
    m_ClimberMotor1.set(0.3);
  }
  public void moveClimber(double value){

    m_ClimberMotor1.set(value);
  }


  @Override
  public void periodic() {
    // int Ticks = ThroughBore.get(); // 2000 ticks are about a rotation.
    // SmartDashboard.putNumber("ThroughBore", Ticks);
    // moveClimber(0.1);
    // // This method will be called once per schedule;r run
  }
}

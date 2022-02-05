// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;



/** Add your docs here. */
public class SUB_Intake extends SubsystemBase {
    // private final Solenoid m_IntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,1);
    private CANSparkMax m_IntakeMotor= new CANSparkMax(ShooterConstants.kIntake,MotorType.kBrushless);
    private RelativeEncoder m_IntakeEncoder = m_IntakeMotor.getEncoder();

    public SUB_Intake() { 
        
    }
    
    // public void setDeployIntake(){
    //     m_IntakeSolenoid.set(true);
    // }
    
    // public void setRetractIntake(){
    //     m_IntakeSolenoid.set(false);
    // }

    public void setIntakeForward(){
        m_IntakeMotor.set(ShooterConstants.IntakeForward);
    }

    public void setIntakeReverse(){
        m_IntakeMotor.set(ShooterConstants.IntakeReverse);
    }
    public void setIntakeOff(){
        m_IntakeMotor.set(ShooterConstants.IntakeOff);
    }
    public double getVelocity(){
        return m_IntakeEncoder.getVelocity();
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("IntakeRPM",getVelocity());
    }
}
        
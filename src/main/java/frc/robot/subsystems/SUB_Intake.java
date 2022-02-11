// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Util.DigitalSensor;



/** Add your docs here. */
public class SUB_Intake extends SubsystemBase {
    // private final Solenoid m_IntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,1);
    public CANSparkMax m_FrontIntakeMotor= new CANSparkMax(IndexerConstants.kFrontIntake,MotorType.kBrushless);
    public CANSparkMax m_BackIntakeMotor= new CANSparkMax(IndexerConstants.kBackIntake,MotorType.kBrushless);
    private RelativeEncoder m_FrontIntakeEncoder = m_FrontIntakeMotor.getEncoder();
    private RelativeEncoder m_BackIntakeEncoder = m_BackIntakeMotor.getEncoder();
    public DigitalSensor m_FrontIntakeSensor = new DigitalSensor(IndexerConstants.kFrontIntakeIR);
    private DigitalSensor m_BackIntakeSensor = new DigitalSensor(IndexerConstants.kBackIntakeIR);
    public SUB_Intake() { 
    
    }
    
    // public void setDeployIntake(){
    //     m_IntakeSolenoid.set(true);
    // }
    
    // public void setRetractIntake(){
    //     m_IntakeSolenoid.set(false);
    // }

    public void setFrontIntakeForward(){
        m_FrontIntakeMotor.set(IndexerConstants.IntakeForward);
    }

    public void setFrontIntakeReverse(){
        m_FrontIntakeMotor.set(IndexerConstants.IntakeReverse);
    }
    public void setFrontIntakeOff(){
        m_FrontIntakeMotor.set(IndexerConstants.IntakeOff);
    }
    public double getFrontVelocity(){
        return m_FrontIntakeEncoder.getVelocity();
    }
    public boolean getFrontIntakeStatus(){
        boolean frontIntakeStatus = m_FrontIntakeSensor.get();
        return frontIntakeStatus;
      }

      public void setBackIntakeForward(){
        m_FrontIntakeMotor.set(IndexerConstants.IntakeForward);
    }
    public void setBackIntakeReverse(){
        m_FrontIntakeMotor.set(IndexerConstants.IntakeReverse);
    }
    public void setBackIntakeOff(){
        m_FrontIntakeMotor.set(IndexerConstants.IntakeOff);
    }
    public double getBackVelocity(){
        return m_BackIntakeEncoder.getVelocity();
    }
    public boolean getBackIntakeStatus(){
        boolean backIntakeStatus = m_BackIntakeSensor.get();
        return backIntakeStatus;
      }
    public boolean getIntakeStatus(){
        boolean intakeStatus;
        if (getBackIntakeStatus() == true){
            return true;
        }else if (getFrontIntakeStatus() == true){
            return true;
        }else {
            return false;
        }
    }



    @Override
    public void periodic() {
        // SmartDashboard.putNumber("IntakeRPM",getVelocity());
    }
}
        
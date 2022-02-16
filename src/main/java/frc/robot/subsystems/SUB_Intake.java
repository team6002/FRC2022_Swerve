// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Util.TRG_Intake;
import frc.robot.subsystems.FSM_IntakeStatus.State;



/** Add your docs here. */
public class SUB_Intake extends SubsystemBase {
    private FSM_IntakeStatus m_IntakeStatus;

    public CANSparkMax m_FrontIntakeMotor= new CANSparkMax(IndexerConstants.kFrontIntake,MotorType.kBrushless);
    public CANSparkMax m_BackIntakeMotor= new CANSparkMax(IndexerConstants.kBackIntake,MotorType.kBrushless);
    private CANSparkMax m_HopperMotor= new CANSparkMax(IndexerConstants.kHopper,MotorType.kBrushless);
    private CANSparkMax m_IndexerMotor= new CANSparkMax(IndexerConstants.kIndexer,MotorType.kBrushless);
   
    public DigitalInput m_FrontIntakeSensor = new DigitalInput(IndexerConstants.kFrontIntakeIR);
    private DigitalInput m_BackIntakeSensor = new DigitalInput(IndexerConstants.kBackIntakeIR);
    private DigitalInput m_HopperSensor = new DigitalInput(IndexerConstants.kHopperIR);

    private RelativeEncoder m_FrontIntakeEncoder = m_FrontIntakeMotor.getEncoder();
    private RelativeEncoder m_BackIntakeEncoder = m_BackIntakeMotor.getEncoder();
  
    public double frontIntakeState;
    public double backIntakeState;
    public double indexerState;
    public double hopperState;


    public SUB_Intake(FSM_IntakeStatus p_IntakeStatus) {
    m_IntakeStatus = p_IntakeStatus;
      
    m_HopperMotor.setInverted(true);
    m_IndexerMotor.setInverted(true);
    }
    
    public void setFrontIntakeForward(){
        m_FrontIntakeMotor.set(IndexerConstants.IntakeForward);
        frontIntakeState = 1;
    }

    public void setFrontIntakeReverse(){
        m_FrontIntakeMotor.set(IndexerConstants.IntakeReverse);
        frontIntakeState = -1;
    }
    public void setFrontIntakeOff(){
        m_FrontIntakeMotor.set(IndexerConstants.IntakeOff);
        frontIntakeState = 0;
    }
    public double getFrontVelocity(){
        return m_FrontIntakeEncoder.getVelocity();
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


  public void setHopperOff(){
    m_HopperMotor.set(IndexerConstants.HopperOff);
    hopperState = 0;
  }

  public void setHopperForward(){
    m_HopperMotor.set(IndexerConstants.HopperForward);
    hopperState = 1;
  }

  public void setHopperBackward(){
    m_HopperMotor.set(IndexerConstants.HopperBackward);
    hopperState = -1;
  }
  
  public void setIndexerOff(){
    m_IndexerMotor.set(IndexerConstants.IndexerOff);
    indexerState = 0;
  }
  
  public void setIndexerForward(){
    m_IndexerMotor.set(IndexerConstants.IndexerForward);
    indexerState = 1;
  }

  public void setIndexerBackward(){
    m_IndexerMotor.set(IndexerConstants.IndexerBackward);
    indexerState = -1;
  }
  public boolean getHopperStatus(){
    return m_HopperSensor.get();
  }





    @Override
    public void periodic() {
      if (m_IntakeStatus.getState(State.SHOOTING)){

      }else 
      if (getHopperStatus()){
        setHopperOff();
      }
      SmartDashboard.putBoolean("HopperStatus", getHopperStatus());
    }
}
        
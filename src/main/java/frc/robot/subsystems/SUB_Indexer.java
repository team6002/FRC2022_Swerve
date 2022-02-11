// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Util.DigitalSensor;



public class SUB_Indexer extends SubsystemBase {

  private CANSparkMax m_HopperMotor= new CANSparkMax(IndexerConstants.kHopper,MotorType.kBrushless);
  private CANSparkMax m_IndexerMotor= new CANSparkMax(IndexerConstants.kIndexer,MotorType.kBrushless);
  private DigitalSensor m_HopperSensor = new DigitalSensor(IndexerConstants.kHopperIR);
  public boolean hopperStatus;
  /** Creates a new SUB_Indexer. */
  public SUB_Indexer() {  
  }

  public void setHopperOff(){
    m_HopperMotor.set(IndexerConstants.HopperOff);
  }

  public void setHopperForward(){
    m_HopperMotor.set(IndexerConstants.HopperForward);
  }

  public void setHopperBackward(){
    m_HopperMotor.set(IndexerConstants.HopperBackward);
  }
  
  public void setIndexerOff(){
    m_IndexerMotor.set(IndexerConstants.IndexerOff);
  }
  
  public void setIndexerForward(){
    m_IndexerMotor.set(IndexerConstants.IndexerForward);
  }

  public void setIndexerBackward(){
    m_IndexerMotor.set(IndexerConstants.IndexerBackward);
  }
  public boolean getHopperStatus(){
    boolean hopperStatus = m_HopperSensor.getAsBoolean();
    return hopperStatus;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

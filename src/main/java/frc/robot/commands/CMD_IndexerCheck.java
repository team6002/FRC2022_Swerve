// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Indexer;
import frc.robot.subsystems.SUB_Intake;
public class CMD_IndexerCheck extends CommandBase {
  private SUB_Indexer m_Indexer;
  private SUB_Intake m_Intake;
  /** Creates a new CMD_HopperFull. */
  public CMD_IndexerCheck(SUB_Indexer p_Indexer, SUB_Intake p_Intake) {
    m_Indexer = p_Indexer;
    m_Intake = p_Intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_Indexer.getHopperStatus() == true){
      m_Indexer.setHopperOff();
    }else if (m_Indexer.getHopperStatus() == true && m_Intake.getIntakeStatus() == true ){
      m_Indexer.setHopperOff();
      m_Intake.setFrontIntakeOff();
      m_Intake.setBackIntakeOff();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

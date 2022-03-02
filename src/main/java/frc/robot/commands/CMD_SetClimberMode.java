// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FSM_ClimberMode;
import frc.robot.subsystems.FSM_ClimberMode.ClimberState;
public class CMD_SetClimberMode extends CommandBase {
  /** Creates a new CMD_SetIntakeStatus. */
  FSM_ClimberMode m_ClimberMode;
  ClimberState m_wantedStatus;
 
  public CMD_SetClimberMode(FSM_ClimberMode p_ClimberMode, ClimberState p_wantedStatus) {
    m_ClimberMode = p_ClimberMode;
    m_wantedStatus = p_wantedStatus;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ClimberMode.setState(m_wantedStatus);
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
    return true;
  }
}

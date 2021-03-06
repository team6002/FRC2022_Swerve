// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Turret;

public class CMD_setTurretMode extends CommandBase {
  /** Creates a new CMD_setTurretMode. */
  SUB_Turret m_turret;
  int m_wantedTurretMode;
  public CMD_setTurretMode(int p_wantedTurretMode, SUB_Turret p_turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = p_turret;
    m_wantedTurretMode = p_wantedTurretMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.setTurretMode(m_wantedTurretMode);
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

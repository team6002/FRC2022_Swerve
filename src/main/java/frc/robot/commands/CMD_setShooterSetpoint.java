// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_setShooterSetpoint extends CommandBase {
  /** Creates a new CMD_setShooterSetpoint. */
  SUB_Shooter m_shooter;
  double m_wantedShooterValue;
  public CMD_setShooterSetpoint(SUB_Shooter p_shooter, double p_wantedShooterValue) {
    m_shooter = p_shooter;
    m_wantedShooterValue = p_wantedShooterValue;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setShooterSetpoint(m_wantedShooterValue);
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

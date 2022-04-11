// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_Turret;

public class CMD_ChangeShooterMode extends CommandBase {
  /** Creates a new CMD_ChangeShooterMode. */
  SUB_Shooter m_shooter;
  SUB_Turret m_turret;
  public CMD_ChangeShooterMode(SUB_Shooter p_shooter, SUB_Turret p_turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = p_shooter;
    m_turret = p_turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_shooter.getShooterMode()<2 ){
    m_shooter.setShooterMode(m_shooter.getShooterMode() + 1);
    }else{
      m_shooter.setShooterMode(0);
    }
    if (m_shooter.getShooterMode() != m_shooter.getPreviousShooterMode()){
      if (m_shooter.getShooterMode() == 0){
        m_turret.setTurretMode(0);
      }else {
        m_turret.setTurretMode(1);
        m_turret.setSidePosition();
      }
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
    return true;
  }
}

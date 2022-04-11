// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;
public class CMD_BallCountCheck extends CommandBase {
  /** After Shooting */
  SUB_Intake m_intake;
  SUB_Shooter m_shooter;
  public CMD_BallCountCheck(SUB_Intake p_intake, SUB_Shooter p_shooter) {
    m_intake = p_intake;
    m_shooter = p_shooter;
    // Use addRequirements() here to declare subsystem dependencies.      
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_intake.getHopperStatus() && m_intake.getFrontStatus() || m_intake.getBackStatus()){
      m_shooter.setFirstBall(true);
    }else{
      m_shooter.setFirstBall(false);
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

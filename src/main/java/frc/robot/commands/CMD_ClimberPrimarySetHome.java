// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Climber;

public class CMD_ClimberPrimarySetHome extends CommandBase {
  /** Creates a new CMD_ClimberPrimarySetHome. */
  SUB_Climber m_climber;
  private boolean m_disengageOnCompletion;
  public CMD_ClimberPrimarySetHome(SUB_Climber p_climber, boolean p_disengageOnCompletion) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = p_climber;
    m_disengageOnCompletion = p_disengageOnCompletion;
    // addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.setPrimaryGearEngage();
    m_climber.movePrimaryClimber(-0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("FINISHED HOMING PRIMARY CLIMBER");
    m_climber.setPrimaryEncoder(0); //reset encoder
    m_climber.movePrimaryClimber(0); //stop moving
    if(m_disengageOnCompletion){
      m_climber.setPrimaryGearDisengage(); //latch
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.getPrimaryHomeLimitSwitch();
  }
}

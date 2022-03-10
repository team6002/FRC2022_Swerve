// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.SUB_Climber;

public class CMD_ClimbDeploySecondaryClimber extends CommandBase {
  /** Creates a new CMD_ClimbDeploySecondaryClimber. */
  SUB_Climber m_climber;
  public CMD_ClimbDeploySecondaryClimber(SUB_Climber p_climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = p_climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setSecondaryPosition(ClimberConstants.SecondaryClimberDeploy);
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

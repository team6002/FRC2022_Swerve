// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.SUB_Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_ClimberLevitate extends InstantCommand {
  //Used after CMD_InitalizeClimbMode, lifts the robot a few inches off the ground
  SUB_Climber m_climber;
  public CMD_ClimberLevitate(SUB_Climber p_climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = p_climber;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.setPrimaryPosition(ClimberConstants.PrimaryClimberLevitate);
  }
}

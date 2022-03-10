// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SUB_Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_InitalizeClimbMode extends ParallelCommandGroup {
  /** 
   * Deploys the primary and secondary arms 
  */
  SUB_Climber m_climber;
  public CMD_InitalizeClimbMode(SUB_Climber p_climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_climber = p_climber;
    addCommands(
      new SequentialCommandGroup(
        new CMD_ClimberPrimarySetHome(m_climber, false),
        new CMD_ClimbDeployPrimaryClimber(m_climber)
      ),
      new SequentialCommandGroup(
        new CMD_ClimberSecondarySetHome(m_climber, false),
        new CMD_ClimbDeploySecondaryClimber(m_climber)
      )
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.SUB_Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_ClimbFull extends SequentialCommandGroup {
  /** 
   * This is the partial climb manuever to hang on the 10 pts bar.
   * Also prepares for the full climb manuever.
  */
  SUB_Climber m_climber;
  public CMD_ClimbFull(SUB_Climber p_climber) {
    m_climber = p_climber;
    addCommands(
      new CMD_NotReadyToUnlatch(m_climber)
      ,new CMD_ClimberPrimaryArmMove(m_climber, 0, 1) //Climber lift to 1st bar
      ,new CMD_ClimberSecondarySolonoidExtend(m_climber)
      ,new WaitCommand(0.1)
      ,new CMD_ClimberSecondarySolonoidRetract(m_climber)
      // ,new CMD_ClimberSecondaryArmMove(m_climber, 28.809, 0.5) //Climber captures 2nd bar
      // ,new CMD_ClimberSecondaryArmMove(m_climber, 32.0, 0.5) //Climber captures 2nd bar
      // ,new ParallelCommandGroup(
        ,new CMD_ClimberSecondaryArmMove(m_climber, 33.5, 0.5),
        // new SequentialCommandGroup(
          // new WaitCommand(0.70),
          new CMD_ClimberPrimaryArmMove(m_climber, 20.0, 0.5)
        // ) //The partial release
      // ,new CMD_ClimberPrimaryArmMove(m_climber, 20.0, 0.5)
      // )//major swing alert WEEEEEEEEEEEEE
      // ,new ParallelCommandGroup(       
      ,new CMD_ClimberSecondaryArmMove(m_climber,0, 0.1).withTimeout(3)
      // new SequentialCommandGroup(2)
      // , new CMD_ClimberPrimaryArmMove(m_climber, 34, 0.5)
      ,new CMD_ClimbDeployPrimaryClimber(m_climber)
      // )
      , new WaitCommand(0.5)
      , new CMD_ClimberSecondaryArmMove(m_climber, 1, 0.5).withTimeout(3)
      , new WaitCommand(0.5)
      , new CMD_ClimberSecondaryArmMove(m_climber, 2, 0.5).withTimeout(3)
      , new WaitCommand(0.5)
      , new CMD_ClimberSecondaryArmMove(m_climber, 5,0.5).withTimeout(3)
      , new CMD_ReadyToUnlatch(m_climber)
      
      
    );
  }
}

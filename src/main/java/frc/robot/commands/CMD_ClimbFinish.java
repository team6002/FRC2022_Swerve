// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SUB_Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_ClimbFinish extends SequentialCommandGroup {
  /** After a partial climb the robot goes for the full climb 
   * Positions after partial climb:
   * Primary Climber - 70
   * Secondary Climber - 110
   * This file is just for reference of what full should do after partial
   * NOT PART OF CLIMB
   * NOT PART OF CLIMB
   * NOT PART OF CLIMB
  */
  SUB_Climber m_climber;
  public CMD_ClimbFinish(SUB_Climber p_climber) {
    m_climber = p_climber;

    addCommands(
      new CMD_ClimberSecondaryArmMove(m_climber, 40, 1),//pull robot in
      new CMD_ClimberPrimaryArmMove(m_climber, 100, 1), //put primary arm behind 2nd bar
      new CMD_ClimberSecondaryArmMove(m_climber, 85,1), //lean primary arm against 2nd bar
      // new WaitCommand(0.1),
      new CMD_ClimberPrimaryArmMove(m_climber, 40, 1),  //swap hands on 2nd bar
      //REPEAT PARTIAL CLIMB
      new CMD_ClimberSecondaryArmMove(m_climber, 115, 1), //lay back the 2nd arm before repeating partial
      new CMD_ClimberPrimaryArmMove(m_climber, 0, 1), //Climber lift to 1st bar
      new WaitCommand(0.1),
      new CMD_ClimberSecondaryArmMove(m_climber, 97, 1), //Climber captures 2nd bar
      new ParallelCommandGroup(
        new CMD_ClimberPrimaryArmMove(m_climber, 10, 2), //Start rocking down
        new CMD_ClimberSecondaryArmMove(m_climber, 105, 2)
      ),
      new ParallelCommandGroup(
        new CMD_ClimberPrimaryArmMove(m_climber, 20, 2),
        new CMD_ClimberSecondaryArmMove(m_climber, 110, 2)
      ),
      new CMD_ClimberPrimaryArmMove(m_climber, 30, 2),
      new CMD_ClimberPrimaryArmMove(m_climber, 70, 2) //The release
    );
  }
}

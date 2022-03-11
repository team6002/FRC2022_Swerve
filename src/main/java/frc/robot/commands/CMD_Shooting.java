// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FSM_IntakeStatus;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_Turret;
import frc.robot.subsystems.FSM_IntakeStatus.IntakeState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_Shooting extends SequentialCommandGroup {
  /** Creates a new CMD_Shooting. */

  SUB_Intake m_intake;
  FSM_IntakeStatus m_intakeStatus;
  SUB_Shooter m_shooter;
  public CMD_Shooting(SUB_Intake p_intake, FSM_IntakeStatus p_intakeStatus, 
                    SUB_Shooter p_shooter) {//SUB_Turret pTurret, 

    m_intake = p_intake;
    m_intakeStatus = p_intakeStatus;
    m_shooter = p_shooter;
    // m_intakeStatus.setState(IntakeState.SHOOTING);
    addCommands(
      new CMD_SetIntakeStatus(p_intakeStatus, IntakeState.SHOOTING),
      new CMD_ShooterOn(m_shooter),
      new CMD_IndexerForward(m_intake),
      new CMD_HopperForward(m_intake)
      // new CMD_HopperLoad(m_intake)
      
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.FSM_IntakeStatus;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_Turret;
import frc.robot.subsystems.FSM_IntakeStatus.IntakeState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_ShortShooting extends SequentialCommandGroup {
  /** Shoots every ball that we are holding, emptying out the hopper in the process */

  SUB_Intake m_intake;
  FSM_IntakeStatus m_intakeStatus;
  SUB_Shooter m_shooter;
  SUB_Turret m_turret;
  public CMD_ShortShooting(SUB_Intake p_intake, FSM_IntakeStatus p_intakeStatus, 
                    SUB_Shooter p_shooter, SUB_Turret p_turret) {//SUB_Turret pTurret, 
    m_intake = p_intake;
    m_intakeStatus = p_intakeStatus;
    m_shooter = p_shooter;
    m_turret = p_turret;
    // addRequirements(m_shooter);
    
    // m_intakeStatus.setState(IntakeState.SHOOTING);
    addCommands(
      new CMD_setTurretMode(1,m_turret)
      ,new CMD_ShooterManualMode(m_shooter)
      ,new CMD_SideTurret(m_turret)
      ,new CMD_ShooterHoodRetract(p_shooter)
      ,new CMD_SetIntakeStatus(p_intakeStatus, IntakeState.SHOOTING),
      new PrintCommand("changed state to shooter")
      ,new CMD_setShooterSetpoint(m_shooter, 2250)
      ,new CMD_ShooterOn(m_shooter),
      new PrintCommand("turned on shooter"),
      new CMD_IndexerForward(m_intake),
      new PrintCommand("turned on indexer"),
      new CMD_HopperForward(m_intake),
      new PrintCommand("turned on hopper"),
      // new WaitCommand(0.75),
      new CMD_HopperCheck(m_intake),
      new PrintCommand("checked hopper")
      ,new WaitCommand(0.35)
      ,new CMD_HopperOff(m_intake)
      ,new WaitCommand(0.5)
      ,new CMD_setShooterSetpoint(m_shooter, 2375)
      ,new CMD_HopperForward(m_intake)
      ,new CMD_SetIntakeStatus(p_intakeStatus, IntakeState.INTAKE)
      ,new PrintCommand("changed state to intake")
      );
  }
}
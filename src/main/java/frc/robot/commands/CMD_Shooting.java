// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FSM_IntakeStatus;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.FSM_IntakeStatus.IntakeState;

public class CMD_Shooting extends CommandBase {
  SUB_Shooter m_shooter;
  SUB_Intake m_intake;
  FSM_IntakeStatus m_intakeStatus;
  /** Creates a new CMD_Shooting. */
  public CMD_Shooting(SUB_Intake p_intake, SUB_Shooter p_shooter, FSM_IntakeStatus p_intakeStatus) {
    m_shooter = p_shooter;
    m_intake = p_intake;
    m_intakeStatus = p_intakeStatus;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new SequentialCommandGroup( 
      new CMD_SetIntakeStatus(m_intakeStatus, IntakeState.SHOOTING),
    new CMD_ShooterOn(m_shooter),
    new CMD_IndexerForward(m_intake),
    new CMD_HopperForward(m_intake),
    new CMD_BackIntakeForward(m_intake),
    new CMD_FrontIntakeForward(m_intake),
    new CMD_HopperCheck(m_intake),
    new CMD_IndexerOff(m_intake),
    new CMD_ShooterOff(m_shooter),
    new CMD_SetIntakeStatus(m_intakeStatus, IntakeState.INTAKE)
    );
   
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

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
  public CMD_Shooting(SUB_Turret pTurret, SUB_Intake pIntake, 
                    FSM_IntakeStatus pIntakeStatus, SUB_Shooter pShooter) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_SetIntakeStatus(pIntakeStatus, IntakeState.SHOOTING),
      new CMD_ShooterOn(pShooter),
      new CMD_IndexerForward(pIntake),
      new CMD_HopperForward(pIntake),
      new CMD_BackIntakeForward(pIntake),
      new CMD_FrontIntakeForward(pIntake),
      new CMD_FrontIntakeOff(pIntake),
      new CMD_BackIntakeOff(pIntake)
      // new CMD_HopperCheck(m_intake),
      // new CMD_IndexerOff(m_intake),
      // new CMD_SetIntakeStatus(m_intakeStatus, IntakeState.INTAKE)
      
    );
  }
}

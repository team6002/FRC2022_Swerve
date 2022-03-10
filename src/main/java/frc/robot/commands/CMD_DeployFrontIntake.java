// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FSM_IntakeStatus;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.FSM_IntakeStatus.IntakeState;

public class CMD_DeployFrontIntake extends SequentialCommandGroup {
  /** Delpoys front intake */
  public CMD_DeployFrontIntake(SUB_Intake p_intake, FSM_IntakeStatus p_intakeStatus) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_FrontIntakeForward(p_intake),
      new CMD_HopperForward(p_intake),
      new CMD_FrontSolonoidExtend(p_intake),
      new CMD_SetIntakeStatus(p_intakeStatus, IntakeState.INTAKE)
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FSM_IntakeStatus;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.FSM_IntakeStatus.IntakeState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_DeployBackIntake extends SequentialCommandGroup {
  /** Deploy Back Intake*/
  public CMD_DeployBackIntake(SUB_Intake p_intake, FSM_IntakeStatus p_intakeStatus) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_BackSolonoidExtend(p_intake),
      new CMD_HopperForward(p_intake),
      new CMD_BackIntakeForward(p_intake),
      new CMD_SetIntakeStatus(p_intakeStatus, IntakeState.INTAKE)
    );
  }
}

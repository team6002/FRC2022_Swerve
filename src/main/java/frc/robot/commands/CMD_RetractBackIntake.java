// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SUB_Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_RetractBackIntake extends SequentialCommandGroup {
  /** Pulls the Back Intake back in */
  public CMD_RetractBackIntake(SUB_Intake p_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_BackSolonoidRetract(p_intake),
      new CMD_BackIntakeOff(p_intake),
      new CMD_HopperOff(p_intake)
      //INTAKESTATE BACK RETRACTED?
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class CMD_DrivetrainReset extends CommandBase {
  /** Creates a new CMD_DrivetrainReset. */
  SwerveDrivetrain m_drivetrain;
  public CMD_DrivetrainReset(SwerveDrivetrain p_drivetrain) {
    m_drivetrain = p_drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.syncAllAngles();
    m_drivetrain.zeroHeading();
    m_drivetrain.resetDriveEncoder();
    m_drivetrain.resetOdometry(new Pose2d(0,0, new Rotation2d(0)));
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_Turret;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.FSM_IntakeStatus;
import frc.robot.commands.CMD_BackIntakeDeploy;
import frc.robot.commands.CMD_BackIntakeRetract;
import frc.robot.commands.CMD_BackIntakeToggle;
import frc.robot.commands.CMD_DrivetrainReset;
import frc.robot.commands.CMD_FrontIntakeDeploy;
import frc.robot.commands.CMD_FrontIntakeRetract;
import frc.robot.commands.CMD_FrontIntakeToggle;
import frc.robot.commands.CMD_Shooting;
import frc.robot.commands.CMD_SpinInPlace;
import frc.robot.commands.CMD_StopShooting;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AUTO_4balls extends SequentialCommandGroup {
  SwerveDrivetrain m_drivetrain;
  SUB_Intake m_intake;
  FSM_IntakeStatus m_IntakeStatus;
  AUTO_Trajectory m_trajectory;
  SUB_Shooter m_shooter;
  SUB_Turret m_turret;
  /** Creates a new AUTO_4balls. */
  public AUTO_4balls(SwerveDrivetrain p_drivetrain, SUB_Intake p_intake, FSM_IntakeStatus p_IntakeStatus, 
                            AUTO_Trajectory p_trajectory, SUB_Shooter p_shooter, SUB_Turret p_turret) {
    m_drivetrain = p_drivetrain;
    m_intake = p_intake;
    m_IntakeStatus = p_IntakeStatus;
    m_trajectory = p_trajectory;
    m_shooter = p_shooter;
    m_turret = p_turret;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(  
      new SequentialCommandGroup(
      new CMD_DrivetrainReset(m_drivetrain),
      new CMD_FrontIntakeDeploy(m_intake, m_IntakeStatus),
      m_trajectory.driveTrajectory(m_trajectory.firstTrajectory),
      new CMD_Shooting(m_intake, m_IntakeStatus, m_shooter,m_turret).withTimeout(3),
      new CMD_FrontIntakeRetract(m_intake, m_IntakeStatus),
      new CMD_BackIntakeDeploy(m_intake, m_IntakeStatus),
      m_trajectory.driveTrajectory(m_trajectory.secondTrajectory),
      new CMD_Shooting(m_intake, m_IntakeStatus, m_shooter,m_turret).withTimeout(3),
      new CMD_Shooting(m_intake, m_IntakeStatus, m_shooter,m_turret).withTimeout(3),
      m_trajectory.driveTrajectory(m_trajectory.thirdTrajectory),
      new WaitCommand(0.5),
      m_trajectory.driveTrajectory(m_trajectory.fourthTrajectory),
      new CMD_Shooting(m_intake, m_IntakeStatus, m_shooter,m_turret).withTimeout(3),
      new CMD_Shooting(m_intake, m_IntakeStatus, m_shooter,m_turret).withTimeout(3),
      new CMD_BackIntakeRetract(m_intake, m_IntakeStatus)
      ,new CMD_DrivetrainReset(m_drivetrain)
      )

    );
  }


}

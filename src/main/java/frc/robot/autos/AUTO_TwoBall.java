// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.AUTO_Trajectory;
import frc.robot.commands.CMD_FrontIntakeToggle;
import frc.robot.commands.CMD_ShooterOff;
import frc.robot.commands.CMD_Shooting;
import frc.robot.commands.CMD_TurretMode;
import frc.robot.subsystems.FSM_IntakeStatus;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_Turret;
import frc.robot.subsystems.SwerveDrivetrain;

public class AUTO_TwoBall extends SequentialCommandGroup {
  /** 
   * 3 Ball Autonomous for the right side(looking from driver station)
  */
  SUB_Turret m_Turret;
  SUB_Intake m_Intake;
  FSM_IntakeStatus m_IntakeStatus;
  SUB_Shooter m_Shooter;
  AUTO_Trajectory m_Trajectory;

  public AUTO_TwoBall(SUB_Turret p_Turret, SUB_Intake p_Intake, 
                FSM_IntakeStatus p_IntakeStatus, SUB_Shooter p_Shooter, 
                SwerveDrivetrain p_Drivetrain, AUTO_Trajectory p_AutoTrajectory) {
    
    m_Turret = p_Turret;
    m_Intake = p_Intake;
    m_IntakeStatus = p_IntakeStatus;
    m_Shooter = p_Shooter;
    m_Trajectory = p_AutoTrajectory;
    addCommands(
      new CMD_TurretMode(m_Turret), //switch on auto mode for turret
      new CMD_Shooting(m_Intake, m_IntakeStatus, m_Shooter),//first ball
      new CMD_FrontIntakeToggle(m_Intake, m_IntakeStatus), //Deploying front intake
      m_Trajectory.driveTrajectory(m_Trajectory.FirstBallTrajectory),
      new CMD_Shooting(m_Intake, m_IntakeStatus, m_Shooter)//second ball

    );
  }

}

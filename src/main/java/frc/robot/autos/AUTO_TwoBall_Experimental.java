 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.AUTO_Trajectory;
import frc.robot.commands.CMD_FrontIntakeForward;
import frc.robot.commands.CMD_FrontIntakeOff;
import frc.robot.commands.CMD_FrontIntakeToggle;
import frc.robot.commands.CMD_FrontSolonoidRetract;
import frc.robot.commands.CMD_ShooterOff;
import frc.robot.commands.CMD_Shooting;
import frc.robot.commands.CMD_StopShooting;
import frc.robot.commands.CMD_TurretMode;
import frc.robot.subsystems.FSM_IntakeStatus;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_Turret;
import frc.robot.subsystems.SwerveDrivetrain;

public class AUTO_TwoBall_Experimental extends SequentialCommandGroup {
  /** 
   * 2 Ball Autonomous for all positions.
   * Unfortunately experimental doesn't work with current intake code  
   * It also looks like shooting one at a time is better for this tournament
   * -Daniel 3/12/2022 
  */
  SUB_Turret m_Turret;
  SUB_Intake m_Intake;
  FSM_IntakeStatus m_IntakeStatus;
  SUB_Shooter m_Shooter;
  AUTO_Trajectory m_Trajectory;

  public AUTO_TwoBall_Experimental(SUB_Turret p_Turret, SUB_Intake p_Intake, 
                FSM_IntakeStatus p_IntakeStatus, SUB_Shooter p_Shooter, 
                SwerveDrivetrain p_Drivetrain, AUTO_Trajectory p_AutoTrajectory) {
    
    m_Turret = p_Turret;
    m_Intake = p_Intake;
    m_IntakeStatus = p_IntakeStatus;
    m_Shooter = p_Shooter;
    m_Trajectory = p_AutoTrajectory;
    addCommands(
      // new CMD_TurretMode(m_Turret), //switch on auto mode for turret
      // new CMD_FrontIntakeToggle(m_Intake, m_IntakeStatus), //Deploying front intake
      // new CMD_FrontIntakeForward(m_Intake, m_IntakeStatus),
      m_Trajectory.driveTrajectory(m_Trajectory.FirstBallTrajectoryPathWeaver)
      // new WaitCommand(0.5),
      // new CMD_Shooting(m_Intake, m_IntakeStatus, m_Shooter),//first ball
      // new WaitCommand(3),
      // new CMD_StopShooting(m_Intake, m_IntakeStatus, m_Shooter)

    );
  }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.AUTO_Trajectory;
import frc.robot.commands.CMD_DeployBackIntake;
import frc.robot.commands.CMD_DeployFrontIntake;
import frc.robot.commands.CMD_ForceFeedToShooter;
import frc.robot.commands.CMD_RetractBackIntake;
import frc.robot.commands.CMD_RetractFrontIntake;
import frc.robot.commands.CMD_ShooterOff;
import frc.robot.commands.CMD_Shooting;
import frc.robot.commands.CMD_TurretMode;
import frc.robot.subsystems.FSM_IntakeStatus;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_Turret;
import frc.robot.subsystems.SwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AUTO_Shoot extends SequentialCommandGroup {
  /** 
   * 3 Ball Autonomous
  */
  SUB_Turret m_Turret;
  SUB_Intake m_Intake;
  FSM_IntakeStatus m_IntakeStatus;
  SUB_Shooter m_Shooter;
  AUTO_Trajectory m_Trajectory;

  public AUTO_Shoot(SUB_Turret p_Turret, SUB_Intake p_Intake, 
                FSM_IntakeStatus p_IntakeStatus, SUB_Shooter p_Shooter, 
                SwerveDrivetrain p_Drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_Turret = p_Turret;
    m_Intake = p_Intake;
    m_IntakeStatus = p_IntakeStatus;
    m_Shooter = p_Shooter;
    m_Trajectory = new AUTO_Trajectory(p_Drivetrain);
    addCommands(
      new CMD_TurretMode(m_Turret),
      new CMD_Shooting(m_Turret, m_Intake, m_IntakeStatus, m_Shooter),//first ball
      new ParallelRaceGroup(//clear hopper
        new CMD_ForceFeedToShooter(m_Intake),
        new WaitCommand(2)
      ),
      new CMD_DeployFrontIntake(m_Intake, m_IntakeStatus),
      m_Trajectory.driveTrajectory(m_Trajectory.FirstBallTrajectory),
      new WaitCommand(0.1), //intaking ball
      new ParallelRaceGroup(
        new CMD_Shooting(m_Turret, m_Intake, m_IntakeStatus, m_Shooter), //second ball
        new WaitCommand(1.5)
      ),
      new ParallelRaceGroup(//clear hopper
        new CMD_ForceFeedToShooter(m_Intake),
        new WaitCommand(2)
      ),
      new CMD_DeployBackIntake(m_Intake, m_IntakeStatus), //probably don't have to do
      m_Trajectory.driveTrajectory(m_Trajectory.SecondBallTrajectory),
      new ParallelRaceGroup(
        new CMD_Shooting(m_Turret, m_Intake, m_IntakeStatus, m_Shooter), //third ball
        new WaitCommand(1.5)
      ),
      new ParallelRaceGroup(//clear hopper
        new CMD_ForceFeedToShooter(m_Intake),
        new WaitCommand(1)
      ),
      new CMD_RetractFrontIntake(m_Intake),
      new CMD_RetractBackIntake(m_Intake),
      new CMD_ShooterOff(m_Shooter)
    );
  }

}

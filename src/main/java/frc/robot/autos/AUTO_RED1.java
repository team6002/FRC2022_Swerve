// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.FSM_IntakeStatus;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Navx;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.FSM_IntakeStatus.IntakeState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AUTO_RED1 extends SequentialCommandGroup {
  /** Creates a new AUTO_ForwardWaitBack. */
  public final SwerveDrivetrain m_drivetrain;
  public final AUTO_Trajectory m_trajectory;
  final SUB_Intake m_intake;
  final SUB_Shooter m_shooter;
  final FSM_IntakeStatus m_intakeStatus;
  public AUTO_RED1(SwerveDrivetrain drivetrain, AUTO_Trajectory trajectory, SUB_Intake p_Intake, SUB_Shooter p_shooter, FSM_IntakeStatus p_intakeStatus) {
    m_drivetrain = drivetrain;
    m_trajectory = trajectory;
    m_intake = p_Intake;
    m_shooter = p_shooter;
    m_intakeStatus = p_intakeStatus;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    // new CMD_FrontIntakeForward(m_intake),
    // trajectory.driveTrajectory(trajectory.FirstBallTrajectory),
    // new CMD_Shooting(m_intake, m_shooter, m_intakeStatus),
    // new CMD_FrontIntakeForward(m_intake),
    // trajectory.driveTrajectory(trajectory.SecondBallTrajectory),
    // new CMD_Shooting(m_intake, m_shooter, m_intakeStatus),
    // new CMD_FrontIntakeForward(m_intake)
    // trajectory.driveTrajectory(trajectory.ThirdBallTrajectory),
    // new WaitCommand(2),
    // trajectory.driveTrajectory(trajectory.ReturnTrajectory),
    // new CMD_Shooting(m_intake, m_shooter, m_intakeStatus)
    );
    
  }
  
}
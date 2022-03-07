// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveDrivetrain;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AUTO_Example extends SequentialCommandGroup {
  /** Creates a new AUTO_ForwardWaitBack. */
  public final SwerveDrivetrain m_drivetrain;
  public final AUTO_Trajectory m_trajectory;
  public AUTO_Example(SwerveDrivetrain drivetrain, AUTO_Trajectory trajectory) {
    m_drivetrain = drivetrain;
    m_trajectory = trajectory;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    // trajectory.driveTrajectory(trajectory.exampleTrajectory)
    );
    
  }
  
}
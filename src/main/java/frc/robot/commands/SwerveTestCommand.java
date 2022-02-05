package frc.robot.commands;

// import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;


public class SwerveTestCommand extends CommandBase {

  private final SwerveDrivetrain drivetrain;
  public boolean fieldMode = false;
  
  public SwerveTestCommand(SwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);


  }
 
  @Override
  public void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.

    final var xSpeed = 1;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    
    final var ySpeed = 0;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    
    final var rot = 0;

    boolean fieldRelative = drivetrain.getFieldMode();

    drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  public double jStickBand(double value) {
    if (Math.abs(value) < 0) return 0;

    return value;
  }

}
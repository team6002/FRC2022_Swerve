package frc.robot.commands;

// import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;


public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrivetrain drivetrain;
  private final XboxController controller;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  public boolean fieldMode = false;
  double deadzone = 0.2;	//variable for amount of deadzone
  double y = 0;           //variable for forward/backward movement
  double x = 0;           //variable for side to side movement
  double turn = 0;        //variable for turning movement
  
  public SwerveDriveCommand(SwerveDrivetrain drivetrain, XboxController controller) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.controller = controller;
  }

  @Override
  public void initialize() {
  }
 
  @Override
  public void execute() {
      y = controller.getLeftY();

      x = controller.getLeftX();
    
      turn = controller.getRightX();
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.

    final var xSpeed =
      -xspeedLimiter.calculate(jStickBand(y))
        * SwerveDrivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    
    final var ySpeed =
      -yspeedLimiter.calculate(jStickBand(x))
        * SwerveDrivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    
    final var rot =
      -rotLimiter.calculate(jStickBand(turn))
        * SwerveDrivetrain.kMaxAngularSpeed;

    if(controller.getLeftTriggerAxis() >= 0.7){
      //left evasive
      drivetrain.LeftEvasive();
    }else if(controller.getRightTriggerAxis() >= 0.7){
      //right evasive
      drivetrain.RightEvasive();
    }else{
      drivetrain.NonEvasive();
      //non evasive
    }

    if(controller.getAButtonPressed()){
      drivetrain.fieldModeChange();
    }
    boolean fieldRelative = drivetrain.getFieldMode();

    SmartDashboard.putNumber("xspeed", xSpeed);
    SmartDashboard.putNumber("yspeed", ySpeed);
    SmartDashboard.putNumber("rotspeed", rot);
    // SmartDashboard.putNumber("yaxis", controller.getLeftY());
    // SmartDashboard.putNumber("x-axis", controller.getRightX());

    drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  public double jStickBand(double value) {
    if (Math.abs(value) < deadzone) return 0;

    return value;
  }

}
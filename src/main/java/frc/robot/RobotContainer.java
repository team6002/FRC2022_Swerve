// EMILY IS YOUR MESSAGE STILL THERE

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.FSM_IntakeStatus.IntakeState;
import frc.robot.autos.AUTO_Trajectory;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_driverController;
  private final XboxController m_operatorController;
  public final SwerveDrivetrain m_drivetrain = new SwerveDrivetrain();
  // public final SUB_Navx m_NavxGyro = new SUB_Navx();
  public final FSM_IntakeStatus m_intakeStatus = new FSM_IntakeStatus();
  public final SUB_Intake m_intake = new SUB_Intake(m_intakeStatus);
  public final SUB_Climber m_climber = new SUB_Climber();
  // public final SUB_Turret m_turret = new SUB_Turret();
  // public final AUTO_Trajectory m_autotrajectory = new AUTO_Trajectory(m_drivetrain);
  public final SUB_Shooter m_shooter = new SUB_Shooter();
  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driverController = new XboxController(0);
    m_operatorController = new XboxController(1);
    SmartDashboard.putData("SyncAngles", new CMD_SyncSwerveEncoders(m_drivetrain));
    SmartDashboard.putData("ResetAngles", new CMD_ResetSwerve(m_drivetrain));
    // SmartDashboard.putData("Secondary Climber Home", new CMD_ClimberSecondarySetHome(m_climber,true));
    // SmartDashboard.putData("Primary Climber Home", new CMD_ClimberPrimarySetHome(m_climber,true));
    // SmartDashboard.putData("Reset NavX", new CMD_ResetNavX(m_NavxGyro));
    
    // Configure the button bindings
    configureButtonBindings();
    m_drivetrain.setDefaultCommand(new SwerveDriveCommand(m_drivetrain, m_driverController));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
      .whenPressed(new CMD_ResetNavX(m_drivetrain)
    );

    // new POVButton(m_operatorController, 90)
    //   .whenPressed(new CMD_SideTurret(m_turret));
    
    // new POVButton(m_operatorController, 0)
    //   .whenPressed(new CMD_FrontTurret(m_turret));
    
    // new POVButton(m_operatorController, 180)
    //   .whenPressed(new CMD_BackTurret(m_turret));
    
    // new POVButton(m_operatorController, 270)
    //   .whenPressed(new CMD_ResetTurret(m_turret));

    // new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
    //   .whenPressed(new CMD_TurretMode(m_turret));

   
   /** 
    * IMPROVEMENTS:
    * Alter command structure so that commands tell robot what statee to be in, and 
    *   subsystem handles actual movement to state
    * Decide flow of subsystems (when shooter turns off, intake when?)
    * Idle state for Shooter (lower rpm)
    * Global Robot Status (network tables, or constants-like file)
    * Singleton Subsystems? (getInstance())
  */ 

  // shooting
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
    .whenPressed(new CMD_Shooting(m_intake, m_intakeStatus, m_shooter));

    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
    .whenPressed(new ParallelCommandGroup(
      new CMD_ShooterOff(m_shooter),
      // new CMD_HopperOff(m_intake),
      new CMD_IndexerOff(m_intake),
      // new CMD_BackIntakeOff(m_intake),
      // new CMD_FrontIntakeOff(m_intake)
      new CMD_SetIntakeStatus(m_intakeStatus, IntakeState.INTAKE)
      )
    );

    new JoystickButton(m_operatorController, XboxController.Button.kBack.value)
      .whenPressed(new CMD_ClimberPrimarySetHome(m_climber, true)
    );

    new JoystickButton(m_operatorController, XboxController.Button.kStart.value)
      .whenPressed(new CMD_ClimberSecondarySetHome(m_climber, true)
    );

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
      .whenPressed(new CMD_FrontIntakeToggle(m_intake, m_intakeStatus)
    );

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
      .whenPressed(new CMD_BackIntakeToggle(m_intake, m_intakeStatus)
    );

    // new POVButton(m_driverController, 180)
    //   .whenPressed(new CMD_InitalizeClimbMode(m_climber, m_turret)
    // );

    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
      .whenPressed(new CMD_ClimbPartial(m_climber)
    );

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
      .whenPressed(new CMD_ClimbFull(m_climber)
    );

  }
  

  public XboxController getDriveController(){
    return m_driverController;
  }

  public XboxController getOperatorController() {
    return m_operatorController;
  }

  public double getRightX2() {
    return m_operatorController.getRightX();
  }

  // public void turretOpenLoop(double d) {
  //   if(Math.abs(d) < .06) {
  //     m_turret.setOpenLoop(0);
  //   }
  //   else {
  //     m_turret.setOpenLoop(d);
  //   }
  // }
}

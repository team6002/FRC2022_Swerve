// EMILY IS YOUR MESSAGE STILL THERE

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import javax.swing.SwingWorker.StateValue;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.FSM_ClimberMode.ClimberState;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Util.DigitalSensor;
import frc.robot.Util.TRG_ClimberMode;
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
  private final XboxController m_controller = new XboxController(0);
  private final XboxController m_secondController = new XboxController(1);
  public final SwerveDrivetrain m_drivetrain = new SwerveDrivetrain();
  final FSM_IntakeStatus m_intakeStatus = new FSM_IntakeStatus();
  final SUB_Intake m_intake = new SUB_Intake(m_intakeStatus);
  final FSM_ClimberMode m_climberMode = new FSM_ClimberMode();
  public final SUB_Navx m_NavxGyro = new SUB_Navx();
  final SUB_Climber m_climber = new SUB_Climber();
  public final SUB_Turret m_turret = new SUB_Turret();
  public final AUTO_Trajectory trajectory = new AUTO_Trajectory(m_drivetrain);
  public final SUB_Shooter m_shooter = new SUB_Shooter();
  Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putData("SyncAngles", new CMD_SyncSwerveEncoders(m_drivetrain));
    SmartDashboard.putData("ResetAngles", new CMD_ResetSwerve(m_drivetrain));
    // Configure the button bindings
    configureButtonBindings();
    // m_FRCGyro.calibrateFRCGyro();
    // m_shooter.setDefaultCommand(new CMD_ShooterOn(m_shooter,m_secondController));
    m_climber.setDefaultCommand(new CMD_ClimberMove(m_climber, m_secondController));
    m_drivetrain.setDefaultCommand(new SwerveDriveCommand(m_drivetrain,m_controller));
    // m_drivetrain.setDefaultCommand(new SwerveTestCommand(m_drivetrain,m_controller));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_secondController, XboxController.Button.kA.value)
    .and(new TRG_ClimberMode(m_climberMode, ClimberState.SHOOTING))
    .whenActive(new CMD_Shooting(m_intake, m_shooter, m_intakeStatus))
    .and(new TRG_ClimberMode(m_climberMode, ClimberState.CLIMBING))
    ;

    new JoystickButton(m_secondController, XboxController.Button.kB.value)
      .toggleWhenPressed(new CMD_SetClimberMode(m_climberMode, ClimberState.CLIMBING)
      ).whenPressed(new CMD_SetClimberMode(m_climberMode, ClimberState.SHOOTING)
      );

    new JoystickButton(m_secondController, XboxController.Button.kY.value)
    .whenPressed(new CMD_ClimberMainToggle(m_climber)
    );
    
    new JoystickButton(m_secondController, XboxController.Button.kX.value)
    .whenPressed(new CMD_ClimberSecondToggle(m_climber)
    );
  

    new JoystickButton(m_controller, XboxController.Button.kA.value)
    .whenPressed(new SequentialCommandGroup(
      new CMD_FrontIntakeForward(m_intake),
      new CMD_FrontSolonoidExtend(m_intake)
    ));
    new JoystickButton(m_controller, XboxController.Button.kB.value)
    .whenPressed(new SequentialCommandGroup(
      new CMD_FrontSolonoidRetract(m_intake)
      
    ));
    new JoystickButton(m_controller, XboxController.Button.kX.value)
    .whenPressed(new CMD_BackSolonoidRetract(m_intake)
    );
    new JoystickButton(m_controller, XboxController.Button.kY.value)
    .whenPressed(new SequentialCommandGroup(
      new CMD_BackSolonoidExtend(m_intake),
      new CMD_BackIntakeForward(m_intake)

    ));
  }

  public XboxController getDriveController(){
    return m_controller;
  }





 
  
}

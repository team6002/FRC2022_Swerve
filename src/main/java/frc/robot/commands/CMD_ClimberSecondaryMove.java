package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Climber;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer;

public class CMD_ClimberSecondaryMove extends CommandBase {
    SUB_Climber m_Climber;
    private XboxController secondController;
    public CMD_ClimberSecondaryMove(SUB_Climber p_Climber, XboxController secondController)
    {
        m_Climber = p_Climber;
        addRequirements(p_Climber);
        this.secondController = secondController;
    }
  
    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
    double x = 0;
    double deadzone = 0.3;
       if(secondController.getRightY() > deadzone || secondController.getRightY() < -deadzone) {
           x = secondController.getRightY();
         }
      m_Climber.moveSecondaryClimber(x);
      // m_Climber.moveClimber(0.3);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
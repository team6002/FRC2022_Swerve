package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Climber;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer;

public class CMD_ClimberMove extends CommandBase {
    SUB_Climber m_Climber;
    private XboxController secondController;
    public CMD_ClimberMove(SUB_Climber p_Climber, XboxController secondController)
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
     double Ly = 0;
     double Ry = 0;
     double deadzone = 0.5;
        if(secondController.getLeftY() > deadzone || secondController.getLeftY() < -deadzone) {
            Ly = secondController.getLeftY();
          }
      m_Climber.movePrimaryClimber(Ly);

       if(secondController.getRightY() > deadzone || secondController.getRightY() < -deadzone) {
           Ry = secondController.getRightY();
         }
      m_Climber.moveSecondaryClimber(Ry);
      // m_Climber.moveClimber(0.3);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
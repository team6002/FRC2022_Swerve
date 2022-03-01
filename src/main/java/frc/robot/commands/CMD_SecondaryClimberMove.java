package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Climber;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotContainer;

public class CMD_ClimberThing extends CommandBase {
    SUB_Climber m_Climber;
    private XboxController secondController;
    public CMD_ClimberThing(SUB_Climber p_Climber, XboxController secondController)
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
      m_Climber.moveClimber(secondController.getLeftY());
      // m_Climber.moveClimber(0.3);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
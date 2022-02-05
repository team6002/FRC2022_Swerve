package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SUB_Shooter;
import edu.wpi.first.wpilibj.XboxController;

public class CMD_ShooterOn extends CommandBase {
    SUB_Shooter m_Shooter;
    private XboxController secondController;
    public CMD_ShooterOn(SUB_Shooter p_Shooter, XboxController secondController)
    {
        m_Shooter = p_Shooter;
        addRequirements(p_Shooter);
        this.secondController = secondController;
    }
  
    @Override
    public void initialize() {
       
    }

    @Override
    public void execute(){
      m_Shooter.setVoltage(secondController.getLeftY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

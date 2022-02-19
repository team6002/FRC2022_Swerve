package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_ShooterOn extends CommandBase {
    SUB_Shooter m_Shooter;
    public CMD_ShooterOn(SUB_Shooter p_Shooter)
    {
        m_Shooter = p_Shooter;
    }
  
  
    @Override
    public void initialize() {
       m_Shooter.readyShooter();
    }

    @Override
    public void execute(){
      
    }

    @Override
    public boolean isFinished() {
        if (m_Shooter.getVelocity() >= ShooterConstants.kShootingVelocity){
            return true;
        }else return false;
    }
}

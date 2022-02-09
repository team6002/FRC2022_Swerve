package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;

public class SUB_Turret {
    private CANSparkMax m_Turret = new CANSparkMax(ShooterConstants.kTurretMotor, MotorType.kBrushless);
    
    public SUB_Turret()
    {
        m_Turret.restoreFactoryDefaults();
        m_Turret.setIdleMode(IdleMode.kBrake);
    }

    

}

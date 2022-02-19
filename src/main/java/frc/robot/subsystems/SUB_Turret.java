// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;



/** Add your docs here. */
public class SUB_Turret extends SubsystemBase {
    private CANSparkMax m_TurretMotor = new CANSparkMax(TurretConstants.kTurretMotor, MotorType.kBrushless);


    public SUB_Turret() {

    }
    
    public void setTurretForward(){
        m_TurretMotor.setVoltage(5);
    }

    public void setTurretReverse(){
        m_TurretMotor.setVoltage(-8);
    }
    public void setTurretOff(){
        m_TurretMotor.setVoltage(0);
     
    }
 



    @Override
    public void periodic() {
    }
}
        
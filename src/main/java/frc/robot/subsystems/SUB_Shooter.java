package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class SUB_Shooter extends SubsystemBase{
    private CANSparkMax m_ShooterMaster = new CANSparkMax(ShooterConstants.kShooterMaster, MotorType.kBrushless);
    private CANSparkMax m_ShooterSlave = new CANSparkMax(ShooterConstants.kShooterSlave, MotorType.kBrushless);

    private RelativeEncoder m_ShooterMasterEncoder = m_ShooterMaster.getEncoder();
    // private RelativeEncoder m_ShooterSlaveEncoder = m_ShooterSlave.getEncoder();

    private SparkMaxPIDController m_Controller = m_ShooterMaster.getPIDController();

    public SUB_Shooter()
    {
    
        m_ShooterMaster.restoreFactoryDefaults();
        m_ShooterSlave.restoreFactoryDefaults();

        m_ShooterMaster.setIdleMode(IdleMode.kCoast);
        m_ShooterSlave.setIdleMode(IdleMode.kCoast);

        m_ShooterSlave.follow(m_ShooterMaster, true);
        m_ShooterMaster.setInverted(true);

        m_Controller.setFF(ShooterConstants.kShooterFF);
        m_Controller.setP(ShooterConstants.kShooterP);
        m_Controller.setI(ShooterConstants.kShooterI);
        m_Controller.setD(ShooterConstants.kShooterD);

        m_Controller.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);
        m_Controller.setSmartMotionMaxVelocity(ShooterConstants.kShootingVelocity, 0);
        m_Controller.setSmartMotionMaxAccel(ShooterConstants.kShootingAccel, 0);

    }

    public void setVoltage(double value)
    {
        m_ShooterMaster.set(value);
    }

    public void shooterOff()
    {
        m_ShooterMaster.set(0);
    }

    public void readyShooter()
    {
        m_Controller.setReference(ShooterConstants.kShootingVelocity, CANSparkMax.ControlType.kVelocity);
    }

    public double getVelocity()
    {
        return m_ShooterMasterEncoder.getVelocity();
    }
    @Override
    public void periodic() {
        getVelocity();
        SmartDashboard.putNumber("ShooterVelocity",getVelocity());
        // SmartDashboard.putNumber("Shootervelocity",getVelocity());
        // double kFF = SmartDashboard.getNumber("kShooterFF", ShooterConstants.kShooterFF);
        // double kP = SmartDashboard.getNumber("kShooterP", ShooterConstants.kShooterP);
        // double kD = SmartDashboard.getNumber("kShooterD", ShooterConstants.kShooterD);
        // if (kFF != shooterFF){
        //     shooterFF = kFF;
        //     m_Controller.setFF(shooterFF);
        // }
        // if (kP != shooterP){
        //     shooterP = kP;
        //     m_Controller.setP(shooterP);
        // }
        // if (kD != shooterD){
        //     shooterD = kD;
        //     m_Controller.setD(shooterD);
        // }
        // This method will be called once per scheduler run
      
      }

}

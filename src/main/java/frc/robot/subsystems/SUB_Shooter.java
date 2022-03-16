package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class SUB_Shooter extends SubsystemBase{
    //motors
    private CANSparkMax m_ShooterMaster = new CANSparkMax(ShooterConstants.kShooterMaster, MotorType.kBrushless);
    private CANSparkMax m_ShooterSlave = new CANSparkMax(ShooterConstants.kShooterSlave, MotorType.kBrushless);

    //encoders
    private RelativeEncoder m_ShooterMasterEncoder = m_ShooterMaster.getEncoder();
    // private RelativeEncoder m_ShooterSlaveEncoder = m_ShooterSlave.getEncoder();

    //PID controller
    private SparkMaxPIDController m_Controller = m_ShooterMaster.getPIDController();

    private double m_ShooterSetpoint = ShooterConstants.kShootingVelocity;

    private boolean wantShooter = false;
    private boolean twoBall = true;
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
        SmartDashboard.putNumber("Desired Shooter Setpoint", ShooterConstants.kShootingVelocity);
    }

    // //turns on shooter
    // public void shooterOn()
    // {
    //     m_ShooterMaster.set(ShooterConstants.kShooterSpeed);
    // }

    //turns off shooter
    public void shooterOff()
    {
        wantShooter = false;
        // m_ShooterMaster.set(0);
    }

    //gets the shooter up to speed
    public void readyShooter()
    {
        wantShooter = true;
        // m_Controller.setReference(ShooterConstants.kShootingVelocity, CANSparkMax.ControlType.kVelocity);
    }

    //checks if the shooter is ready to shoot
    public boolean isReady(double setpoint, double epsilon)
    {
        return (getVelocity() - epsilon <= setpoint) && (getVelocity() + epsilon >= setpoint);
    }

    //gets velocity of shooter
    public double getVelocity()
    {
        return m_ShooterMasterEncoder.getVelocity();
    }
    public double getShooterSetpoint(){
        return m_ShooterSetpoint;
    }

    //AUTONOMOUS USE ONLY
    // public void twoBallSecondShot(){
    //     m_ShooterSetpoint = 
    // }
    // public void setShooterSetpoint(double setpoint){
        // m_ShooterSetpoint = setpoint;
    // }

    public void setTwoBall(boolean auto){
        twoBall = auto;
    }
    
    @Override
    public void periodic() {
        //must press tab to set in smartdashboard
        m_ShooterSetpoint = SmartDashboard.getNumber("Desired Shooter Setpoint", 
                                                        ShooterConstants.kShootingVelocity);
        if(wantShooter){
            if(twoBall){// Hack to set different setpoint in auton for tourny, fix -Daniel 3/12/22
                m_Controller.setReference(4750, ControlType.kVelocity);
            } else {
                m_Controller.setReference(m_ShooterSetpoint, ControlType.kVelocity);
            }
        }else{
            m_Controller.setReference(0, ControlType.kDutyCycle);
        }
        SmartDashboard.putBoolean("Shooting", wantShooter);
        SmartDashboard.putNumber("ShooterVelocity", getVelocity());
    }
}

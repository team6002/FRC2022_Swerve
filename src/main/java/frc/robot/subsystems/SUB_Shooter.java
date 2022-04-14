package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.lib.util.linearInterpolator;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SUB_Shooter extends SubsystemBase{
    //motors
    private CANSparkMax m_ShooterMaster = new CANSparkMax(ShooterConstants.kShooterMaster, MotorType.kBrushless);
    private CANSparkMax m_ShooterSlave = new CANSparkMax(ShooterConstants.kShooterSlave, MotorType.kBrushless);

    // solenoids
    private Solenoid m_HoodSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ShooterConstants.kShooterHoodSolonoid);
    //encoders
    private RelativeEncoder m_ShooterMasterEncoder = m_ShooterMaster.getEncoder();
    // private RelativeEncoder m_ShooterSlaveEncoder = m_ShooterSlave.getEncoder();

    //PID controller
    private SparkMaxPIDController m_Controller = m_ShooterMaster.getPIDController();

    private double m_ShooterSetpoint = ShooterConstants.kShootingVelocity;
    
    private linearInterpolator m_ShooterInterpolator;
    private boolean wantShooter = false;
    private int m_shooterMode = 0;
    private int m_previousShooterMode = 0;
    private double m_targetDistance;
    private double m_shooterOffset;
    private boolean m_firstBall = false;
    private double m_waitTime = 0;
    //Network Table
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");


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
        m_ShooterInterpolator = new linearInterpolator(ShooterConstants.kShooterArray);
    }

    public void setFirstBall(boolean p_wantedAmount){
        m_firstBall = p_wantedAmount;
    }
    // set the mode to different ones 0 is auto, 1 is short, 2 is low mode, 4 is Reverse.
    public void setShooterMode(int p_wantedMode){
        m_shooterMode = p_wantedMode;
    }
    public int getShooterMode(){
        return m_shooterMode;
    }
    public int getPreviousShooterMode(){
        return m_previousShooterMode;
    }
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

    public void setShooterSetpoint(double p_value){
        m_ShooterSetpoint = p_value;
    }
    public double getDistance() {
        // double y = 0.0;
        try {
            m_targetDistance = table.getEntry("ty").getDouble(0.0);
        }
        catch(Exception e) {
            
        }

        return m_targetDistance;
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
    public void setShooterOffset(double value){
        m_shooterOffset += value;
    }
    
    public double getWaitTime(){
        return m_waitTime;
    }
    public void extendHood(){
        m_HoodSolenoid.set(false);
    }
    public void retractHood(){
        m_HoodSolenoid.set(true);
    }
    public void reverseShooter(){
        setShooterMode(4);
    }
    @Override
    public void periodic() {
        m_targetDistance = getDistance();                                                
        if(wantShooter){
            if (m_shooterMode == 0){
            m_ShooterSetpoint = m_ShooterInterpolator.getInterpolatedValue(m_targetDistance) + m_shooterOffset;
            m_Controller.setReference(m_ShooterSetpoint, ControlType.kVelocity);
            m_waitTime = 0;
            extendHood();
            }else if (m_shooterMode == 1){
                m_waitTime = 0.5;
                retractHood();
                if (m_firstBall){
                    m_ShooterSetpoint = ShooterConstants.kCloseShootingVelocityFirstShot;
                    m_Controller.setReference(m_ShooterSetpoint, ControlType.kVelocity);
                }else{
                    m_ShooterSetpoint = ShooterConstants.kCloseShootingVelocitySecondShot;
                    m_Controller.setReference(m_ShooterSetpoint, ControlType.kVelocity);
                }
            }else if (m_shooterMode == 2){
                extendHood();
                m_ShooterSetpoint = ShooterConstants.kLowShootingVelocity;
                m_Controller.setReference(m_ShooterSetpoint, ControlType.kVelocity);
                m_waitTime = 0;
            }
            else if (m_shooterMode == 4){
                m_Controller.setReference(ShooterConstants.kReverseShooterVelocity, ControlType.kVelocity);
            }

        }else{
            m_Controller.setReference(0, ControlType.kDutyCycle);
        }
        SmartDashboard.putNumber("targetDistance", m_targetDistance);
        // SmartDashboard.putNumber("ShooterOffset", m_shooterOffset);
        SmartDashboard.putBoolean("Shooting", wantShooter);
        SmartDashboard.putNumber("ShooterVelocity", getVelocity());
        SmartDashboard.putNumber("Interpolated value", m_ShooterSetpoint);
        SmartDashboard.putNumber("Shooter Mode", m_shooterMode);
        m_previousShooterMode = m_shooterMode;
    }
}

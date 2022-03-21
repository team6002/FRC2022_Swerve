package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class SUB_Turret extends SubsystemBase{
    //motors, encoders, PID controller, limit switches
    private CANSparkMax m_Turret = new CANSparkMax(TurretConstants.kTurretMotor, MotorType.kBrushless);
    private final RelativeEncoder m_Encoder = m_Turret.getEncoder();
    private SparkMaxPIDController m_Controller = m_Turret.getPIDController();
    private SparkMaxLimitSwitch m_ForwardLimitSwitch = m_Turret.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    private SparkMaxLimitSwitch m_ReverseLimitSwitch = m_Turret.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    private double center = 27; //the limelight x val goes from -27 to 27
    public int huntDirection = 1;

    //Network Table
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    //joystick
    XboxController joystick;

    public SUB_Turret(XboxController p_joystick){
        joystick = p_joystick;

        m_Turret.setIdleMode(IdleMode.kBrake);
        m_Turret.setInverted(true);

        m_Controller.setFF(TurretConstants.kTurretFF);
        m_Controller.setP(TurretConstants.kTurretP);
        m_Controller.setI(TurretConstants.kTurretI);
        m_Controller.setD(TurretConstants.kTurretD);

        m_Controller.setOutputRange(TurretConstants.kMinTurretOutput, TurretConstants.kMaxTurretOutput);

        m_ForwardLimitSwitch.enableLimitSwitch(true);
        m_ReverseLimitSwitch.enableLimitSwitch(true);

        //testing encoders (figure out converstion factor on real robo)
        m_Encoder.setPositionConversionFactor(12);
        m_Encoder.setPosition(-90);//0
    }

    public void turretReset() {
        turretMode = -1;
    }

    //2020 robot positions
    private double targetPosition = -90;//0;

    //turretMode: 1=manual, 0 = auto (default), -1=calibration, 3=joystick mode
    private int turretMode = 1;
    private final double RESET_TURRET = 140; // value of encoder when left limit switch is triggered

    public void setFrontPosition() {
        targetPosition = 90;
    }

    public void setBackPosition() {
        targetPosition = -90;
    }

    public void setSidePosition() {
        targetPosition = 0;
    }

    public void increasePosition() {
        targetPosition++;
    }

    public void setTurretMode(int wantedMode) {
        turretMode = wantedMode;
    }

    public int getTurretMode() {
        return turretMode;
    }
    
    //sets the which way the turret should turn to find a target
    public void setHuntDirection(int dir) {
        if(dir == 1) {
            huntDirection = 1;
        }
        else {
            huntDirection = -1;
        }
    }

    //Reads from the network table (x and y val is how far the camera is from the target)
    public double readtX() { 
        double x = 0.0;
        try {
            x = table.getEntry("tx").getDouble(0.0);
        }
        catch(Exception e) {
            
        }
        
        return x;
    }

    public double readtY() {
        double y = 0.0;
        try {
            y = table.getEntry("ty").getDouble(0.0);
        }
        catch(Exception e) {
            
        }
        
        return y;
    }

    //if the limelight sees any targets
    public double readtV() {
        double v = 0.0;
        try {
            v = table.getEntry("tv").getDouble(0.0);
        }
        catch(Exception e) {
            
        }
        
        return v;
    }

    //calculate how far the target is from center
    //right is negative, left is positive
    public double diffFromCenter() {
        return readtX() - (center + OFFSET);
    }

    //set offset
    private int OFFSET = 0;
    public void setOffset(){
        OFFSET = 0;
    }

    public void setEncoderPosition(int pos){
        m_Encoder.setPosition(pos);        
    }

    public void setOpenLoop(double d) {
        m_Turret.set(d);
    }

    @Override
    public void periodic() {
        double sentOutput = 0;
        double diffFromCenter = 0;

        if(turretMode == 0) { //auto mode = 0
            targetPosition = 0;
            if(readtX() == 0) { //no target found
                //move turret towards hunt direction, hunt direction -1 = counterclockwise +1 = clockwise
                //A NOTE FOR THE FUTURE:
                // slow down near the limit switches, maybe lowered voltage or pid.
                if(m_ForwardLimitSwitch.isPressed() == true) {
                    setHuntDirection(-1);
                }
                else if(m_ReverseLimitSwitch.isPressed() == true) {
                    setHuntDirection(1);
                }
    
                diffFromCenter = -999;
                sentOutput = huntDirection * TurretConstants.kTurretHuntVoltage;
            }
            else {
                diffFromCenter = -diffFromCenter();
                sentOutput = readtX() / center * TurretConstants.kTurretVoltage;

                if(Math.abs(diffFromCenter) < 2)
                {
                    sentOutput = 0;
                }

                // if(m_ForwardLimitSwitch.isPressed() == true && sentOutput < 0)
                // {
                //     sentOutput = 0;
                // }
                // else if(m_ReverseLimitSwitch.isPressed() == true && sentOutput > 0)
                // {
                //     sentOutput = 0;
                // }
            }
        }
        else if (turretMode == 1) { //manual position mode
            sentOutput = (targetPosition - m_Encoder.getPosition()) / 180 * TurretConstants.kTurretMannualVoltage;

            double minvolef = 0.6;
            if(sentOutput > TurretConstants.kTurretMannualVoltage) {
                sentOutput = TurretConstants.kTurretMannualVoltage;
            }
            if(Math.abs(sentOutput) < minvolef) {
                if (sentOutput < 0) {
                    sentOutput = -minvolef;
                }
                else {
                    sentOutput = minvolef;
                }
            }

            if (Math.abs(targetPosition - m_Encoder.getPosition()) < 1) {
                sentOutput = 0;
            }

            // if(m_ForwardLimitSwitch.isPressed() == true && sentOutput < 0)
            // {
            //     sentOutput = 0;
            // }
            // else if(m_ReverseLimitSwitch.isPressed() == true && sentOutput > 0)
            // {
            //     sentOutput = 0;
            // }
        }
        else if(turretMode == -1) { //reset turret position
            if(m_ForwardLimitSwitch.isPressed() == true) {
                sentOutput = 0;
                m_Encoder.setPosition(RESET_TURRET);
                targetPosition = 0;
                turretMode = 1;
            }
            else {
                sentOutput = TurretConstants.kTurretResetVoltage;
            }
        } 
        else if (turretMode == 2) { //openloop mode 
            
        }
        else if(turretMode == 3) { //joystick mode
            double xVal = joystick.getLeftX();
            if(Math.abs(xVal) < 0.1) {
                xVal = 0;
            }
            sentOutput = xVal * TurretConstants.kTurretJoystickVoltage;
        }

        if(turretMode != 2) {
            m_Turret.setVoltage(sentOutput);
        }

        // //Shuffleboard Output
        SmartDashboard.putNumber("Turret X", readtX());
        SmartDashboard.putNumber("Turret Y", readtY());
        SmartDashboard.putNumber("Targets?", readtV());
        // SmartDashboard.putNumber("Voltage", sentOutput);
        // SmartDashboard.putNumber("Difference", diffFromCenter);
        // SmartDashboard.putBoolean("Target?", onTarget);
        // SmartDashboard.putNumber("Hunting Direction", huntDirection);
        SmartDashboard.putBoolean("Forward Limit Switch", m_ForwardLimitSwitch.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Switch", m_ReverseLimitSwitch.isPressed());
        // SmartDashboard.putBoolean("Ball color???", redBall);
        SmartDashboard.putNumber("Turret Encoder", m_Encoder.getPosition());
        // SmartDashboard.putNumber("Target Encoder", targetPosition);
        SmartDashboard.putNumber("Turret Mode", turretMode);

        // SmartDashboard.putBoolean("Forward Enabled", m_ForwardLimitSwitch.isLimitSwitchEnabled());
        // SmartDashboard.putBoolean("Reverse Enabled", m_ReverseLimitSwitch.isLimitSwitchEnabled());
    }

    //soft limit forward = 43
    //soft limit reverse = -16
}
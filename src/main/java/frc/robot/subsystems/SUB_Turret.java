package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
    public int huntDirection = 1; //goes positive initially
    private double targetPosition = -Math.PI/2; //-90;

    //turretMode: auto = 0; mannual = 1; reset = -1; joystick = 2
    private int turretMode = 0;
    private final double RESET_TURRET = (140/180) * Math.PI; //140; //value of encoder when left limit switch is triggered

    //Network Table
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    //joystick
    XboxController joystick;

    //ontarget (global bool)

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

        m_Encoder.setPositionConversionFactor((2*Math.PI) / 30); //converts to radian
        m_Encoder.setPosition(-Math.PI/2); //starting position of turret during auto (rad)
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
        return readtX() - center;
    }

    public void turretReset() {
        turretMode = -1;
    }

    public void setFrontPosition() {
        targetPosition = Math.PI/2; //90;
    }

    public void setBackPosition() {
        targetPosition = -Math.PI/2; //-90;
    }

    public void setSidePosition() {
        targetPosition = 0;
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

    public double validateAngle(double p_angle) {
        System.out.println("validating angle");
        System.out.println(p_angle);
        if(Math.abs(p_angle) < RESET_TURRET) {
            return p_angle;
        }
        
        return Math.copySign(RESET_TURRET, p_angle);
    }

    double validAngle = 0;
    @Override
    public void periodic() {
        double sentOutput = 0;

        if(turretMode == 0) { //auto mode = 0
            targetPosition = 0;
            if(readtV() == 0) { //no target found
                //move turret towards hunt direction, hunt direction -1 = counterclockwise +1 = clockwise
                // //A NOTE FOR THE FUTURE:
                // // slow down near the limit switches, maybe lowered voltage or pid.
                // if(m_ForwardLimitSwitch.isPressed() == true) {
                //     setHuntDirection(-1);
                // }
                // else if(m_ReverseLimitSwitch.isPressed() == true) {
                //     setHuntDirection(1);
                // }
                // sentOutput = 0;
                // diffFromCenter = -999;
                // sentOutput = huntDirection * TurretConstants.kTurretHuntVoltage;
            }
            else {
                //algo
                /*
                get degree offset from limelight, add to current pos, check if position is valid
                set smart controller to vaild pos 
                */
                double newAngle = Math.toRadians(readtX()) + m_Encoder.getPosition(); //gets wanted position

                validAngle = validateAngle(newAngle);
                System.out.println("validated angle");
                System.out.println(validAngle);
                m_Controller.setReference(validAngle, ControlType.kPosition);
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

            m_Turret.setVoltage(sentOutput);
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

            m_Turret.setVoltage(sentOutput);
        } 
        // else if(turretMode == 2) { //joystick mode
        //     double xVal = joystick.getLeftX();
        //     if(Math.abs(xVal) < 0.1) {
        //         xVal = 0;
        //     }
        //     sentOutput = xVal * TurretConstants.kTurretJoystickVoltage;
        // }

        //Shuffleboard Output
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
        SmartDashboard.putNumber("Valid Position", validAngle);
        // SmartDashboard.putNumber("Target Encoder", targetPosition);
        SmartDashboard.putNumber("Turret Mode", turretMode);

        // SmartDashboard.putBoolean("Forward Enabled", m_ForwardLimitSwitch.isLimitSwitchEnabled());
        // SmartDashboard.putBoolean("Reverse Enabled", m_ReverseLimitSwitch.isLimitSwitchEnabled());
    }

    //soft limit forward = 43
    //soft limit reverse = -16
}
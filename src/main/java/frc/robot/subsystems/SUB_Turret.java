package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    private double center = 80; //center of the camera (160x120)
    private boolean onTarget = false;
    public int huntDirection = 1;

    //Network Table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Turret");

    //alliance color picker variable
    String RED = "RED";
    String BLUE = "BLUE";
    String bColor = "YOSHI";
    SendableChooser<String> m_color = new SendableChooser<>();

    public SUB_Turret(){
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

        //add options to sendable chooser
        m_color.addOption("RED", RED);
        m_color.addOption("BLUE", BLUE);
        m_color.setDefaultOption("RED", RED);
        SmartDashboard.putData("chooser", m_color);
    }

    public void turretReset() {
        turretMode = -1;
    }

    //2020 robot positions
    private double targetPosition = -90;//0;

    //turretMode: 1=manual, 0 = auto (default), -1=calibration
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
        if(wantedMode == 1) {
            turretMode = wantedMode;
        } else {
            turretMode = 0;
        }
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

    //Reads from the network table
    public double readcX() { 
        double x = -1;
        try {
            x = table.getEntry("cX").getDouble(-1);
        }
        catch(Exception e) {
            
        }
        
        return x;
    }

    public double readcY() {
        return table.getEntry("cY").getDouble(-1);
    }

    //calculate how far the target is from center
    //right is negative, left is positive
    public double diffFromCenter() {
        return readcX() - (center + OFFSET);
    }

    //shooter ball color alliance thangy

    private boolean redBall = true;
    public void setBallColor(boolean col){
        redBall = col; 
    }

    //checks if the alliance color is red or blue
    public boolean checkChooser(){
        if (bColor == "RED"){
            return true;
        } else return false;
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
        double targetX = readcX();
        double sentOutput = 0;
        double diffFromCenter = 0;

        if(redBall != checkChooser()) {
            setOffset();
        } else {
            OFFSET = 0;
        }

        bColor = m_color.getSelected();

        //auto mode = 0
        if(turretMode == 0) {
            targetPosition = 0;
            if(targetX == -1) {
                //no target found
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
                sentOutput = diffFromCenter / center * TurretConstants.kTurretVoltage;

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
        else if (turretMode == 1) {
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
        else if(turretMode == -1) {
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

        if(turretMode != 2) {
            m_Turret.setVoltage(sentOutput);
        }

        // //Shuffleboard Output
        // SmartDashboard.putNumber("X", readcX());
        // SmartDashboard.putNumber("Y", readcY());
        // SmartDashboard.putNumber("Voltage", sentOutput);
        // SmartDashboard.putNumber("Difference", diffFromCenter);
        // SmartDashboard.putBoolean("Target?", onTarget);
        // SmartDashboard.putNumber("Hunting Direction", huntDirection);
        SmartDashboard.putBoolean("Forward Limit Switch", m_ForwardLimitSwitch.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Switch", m_ReverseLimitSwitch.isPressed());
        SmartDashboard.putBoolean("Ball color???", redBall);
        // SmartDashboard.putNumber("Turret Encoder", m_Encoder.getPosition());
        SmartDashboard.putNumber("Target Encoder", targetPosition);
        SmartDashboard.putNumber("Turret Mode", turretMode);

        // SmartDashboard.putBoolean("Forward Enabled", m_ForwardLimitSwitch.isLimitSwitchEnabled());
        // SmartDashboard.putBoolean("Reverse Enabled", m_ReverseLimitSwitch.isLimitSwitchEnabled());
    }

    //soft limit forward = 43
    //soft limit reverse = -16
}
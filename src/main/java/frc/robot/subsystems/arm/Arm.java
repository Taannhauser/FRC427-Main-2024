package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 

public class Arm extends SubsystemBase {
    
    private static Arm instance; 
    //  = new Arm(); 

    public static Arm getInstance() {
        return instance; 
    }
    
    // Define and initiate variables for arm
    private double m_targetPosition = Constants.ArmConstants.kTravelPosition;
    private double m_manualVelocity = 0;
    

    private DigitalInput m_limitSwitch = new DigitalInput(Constants.ArmConstants.kLimitSwitchId);

    private CANSparkMax m_armMotorRight = new CANSparkMax(Constants.ArmConstants.kArmMotorRightId, MotorType.kBrushless);
    private CANSparkMax m_armMotorLeft = new CANSparkMax(Constants.ArmConstants.kArmMotorLeftId, MotorType.kBrushless);

    private AbsoluteEncoder m_armEncoderRight = m_armMotorRight.getAbsoluteEncoder(Type.kDutyCycle);
    
    private PIDController m_armPIDController = new PIDController(Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD);
    
    // public ArmFeedforward m_armFeedforward = new ArmFeedforward(Constants.ArmConstants.kS, Constants.ArmConstants.kG, Constants.ArmConstants.kV, Constants.ArmConstants.kA);

    private ArmControlType m_ArmControlType = Arm.ArmControlType.PID;
    private double m_kG = Constants.ArmConstants.kGravityFF;
    private double m_kS = Constants.ArmConstants.kSpringFF;

    private Arm() {
        setupMotors();
    }

    // motor and encoder config
    public void setupMotors() {
        m_armMotorRight.setInverted(Constants.ArmConstants.kRightMotorInverted);
        m_armMotorLeft.setInverted(Constants.ArmConstants.kLeftMotorInverted);
        
        m_armMotorRight.setSmartCurrentLimit(Constants.ArmConstants.kMotorCurrentLimit);
        m_armMotorLeft.setSmartCurrentLimit(Constants.ArmConstants.kMotorCurrentLimit);

        // conversion factors
        m_armEncoderRight.setPositionConversionFactor(Constants.ArmConstants.kPositionConversionFactor);
        m_armEncoderRight.setVelocityConversionFactor(Constants.ArmConstants.kVelocityConversionFactor);
        
        // position error on which it is tolerable
        m_armPIDController.setTolerance(Constants.ArmConstants.kTolerance);
        
        // left arm motor would follow right arm  motor's voltage intake 
        m_armMotorLeft.follow(m_armMotorRight);
    }

    public void periodic() {
        doSendables();
        
        double impendingVelocity = 0; 

        // velocity controlled by PID and custom FF
        if (m_ArmControlType == ArmControlType.PID) {
           impendingVelocity =  m_armPIDController.calculate(m_armEncoderRight.getPosition(), m_targetPosition) 
                              + m_kG * Math.cos(Math.toRadians(m_armEncoderRight.getPosition()))
                              + m_kS;
        }
        
        // velocity controlled manually
        else if (m_ArmControlType == ArmControlType.MANUAL) {
            impendingVelocity = m_manualVelocity;
        }

        // if the arm is not reaching beyond the limit switch,
        // or if the arm is not reaching beyond 100 degs, 
        // arm is allowed to move 
        // forward defined as away from the limit switch.
        boolean passReverseSoftLimit = (m_limitSwitch.get() || getAngle() < Constants.ArmConstants.kReverseSoftLimit) && impendingVelocity < 0;
        boolean passForwardSoftLimit = getAngle() > Constants.ArmConstants.kForwardSoftLimit && impendingVelocity > 0;

        if (!passReverseSoftLimit && !passForwardSoftLimit) {
                m_armMotorRight.set(impendingVelocity); 
        }

        SmartDashboard.putNumber("Impending Velocity (m/s)", impendingVelocity);
        SmartDashboard.putBoolean("Pass Reverse Soft Limit", passReverseSoftLimit);
        SmartDashboard.putBoolean("Pass Forward Soft Limit", passForwardSoftLimit);

    }

    public void setKG(double kG) {
        this.m_kG = kG;
    }

    public void setKS(double kS) {
        this.m_kS = kS;
    }

    public double getError() {
        return m_armPIDController.getPositionError();
    }

    public void goToAngle(double angle) {
        this.m_targetPosition = angle;
    }

    public double getAngle() {
        return m_armEncoderRight.getPosition();
    }


    public void setSpeed(double speed) {
        this.m_manualVelocity = speed;
    }

    public boolean isAtAngle() {
        return m_armPIDController.atSetpoint();
        
    }

    public void setControlType(ArmControlType type) {
        this.m_ArmControlType = type;
    }

    public void setPID(double p, double i, double d) {
        this.m_armPIDController.setPID(p, i, d);
    }

    public ArmControlState getArmControlState() {
        if (m_targetPosition == Constants.ArmConstants.kGroundPosition) {
            return ArmControlState.GROUND;
        }
        else if (m_targetPosition == Constants.ArmConstants.kTravelPosition) {
            return ArmControlState.TRAVEL;
        }
        else if (m_targetPosition == Constants.ArmConstants.kAmpPosition) {
            return ArmControlState.AMP;
        }
        else if (m_targetPosition == Constants.ArmConstants.kSpeakerPosition) {
            return ArmControlState.SPEAKER;
        }
        else {
            return ArmControlState.CUSTOM;
        }
    }

    public enum ArmControlType {
        MANUAL, 
        PID
    }

    public enum ArmControlState {
        GROUND,
        TRAVEL,
        AMP,
        SPEAKER, 
        CUSTOM
    }

    // add logging for arm 
    // are units correct?
    public void doSendables() {
        SmartDashboard.putNumber("Arm Position (deg)", getAngle()); 
        SmartDashboard.putNumber("Arm Velocity (deg/sec)", m_armEncoderRight.getVelocity());
        SmartDashboard.putNumber("Arm Error (deg)", getError());
        SmartDashboard.putBoolean("Is Arm At Set Point", isAtAngle());
        SmartDashboard.putBoolean("Arm Limit Switch", m_limitSwitch.get());
        SmartDashboard.putString("Arm Control Type", m_ArmControlType.toString());
        SmartDashboard.putString("Arm Control State", getArmControlState().toString());
    }
}

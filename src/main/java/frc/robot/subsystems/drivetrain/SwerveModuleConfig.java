package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModuleConfig {
    private String moduleName; 
    private int kDrive; 
    private int kRotate; 
    private int kEncoder; 
    private double kAbsOffset;
    private boolean kRotateInverted;
    private boolean kDriveInverted; 
    private SensorDirectionValue direction; 

    public SwerveModuleConfig(String moduleName, int kDrive, int kRotate, int kEncoder, double kAbsOffset, boolean kRotateInverted, boolean kDriveInverted, SensorDirectionValue direction) {
        this.moduleName = moduleName; 
        this.kDrive = kDrive; 
        this.kRotate = kRotate; 
        this.kEncoder = kEncoder;
        this.kAbsOffset = kAbsOffset; 
        this.kRotateInverted = kRotateInverted;
        this.kDriveInverted = kDriveInverted; 
        this.direction = direction; 
    }

    public String getModuleName() {
        return this.moduleName; 
    }

    public int getDriveId() {
        return this.kDrive; 
    }

    public int getRotateId() {
        return this.kRotate; 
    }

    public int getEncoderId() {
        return this.kEncoder; 
    }

    public double getAbsoluteEncoderOffset() {
        return this.kAbsOffset; 
    }

    public boolean getRotateInverted() {
        return this.kRotateInverted;
    }

    public boolean getDriveInverted() {
        return this.kDriveInverted; 
    }

    public SensorDirectionValue getAbsoluteEncoderDirection() {
        return this.direction; 
    }
}

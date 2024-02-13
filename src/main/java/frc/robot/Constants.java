// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.SwerveModuleConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
 
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kManipulatorControllerPort = 1; 
  }

  public static class DrivetrainConstants {
    // Swerve IDs
    public static SwerveModuleConfig frontLeft = new SwerveModuleConfig("FrontLeft", 2, 1, 9, -0.265869140625, false, false, SensorDirectionValue.CounterClockwise_Positive); 
    public static SwerveModuleConfig frontRight = new SwerveModuleConfig("FrontRight", 4, 3, 10, 0.291748046875, false, true, SensorDirectionValue.CounterClockwise_Positive); 
    public static SwerveModuleConfig backLeft = new SwerveModuleConfig("BackLeft", 8, 7, 12, -0.35986328125, false, true, SensorDirectionValue.CounterClockwise_Positive); 
    public static SwerveModuleConfig backRight = new SwerveModuleConfig("BackRight", 5, 6, 11, -0.4599609375, false, true, SensorDirectionValue.CounterClockwise_Positive); 


    // Gearing & Conversions
    public static final double kGearRatio = 6.8; // 6.12; // driving gear ratio of each swerve module
    public static final double kWheelRadiusInches = 2; // radius of the wheels
    public static final double kMetersPerRot = Units.inchesToMeters(2 * Math.PI * kWheelRadiusInches / kGearRatio); // calculate the position conversion factor of the swerve drive encoder
    public static final double kMetersPerSecondPerRPM = kMetersPerRot / 60; // calculate the velocity conversion factor of the swerve drive encoder

    public static final double kRotateGearRatio = 1; // gear ratio of the turn encoder (will be 1 as long as we use CANCoders on the output shaft)
    public static final double kDegreesPerRot = 360 / kRotateGearRatio; // position conversion factor of the turn encoder (converts from rotations to degrees)
    public static final double kDegreesPerSecondPerRPM = kDegreesPerRot / 60; // velocity conversion factor of the turn encoder 

    // Drivebase
    public static final double kTrackWidthMeters = Units.inchesToMeters(23.25); // horizontal dist between wheels
    public static final double kWheelBaseMeters = Units.inchesToMeters(21.5); // vertical dist between wheels

    public static final double kDriveBaseRadius = Math.hypot(kTrackWidthMeters, kWheelBaseMeters) / 2; 

    // Kinematics
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      // wheel locations relative to the center of the robot
      new Translation2d(kTrackWidthMeters / 2, kWheelBaseMeters / 2), // front left
      new Translation2d(kTrackWidthMeters / 2, -kWheelBaseMeters / 2), // front right
      new Translation2d(-kTrackWidthMeters / 2, kWheelBaseMeters / 2), // back left
      new Translation2d(-kTrackWidthMeters / 2, -kWheelBaseMeters / 2) // back right
    ); 

    // copied speeds (https://github.com/SwerveDriveSpecialties/swerve-template-2021-unmaintained/blob/master/src/main/java/frc/robot/subsystems/DrivetrainSubsystem.java)
    public static final double kMaxAttainableModuleSpeedMetersPerSecond = 5880.0 / 60.0 / kGearRatio *
    2 * Units.inchesToMeters(kWheelRadiusInches) * Math.PI; // max attainable speed for each drive motor
    public static final double kMaxAttainableRotationRadPerSecond = kMaxAttainableModuleSpeedMetersPerSecond /
    Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0); // max rotation of robot
    
    // TODO: tune these
    public static double kMaxSpeedMetersPerSecond = 2.0; // max velocity (no turning) of robot; may tune to be a fraction of the attainable module speed
    public static double kMaxSlowSpeedMetersPerSecond = 1.0; 
    public static final double kMaxAccelerationMetersPerSecondSquared = kMaxSpeedMetersPerSecond / 0.2; // max acceleration of robot (accelerate to max speed in 1 second)
    public static double kMaxRotationRadPerSecond = 3.14; // max rotation speed of the robot
    public static final double kMaxSlowRotationRadPerSecond = Math.PI / 2; 
    public static final double kMaxRotationAccelerationRadPerSecondSquared = kMaxRotationRadPerSecond / 0.2; // max angular acceleration of robot

    // feedforward values (NO NEED to tune these)
    public static final double ksVolts = 0; 
    public static final double kvVoltSecondsPerMeter = 0; 
    public static final double kaVoltSecondsSquaredPerMeter = 0; 

    // drive speed PID values for a swerve module
    public static final double kModuleDrive_P = 0.01; 
    public static final double kModuleDrive_I = 0; 
    public static final double kModuleDrive_D = 0; 
    public static final double kModuleDrive_FF = 0.226; // 0.31;

    // found from sysid for one of the turn modules or tune by yourself
    // turn PID values for a swerve module
    public static final double kModuleTurn_P = 0.005; 
    public static final double kModuleTurn_I = 0; 
    public static final double kModuleTurn_D = 0.0002; 

    // turn in place PID for the whole robot
    public static final double kTurn_P = 0.054; 
    public static final double kTurn_I = 0; 
    public static final double kTurn_D = 0.001; 
    public static final double kTurn_FF = 0; 
    public static final double kTurnErrorThreshold = 2.0; 
    public static final double kTurnVelocityThreshold = 0; 

    // TODO: tune these but it should be fine
    // current limits for each motor
    public static final int kDriveCurrentLimit = 40; 
    public static final double kDriveRampRate = 0.25; 
     
    public static final int kTurnCurrentLimit = 20; 
    public static final double kTurnRampRate = 0.25;

    // max acceleration/deceleration of each motor (used for high CG robots)
    public static final double kForwardSlewRate = kMaxAccelerationMetersPerSecondSquared; 
    public static final double kStrafeSlewRate = kMaxAccelerationMetersPerSecondSquared; 
    public static final double kTurnSlewRate = kMaxRotationAccelerationRadPerSecondSquared; 
  }

  public static class Trajectory {
    // translational PID of robot for trajectory use
    public static final double kDrive_P = 5; // 2.25; 
    public static final double kDrive_I = 0; 
    public static final double kDrive_D = 0.00;

    // angular PID (same as turn pid)
    public static final double kOmega_P = 5; 
    public static final double kOmega_I = 0; 
    public static final double kOmega_D = 0.00; 

    // max velocity & acceleration robot can go n following a trajectory
    public static final double kMaxVelocityMetersPerSecond = 1; 
    public static final double kMaxAccelerationMetersPerSecondSquared = 1; 

    public static final double kMaxCentripetalAcceleration = 0.8; 
  }

  public class ArmConstants {
    public static final int kLimitSwitchId = 0;
    public static final int kArmMotorRightId = 0;
    public static final int kArmMotorLeftId = 0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    // calculate using reca.lc
    // CoM distance: 21.77 in
    // Arm mass: 20.755 lbs
    public static final double kS = 0; 
    public static final double kG = 0; // 0.79 V
    public static final double kV = 0; // 1.95 V*s/rad
    public static final double kA = 0; // 0.06 V*s^2/rad

    public static final boolean kRightMotorInverted = false;
    public static final boolean kLeftMotorInverted = false; 

    public static final int kMotorCurrentLimit = 40;
    
    public static final float kSoftLimitForward = 100;
    public static final float kSoftLimitReverse = 0;

    public static final double kPositionConversionFactor = 360;
    // velocity = position / 60
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0; 
    public static final double kTolerance = 0;
    
    public static final double kGroundPosition = 0;
    public static final double kTravelPosition = 0;
    public static final double kAmpPosition = 0;
    public static final double kSpeakerPosition = 0;

    public static final double kTravelSpeed = 0;
  }
}

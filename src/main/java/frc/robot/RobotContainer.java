// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.TeleOpCommand;
import frc.robot.subsystems.intakeit.Intakeit;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmControlState;
import frc.robot.subsystems.arm.commands.GoToAmp;
import frc.robot.subsystems.arm.commands.GoToGround;
import frc.robot.subsystems.arm.commands.GoToSpeaker;
import frc.robot.subsystems.arm.commands.GoToTravel;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.commands.TeleOpCommand;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.hang.commands.SetHangSpeed;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeFromGround;
import frc.robot.subsystems.intake.commands.OuttakeToAmp;
import frc.robot.subsystems.intake.commands.SetShooterSpeed;
import frc.robot.subsystems.intake.commands.SetSuckerIntakeSpeed;
import frc.robot.util.DriverController;
import frc.robot.util.DriverController.Mode;
import frc.robot.subsystems.leds.Led;
import frc.robot.subsystems.leds.patterns.LEDPattern;
import frc.robot.subsystems.vision.BackVision;
import frc.robot.subsystems.vision.FrontVision;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  private final AutoPicker autoPicker; 
  // private final SwerveTurnTunerCommand tunerCommand = new SwerveTurnTunerCommand(Constants.DrivetrainConstants.frontLeft);

  // drivetrain of the robot

  private final Intakeit intake = new Intakeit(); 
  
  // arm of the robot
  // private final Arm arm = new Arm();
  private final Drivetrain drivetrain = Drivetrain.getInstance();

  // intake of the bot
  // private final Intake intake = Intake.getInstance(); 

  // leds!
  private final Led led = Led.getInstance(); 
  private final AddressableLEDSim sim = new AddressableLEDSim(led.getLED()); 


  // limelight subsystem of robot
  private final BackVision backVision = BackVision.getInstance();
  private final FrontVision frontVision = FrontVision.getInstance(); 

  // hang mechanism of robot
  private final Hang hang = Hang.getInstance();
  
  // arm of the robot
  private final Arm arm = Arm.getInstance();
  
  private SendableChooser<LEDPattern> patterns = new SendableChooser<>();
  
  
 //  public Command tunegotoangle2 = new TuneGoToAngle(arm);

  // controller for the driver
  private final DriverController driverController =
      new DriverController(0);

  private final CommandXboxController manipulatorController = new CommandXboxController(1); 
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoPicker = new AutoPicker(drivetrain); 
    // Configure the trigger bindings
    configureBindings();

    // driverController.setChassisSpeedsSupplier(drivetrain::getChassisSpeeds); // comment in simulation
    // default command for drivetrain is to calculate speeds from controller and drive the robot
    drivetrain.setDefaultCommand(new TeleOpCommand(drivetrain, driverController));

    // manipulatorController.rightTrigger().whileTrue(intake.outtakeSpeaker(1, 1));
    manipulatorController.rightTrigger()
    .onFalse(
      intake.bottomSpeed(1)
        .andThen(new WaitCommand(0.5))
        .andThen(intake.stopAll())
    )
    .whileTrue(intake.topSpeed(1)); 
    manipulatorController.leftTrigger().onTrue(intake.intakeNote(0.5)).onFalse(intake.stopAll()); 
  }
    
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {

    // --- Driver ---

    driverController.a().onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

  // add headers 
    //
    driverController.rightTrigger()
      .onTrue(new InstantCommand(() -> driverController.setSlowMode(Mode.SLOW)))
      .onFalse(new InstantCommand(() -> driverController.setSlowMode(Mode.NORMAL))); 

    // // right stick y to manually move arm
    // new Trigger(() -> manipulatorController.getRightY() < -0.5) 
    //   .onTrue(new SetVelocity(arm, -Constants.ArmConstants.kTravelSpeed))
    //   .onFalse(new SetVelocity(arm, 0));
      
    // new Trigger(() -> manipulatorController.getRightY() > 0.5)
    //   .onTrue(new SetVelocity(arm, Constants.ArmConstants.kTravelSpeed))
    //   .onFalse(new SetVelocity(arm, 0));  

    // // buttons to move arm to go to setpoints
    // manipulatorController.a().onTrue(new GoToAngle(arm, Constants.ArmConstants.kGroundPosition));
    // manipulatorController.b().onTrue(new GoToAngle(arm, Constants.ArmConstants.kTravelPosition));
    // manipulatorController.x().onTrue(new GoToAngle(arm, Constants.ArmConstants.kSpeakerPosition));
    // manipulatorController.y().onTrue(new GoToAngle(arm, Constants.ArmConstants.kAmpPosition));
  }
  

  // send any data as needed to the dashboard
  public void doSendables() {
    SmartDashboard.putData("Autonomous", autoPicker.getChooser());
    SmartDashboard.putBoolean("gyro connected", drivetrain.gyro.isConnected()); 
    SmartDashboard.putData(patterns);
  }

  // gives the currently picked auto as the chosen auto for the match
  public Command getAutonomousCommand() {
      // return null; 
    return autoPicker.getAuto();
    // return tunerCommand;

  }
}

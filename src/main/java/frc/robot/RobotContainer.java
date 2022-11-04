
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.*;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

/* All variables, objects, and methods declared in lowerCamelCase */

public class RobotContainer {
  /* Controllers */
  private final Joystick translationController = new Joystick(0);
  private final Joystick rotationController = new Joystick(1);
  public final XboxController xboxController = new XboxController(2);
  private final Joystick xboxControllerJoystick = new Joystick(2);
  private final Joystick buttons = new Joystick(3);

  /* Drive Controls */
  private final int translationAxis = Constants.Controllers.TRANSLATION_AXIS;
  private final int strafeAxis = Constants.Controllers.STRAFE_AXIS;
  private final int rotationAxis = Constants.Controllers.ROTATION_AXIS;
  private final boolean openLoop = Constants.OPEN_LOOP;
  private  boolean fieldRelative = Constants.FIELD_RELATIVE;

  /* Driver Buttons */
  private final JoystickButton aButton = new JoystickButton(xboxController, Constants.Controllers.XBOXCONTROLLER_A_BUTTON);
  private final JoystickButton button1 = new JoystickButton(buttons, Constants.Controllers.BUTTON_ONE);



  private final JoystickButton rotationButton = new JoystickButton(rotationController, Constants.Controllers.ROTATION_BUTTON);
  private final JoystickButton translationButton = new JoystickButton(translationController, Constants.Controllers.TRANSLATION_BUTTON);
  
  private final JoystickAnalogButton rightTrigger = new JoystickAnalogButton(xboxController, Constants.Controllers.XBOX_CONTROLLER_RIGHT_TRIGGER, 0.3);  

  private final DPad dPadUp = new DPad(xboxControllerJoystick, 0);
  private final DPad dPadRight = new DPad(xboxControllerJoystick, 90);
  private final DPad dPadDown = new DPad(xboxControllerJoystick, 180);
  private final DPad dPadLeft = new DPad(xboxControllerJoystick, 270);


  /* Subsystems */
  public final DriveTrain s_DriveTrain = new DriveTrain();
  public final LimeLight s_LimeLight = new LimeLight();
  public final Shooter s_Shooter = new Shooter(s_LimeLight);
  public final Intake s_Intake = new Intake();
  public final Climber s_Climber = new Climber();
  
  /* Autos */
  private final DoNothing a_DoNothing = new DoNothing(s_Climber);
  private final LeaveTarmac a_LeaveTarmac = new LeaveTarmac(s_DriveTrain, s_Climber);
  private final OneBallAuto a_OneBallAuto = new OneBallAuto(s_DriveTrain, s_Climber, s_Shooter, s_Intake);
  private final OneBallAuto a_OneBallAutoOne = new OneBallAuto(s_DriveTrain, s_Climber, s_Shooter, s_Intake, 1);
  private final OneBallAuto a_OneBallAutoTwo = new OneBallAuto(s_DriveTrain, s_Climber, s_Shooter, s_Intake, 2);
  private final TwoBallAuto a_TwoBallAuto = new TwoBallAuto(s_DriveTrain, s_Climber, s_Shooter, s_Intake, s_LimeLight);
  private final TwoBallAuto a_TwoBallAutoOne = new TwoBallAuto(s_DriveTrain, s_Climber, s_Shooter, s_Intake, s_LimeLight, 1);
  private final TwoBallAuto a_TwoBallAutoTwo = new TwoBallAuto(s_DriveTrain, s_Climber, s_Shooter, s_Intake, s_LimeLight, 2);
  private final TwoBallAutoThree a_TwoBallAutoThree = new TwoBallAutoThree(s_DriveTrain, s_Climber, s_Shooter,  s_Intake, s_LimeLight);
  private final ThreeBallAuto2910 a_ThreeBallAuto2910 = new ThreeBallAuto2910(s_DriveTrain, s_Climber, s_Shooter,  s_Intake, s_LimeLight);
  private final FiveBallAuto2910 a_FiveBallAuto2910 = new FiveBallAuto2910(s_DriveTrain, s_Intake, s_Climber, s_Shooter, s_LimeLight);
  private final HideAndSeek a_HideAndSeek = new HideAndSeek(s_DriveTrain, s_Intake, s_LimeLight, s_Climber, s_Shooter);
  
  /* Commands */
  private final AutoTargetDetection c_AutoTargetDetection = new AutoTargetDetection(s_DriveTrain, s_LimeLight);
  private final FireShooter c_FireShooter = new FireShooter(s_Shooter, s_Intake);

  private final SendableChooser<Command> s_Command;

  private final ShuffleboardTab autoTab;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if(Constants.AUTO_LOG){
      DataLogManager.start();
      DataLogManager.logNetworkTables(false);
    }
    autoTab = Shuffleboard.getTab("Auto");
    s_Command = new SendableChooser<>();
    s_Command.setDefaultOption("Do Nothing", a_DoNothing);
    s_Command.addOption("Leave Tarmac", a_LeaveTarmac);
    s_Command.addOption("One Ball", a_OneBallAuto);
    s_Command.addOption("One Ball One", a_OneBallAutoOne);
    s_Command.addOption("One Ball Two", a_OneBallAutoTwo);
    s_Command.addOption("Two Ball", a_TwoBallAuto);
    s_Command.addOption("Two Ball One", a_TwoBallAutoOne);
    s_Command.addOption("Two Ball Two", a_TwoBallAutoTwo);
    s_Command.addOption("Two Ball Three", a_TwoBallAutoThree);
    s_Command.addOption("2910 Three Ball", a_ThreeBallAuto2910);
    s_Command.addOption("2910 Five Ball", a_FiveBallAuto2910);
    s_Command.addOption("HideAndSeek", a_HideAndSeek);
    
    autoTab.add("Command Chooser", s_Command).withPosition(0, 0);
    
    s_Shooter.hoodSetPosition(Constants.Shooter.NORMAL_RUN);
    s_DriveTrain.setDefaultCommand(new TeleopSwerve(s_DriveTrain, translationController, rotationController, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop, xboxController));
    s_Shooter.setDefaultCommand(new StartShooter(s_Shooter, xboxController));
    s_Intake.setDefaultCommand(new RunIntake(s_Intake, xboxController));
    s_Climber.setDefaultCommand(new ClimberState(s_Climber, xboxController));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    translationButton.whenPressed(new InstantCommand(() -> s_DriveTrain.zeroGyro()));
   // rotationButton.whenPressed(c_AutoTargetDetection)
   //              .whenReleased(new InstantCommand(() -> c_AutoTargetDetection.cancel()));
   
    dPadUp.whenPressed(new InstantCommand(() -> s_Shooter.setFender()));
    dPadDown.whenPressed(new InstantCommand(() -> s_Shooter.setTarmac()));
    dPadLeft.whenPressed(new InstantCommand(() -> s_Shooter.setLimeLight()));
    dPadRight.whenPressed(new InstantCommand(() -> s_Shooter.setLaunchPad()));

    button1.whenPressed(new InstantCommand(() -> s_DriveTrain.fieldRelativeTrue()))
                  .whenReleased(new InstantCommand(() -> s_DriveTrain.fieldRelativeFalse()));


    rightTrigger.whenPressed(c_FireShooter)
                .whenReleased(new InstantCommand(() -> c_FireShooter.cancel()));
    

    aButton.whenPressed(new InstantCommand(() -> s_LimeLight.turnLightOn()));
            
  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A command that is selected will run in autonomous
    return s_Command.getSelected();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class SystemsCheck extends CommandBase {
  private final Climber s_Climber;
  private final DriveTrain s_DriveTrain;
  private final Intake s_Intake;
  private final LimeLight s_LimeLight;
  private final Shooter s_Shooter;
  private final XboxController xboxController;
  private int currentState = 0;
  private int nextState = 0;
  private int i;
  private NetworkTableEntry intake, spit, drive, shooterLow, shooterLime, shooterFender, shooterTarmac, limelight, shooter, intakeSpit;
  /** Creates a new SystemsCheck. */
  public SystemsCheck(Climber s_Climber, DriveTrain s_DriveTrain, Intake s_Intake, LimeLight s_LimeLight, Shooter s_Shooter, XboxController xboxController, int i) {
    shooterLow = Shuffleboard.getTab("Systems Check" + i).add("ShooterLow", false).withPosition(0, 0).getEntry();
    shooterLime = Shuffleboard.getTab("Systems Check" + i).add("ShooterLime", false).withPosition(1, 0).getEntry();
    shooterFender = Shuffleboard.getTab("Systems Check" + i).add("ShooterFender", false).withPosition(2, 0).getEntry();
    shooterTarmac = Shuffleboard.getTab("Systems Check" + i).add("ShooterTarmac", false).withPosition(3, 0).getEntry();
    shooter = Shuffleboard.getTab("Systems Check" + i).add("Shooter", false).withPosition(4, 0).getEntry();
    intake = Shuffleboard.getTab("Systems Check" + i).add("Intake", false).withPosition(0, 1).getEntry();
    spit = Shuffleboard.getTab("Systems Check" + i).add("Spit", false).withPosition(1, 1).getEntry();
    limelight = Shuffleboard.getTab("Systems Check" + i).add("LimeLight", false).withPosition(2, 1).getEntry();
    drive = Shuffleboard.getTab("Systems Check" + i).add("Drive", false).withPosition(3, 1).getEntry();
    this.s_Climber = s_Climber;
    this.s_DriveTrain = s_DriveTrain;
    this.s_Intake = s_Intake;
    this.s_LimeLight = s_LimeLight;
    this.s_Shooter = s_Shooter;
    this.xboxController = xboxController;
    this.i = i;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Climber, s_DriveTrain, s_Intake, s_LimeLight, s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentState = 0;
    nextState = 0;
    shooterLow.setBoolean(false);
    shooterLime.setBoolean(false);
    shooterFender.setBoolean(false);
    shooterTarmac.setBoolean(false);
    shooter.setBoolean(false);
    intake.setBoolean(false);
    spit.setBoolean(false);
    intakeSpit.setBoolean(false);
    limelight.setBoolean(false);
    drive.setBoolean(false);
    Shuffleboard.selectTab("Systems Check" + i);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xboxController.getRawButton(8)){
      nextState++;
      System.out.println("BUTTON PRESSED");
    }
    if(currentState != nextState){
      currentState = nextState;
      switch(currentState){
        case 1: 
          new InstantCommand((() -> s_Shooter.setLaunchPad()));
          new InstantCommand((() -> s_Shooter.shooterRunToSetting(s_Shooter.shooterLaunch)));
          new InstantCommand((() -> s_Shooter.hoodMoveToSetting(s_Shooter.hoodLaunch)));
        case 2: 
          new InstantCommand((() -> s_Shooter.setLimeLight()));
          new InstantCommand((() -> s_Shooter.shooterRunToSetting(s_Shooter.shooterLime)));
          new InstantCommand((() -> s_Shooter.hoodMoveToSetting(s_Shooter.hoodLime)));
        case 3:
          new InstantCommand((() -> s_Shooter.setFender()));
          new InstantCommand((() -> s_Shooter.shooterRunToSetting(s_Shooter.shooterFender)));
          new InstantCommand((() -> s_Shooter.hoodMoveToSetting(s_Shooter.hoodFender)));
        case 4:
          new InstantCommand((() -> s_Shooter.setTarmac()));
          new InstantCommand((() -> s_Shooter.shooterRunToSetting(s_Shooter.shooterTarmac)));
          new InstantCommand((() -> s_Shooter.hoodMoveToSetting(s_Shooter.hoodTarmac)));
        case 5:
          new InstantCommand((() -> s_Shooter.stopShooter()));
          new InstantCommand((() -> s_Shooter.hoodSetPosition(Constants.Shooter.NORMAL_RUN)));
        case 6:
          new InstantCommand((() -> s_Climber.stageOneSetPosition(40)));
          new InstantCommand((() -> s_Intake.intakeForward()));
          new InstantCommand((() -> s_Intake.stagingForward()));
          new InstantCommand((() -> s_Intake.indexForward()));
        case 7:
          new InstantCommand((() -> s_Intake.spitAll()));
        case 8:
          new InstantCommand((() -> s_Intake.stopAll()));
          new InstantCommand((() -> s_Climber.stageOneSetPosition(1)));
        case 9:
          new InstantCommand((() -> s_LimeLight.turnLightOn()));
          new WaitCommand(0.5);
          new InstantCommand((() -> s_LimeLight.turnLightOff()));
          new WaitCommand(0.5);
          new InstantCommand((() -> s_LimeLight.turnLightOn()));
          new WaitCommand(0.5);
          new InstantCommand((() -> s_LimeLight.turnLightOff()));
        case 10:
          new InstantCommand((() -> s_DriveTrain.drive(new Translation2d(0, 3), 0, true, true)));
          new WaitCommand(0.5);
          new InstantCommand((() -> s_DriveTrain.drive(new Translation2d(0, -3), 0, true, true)));
          new WaitCommand(0.5);
          new InstantCommand((() -> s_DriveTrain.drive(new Translation2d(3, 0), 0, true, true)));
          new WaitCommand(0.5);
          new InstantCommand((() -> s_DriveTrain.drive(new Translation2d(-3, 0), 0, true, true)));
          new WaitCommand(0.5);
          new InstantCommand((() -> s_DriveTrain.drive(new Translation2d(3, 3), 0, true, true)));
          new WaitCommand(0.5);
          new InstantCommand((() -> s_DriveTrain.drive(new Translation2d(-3, -3), 0, true, true)));
          new WaitCommand(0.5);
          new InstantCommand((() -> s_DriveTrain.drive(new Translation2d(0, 0), 1, true, true)));
          new WaitCommand(0.5);
          new InstantCommand((() -> s_DriveTrain.drive(new Translation2d(0, 0), -1, true, true)));
        case 11:
          new InstantCommand((() -> s_DriveTrain.drive(new Translation2d(0, 0), 0, true, true)));
      }
    }
    switch(currentState){
      case 1:
        if(xboxController.getRawButton(1)){
          shooterLow.setBoolean(true);
        }
      case 2:
        if(xboxController.getRawButton(1)){
          shooterLime.setBoolean(true);
        }
      case 3:
        if(xboxController.getRawButton(1)){
          shooterFender.setBoolean(true);
        }
      case 4:
        if(xboxController.getRawButton(1)){
          shooterTarmac.setBoolean(true);
        }
      case 5:
        if(xboxController.getRawButton(1)){
          shooter.setBoolean(true);
        }
      case 6:
        if(xboxController.getRawButton(1)){
          intake.setBoolean(true);
        }
      case 7:
        if(xboxController.getRawButton(1)){
          spit.setBoolean(true);
        }
      case 8:
        if(xboxController.getRawButton(1)){
          intakeSpit.setBoolean(true);
        }
      case 9:
        if(xboxController.getRawButton(1)){
          limelight.setBoolean(true);
        }
      case 10:
        if(xboxController.getRawButton(1)){
          drive.setBoolean(true);
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.stopAll();
    s_Climber.stageOneSetPosition(1);
    s_Shooter.stopShooter();
    s_Shooter.hoodSetPosition(Constants.Shooter.NORMAL_RUN);
    s_DriveTrain.drive(new Translation2d(0, 0), 0, true, true);
    s_LimeLight.turnLightOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(currentState == 11){
      return true;
    }
    else{
      return false;
    }
  }
}

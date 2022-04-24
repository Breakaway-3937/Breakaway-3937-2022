// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
  private XboxController xboxController;
  private Timer timer;
  private int currentCondition = 5;
  private int nextCondition = -5;
  private Intake s_Intake;
  private boolean motorRun = false;
  
  /** Creates a new RunIntake. */
  public RunIntake(Intake s_Intake, XboxController xboxController) {
    this.xboxController = xboxController;
    this.s_Intake = s_Intake;
    timer = new Timer();
    timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentCondition = 5;
    nextCondition = -5;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xboxController.getRawButton(5)){
      nextCondition = -1;
    }
    else if(!xboxController.getRawButton(5)){
      nextCondition = 0;
    }
    if(xboxController.getLeftTriggerAxis() > 0.3){    
      motorRun = true;    
      /*if(s_Intake.ballDetect("intakeSensor") && !s_Intake.colorTest()){
        nextCondition = -1;
      }*/
      if(s_Intake.ballDetect("stagingSensor") && s_Intake.ballDetect("intakeSensor") && timer.get() > 1){
        nextCondition = 0;
      }
      else if(s_Intake.ballDetect("stagingSensor") && !s_Intake.ballDetect("intakeSensor") && timer.get() > 1){
        nextCondition = 1;
      }
      else if(!s_Intake.ballDetect("stagingSensor") && !s_Intake.ballDetect("intakeSensor") && timer.get() > 1){
        nextCondition = 2;
      }
      else if(!s_Intake.ballDetect("stagingSensor") && s_Intake.ballDetect("intakeSensor") && timer.get() > 1){
        nextCondition = 3;
      }
    }
    else if(xboxController.getLeftTriggerAxis() <= 0.3 && motorRun){
      s_Intake.stopAll();
      motorRun = false;
      currentCondition = 5;
      nextCondition = -5;
    }
    if(currentCondition != nextCondition){
      currentCondition = nextCondition;
      if(currentCondition == 0){
        s_Intake.stagingStop();
        s_Intake.intakeStop();
        s_Intake.indexStop();
      }
      else if(currentCondition == 1){
        s_Intake.stagingStop();
        s_Intake.indexForward();
        s_Intake.intakeForward();
      }
      else if(currentCondition == 2){
        s_Intake.stagingForward(0.25);
        s_Intake.indexForward();
        s_Intake.intakeForward();
      }
      else if(currentCondition == 3){
        s_Intake.stagingForward(0.25);
        s_Intake.indexForward();
        s_Intake.intakeForward();
      }
      else if(currentCondition == -1){
        timer.stop();
        timer.reset();
        timer.start();
        s_Intake.stagingStop();
        s_Intake.indexStop();
        s_Intake.intakeStop();
        s_Intake.intakeReverse();
        s_Intake.indexReverse();
        s_Intake.stagingReverse();
      }
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

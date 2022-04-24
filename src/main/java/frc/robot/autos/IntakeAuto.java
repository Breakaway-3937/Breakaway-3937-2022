// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class IntakeAuto extends CommandBase {
  /** Creates a new IntakeAuto. */
  private Intake s_Intake;
  private Climber s_Climber;
  private int nextCondition, currentCondition;
  private Timer timer;
  private boolean done, intake;
  
  public IntakeAuto(Intake s_Intake, Climber s_Climber, boolean intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Intake = s_Intake;
    this.s_Climber = s_Climber;
    addRequirements(s_Intake, s_Climber);
    timer = new Timer();
    timer.start();
    this.intake = intake;
  }
  public IntakeAuto(Intake s_Intake, Climber s_Climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Intake = s_Intake;
    this.s_Climber = s_Climber;
    addRequirements(s_Intake, s_Climber);
    timer = new Timer();
    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    s_Climber.stageOneSetPosition(39.5);
    s_Intake.indexForward();
    s_Intake.intakeForward();
    s_Intake.stagingForward();
    nextCondition = -7;
    currentCondition = -9;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
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

    if(currentCondition != nextCondition){
      currentCondition = nextCondition;
      if(currentCondition == 0){
        s_Intake.stopAll();
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
  public void end(boolean interrupted) {
    if(!intake){
      s_Climber.stageOneSetPosition(5);
      s_Intake.stopAll();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}

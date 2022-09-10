// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberState extends CommandBase {
  /** Creates a new ClimberState. */
  private XboxController xboxController;
  private Climber s_Climber;
  private int currentState;
  private int nextState;
  private boolean readyForNextState = true;
  private boolean extended = false;
  private boolean spit = false;
  private boolean case4Step1 = false;
  private boolean case4Step2 = false;
  private boolean case7Step1 = false;
  private boolean case7Step2 = false;
  private boolean case1Step1 = false;
  private boolean case1Step2 = false;
  
  public ClimberState(Climber s_Climber, XboxController xboxController) {
    /** Creates a new ClimberState. */
      this.s_Climber = s_Climber;
      this.xboxController = xboxController;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(s_Climber);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

      if(xboxController.getRawAxis(2) > 0.3 && !extended){
        s_Climber.stageOneSetPosition(41);
        extended = true;
        Shuffleboard.selectTab("Intake");
      }
      else if(xboxController.getRawAxis(2) <= 0.3 && extended){
        s_Climber.stageOneSetPosition(1);
        extended = false;
        Shuffleboard.selectTab("Drive");
      }
      if(xboxController.getRawButton(5) && !spit){
        s_Climber.stageOneSetPosition(40);
        spit = true;
        Shuffleboard.selectTab("Intake");
      }
      else if(!xboxController.getRawButton(5) && spit){
        s_Climber.stageOneSetPosition(5);
        spit = false;
        Shuffleboard.selectTab("Drive");
      }
      // Check buttons
      if(xboxController.getRawButton(4) && currentState != 1 && currentState < 1){
        s_Climber.setStageOnePIDSmartControl();
        nextState = 1;
        Shuffleboard.selectTab("Climber");
        if(Constants.AUTO_LOG){
          DataLogManager.logNetworkTables(true);
        }
      }
      if(xboxController.getBackButtonPressed() && currentState >= 1){
        nextState = 99;
      }
      if(xboxController.getStartButtonPressed() && currentState >= 1 && readyForNextState) {
        nextState++;
      }

      //Motor commands for new state
      if(nextState != currentState){
        switch(nextState){
          case 0: s_Climber.raiseArmSetPosition(0);
                  break;
          case 1: s_Climber.raiseArmSetPosition(18);
                  s_Climber.stageTwoSetPosition(0);
                  break;
          case 2:          
          case 5:
          case 9: s_Climber.disengage("stageOnePneu");
                  s_Climber.stageOneSetSmartPosition(80);
                  break;
          case 3: s_Climber.engage("stageTwoPneu");
                  s_Climber.stageOneSetSmartPosition(0);
                  break;
          //1st rung secured
          case 4:
          case 8: s_Climber.disengage("raiseArmPneu");
                  s_Climber.raiseArmSetPosition(8);
                  s_Climber.disengage("stageTwoPneu");
                  if(Constants.COMP_BOT){
                    s_Climber.stageTwoSetPosition(69);
                  }
                  else {
                  s_Climber.stageTwoSetPosition(72);
                  }
                  break;

          case 6:
          case 10:s_Climber.disengage("raiseArmPneu");
                  s_Climber.raiseArmSetPosition(18);
                  break;
          case 7: s_Climber.stageOneSetSmartPosition(0);
                  s_Climber.disengage("stageTwoPneu");
                  s_Climber.stageTwoSetPosition(0);
                  break;
          case 11:s_Climber.disengage("stageOnePneu");
                  s_Climber.stageOneSetSmartPosition(40);
                  break; 
        }
        currentState = nextState;
        readyForNextState = false;
      }      
        //Check conditions
      switch(nextState){
        case 0: if(s_Climber.currentPositionRaiseArm == 0){
          s_Climber.disengage("raiseArmPneu");
          s_Climber.stopArm();
          readyForNextState = true;
        }
        break;
        case 1: if(s_Climber.currentPositionRaiseArm > 17 && s_Climber.currentPositionRaiseArm < 19){
          case1Step1 = true;
          s_Climber.engage("raiseArmPneu");
          s_Climber.stopArm();
        }
        if(s_Climber.currentPositionStageTwo < 0.75){
          case1Step2 = true;
          s_Climber.engage("stageTwoPneu");
          s_Climber.stopStageTwo();
        }
        if(case1Step2 && case1Step1){
          readyForNextState = true;
        }
        break;
        case 2:  if(s_Climber.currentPositionStageOne > 79) {
          readyForNextState = true;  
        }
        break;
        case 3: if(s_Climber.currentPositionStageOne < 1.5 && s_Climber.currentPositionStageOne > 0){
          s_Climber.engage("stageOnePneu");
          s_Climber.stopStageOne();      
          readyForNextState = true;  
          }
        break;              
        case 4:
        case 8: if(s_Climber.currentPositionRaiseArm < 9){
          s_Climber.engage("raiseArmPneu");
          s_Climber.stopArm();
          case4Step1 = true;
        } 
        if((s_Climber.currentPositionStageTwo > 71 && !Constants.COMP_BOT) || 
          (s_Climber.currentPositionStageTwo > 68 && Constants.COMP_BOT) ){
          //s_Climber.engage("stageTwoPneu");
          //s_Climber.stopStageTwo();
          case4Step2 = true; 
        }
        if(case4Step1 && case4Step2){
          nextState++; 
        }
        break;  
        case 5:
        case 9: if(s_Climber.currentPositionStageOne > 79) {
          nextState++; 
        }
        break;
        case 6:
        case 10: if(s_Climber.currentPositionRaiseArm > 17.5){
          s_Climber.engage("raiseArmPneu");
          s_Climber.stopArm();
          readyForNextState = true;
        }
        break; 
        case 7: if(s_Climber.currentPositionStageOne < 1){
          s_Climber.engage("stageOnePneu");
          s_Climber.stopStageOne();
          case7Step1 = true;
          }
          if (s_Climber.currentPositionStageTwo < 1){
          s_Climber.engage("stageTwoPneu");
          s_Climber.stopStageTwo();
          case7Step2 = true;
          }
          if(case7Step1 && case7Step2){
          readyForNextState = true;  
          case4Step1 = false;
          case4Step2 = false;
        }
        break;
        case 11: if (s_Climber.currentPositionStageOne < 41){
                nextState = 99;
        }
        break;
        case 99: s_Climber.engage("stageOnePneu");
                 s_Climber.stopStageOne();
                 s_Climber.engage("stagePneuTwo");
                 s_Climber.stopStageTwo();
                 s_Climber.engage("raiseArmPneu");
                 s_Climber.stopArm();
                 nextState = 100;
                 DataLogManager.logNetworkTables(false);
        break;
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

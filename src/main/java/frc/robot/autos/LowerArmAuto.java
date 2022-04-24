// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class LowerArmAuto extends CommandBase {
  private Climber s_Climber;
  private boolean done, done1;
  /** Creates a new LowerArmAuto. */
  public LowerArmAuto(Climber s_Climber) {
    this.s_Climber = s_Climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    done1 = false;
    s_Climber.lowerArm();
    s_Climber.stageTwoSetPosition(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_Climber.currentPositionRaiseArm < 3){
      done = true;
      s_Climber.stopArm();
    }
    if(s_Climber.currentPositionStageTwo > 4.75 && s_Climber.currentPositionStageTwo < 5.25){
      done1 = true;
      s_Climber.stopStageTwo();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Climber.stopStageTwo();
    s_Climber.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(done && done1){
      return true;
    }
    else{
      return false;
    }
  }
}

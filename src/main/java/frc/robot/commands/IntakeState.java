// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class IntakeState extends CommandBase {
  private Climber s_Climber;
  /** Creates a new IntakeState. */
  public IntakeState(Climber s_Climber) {
    this.s_Climber = s_Climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Climber.stageOneSetPosition(40);
    Shuffleboard.selectTab("Intake");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((int) (s_Climber.currentPositionStageOne + 0.5) == 40){
      return true;
    }
    else{
      return false;
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class SpitAuto extends CommandBase {
  private final Climber s_Climber;
  private final Intake s_Intake;
  private final Timer timer;
  private boolean done;
  /** Creates a new SpitAuto. */
  public SpitAuto(Climber s_Climber, Intake s_Intake) {
    this.s_Climber = s_Climber;
    this.s_Intake = s_Intake;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake, s_Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    s_Climber.stageOneSetPosition(40);
    s_Intake.spitAll();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() > 2){
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.stopAll();
    s_Climber.stageOneSetPosition(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}

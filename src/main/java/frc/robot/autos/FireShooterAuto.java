// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FireShooterAuto extends CommandBase {
  private Shooter s_Shooter;
  private Intake s_Intake;
  private Timer timer;
  private boolean flag, flag1;
  private boolean done;


  /** Creates a new FireShooterAuto. */
  public FireShooterAuto(Shooter s_Shooter, Intake s_Intake) {
    this.s_Intake = s_Intake;
    this.s_Shooter = s_Shooter;
    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake, s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    flag = false;
    flag1 = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_Shooter.shooterAtVelocity() && !flag){
      timer.reset();
      timer.start();
      s_Intake.stagingForward(1);
      s_Intake.indexStop();
      s_Intake.intakeStop();
      flag = true;
    }
    else if(s_Shooter.shooterAtVelocity() && timer.get() > 0.25 && !flag1){
      s_Intake.stagingForward(0.8);
      s_Intake.indexForward(0.6);
      s_Intake.intakeForward(0.6);
      timer.reset();
      timer.start();
      flag1 = true;
    }
    if(timer.get() > 1){
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return done;

  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class StartShooterAuto extends CommandBase {
  /** Creates a new StartShooterAuto. */
  private Shooter s_Shooter;
  private boolean done;

  public StartShooterAuto(Shooter s_Shooter) {
    this.s_Shooter = s_Shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    if(s_Shooter.takeLimeLight){
      s_Shooter.shooterRunToSetting(s_Shooter.shooterLime);
      s_Shooter.hoodMoveToSetting(s_Shooter.hoodLime);
    }
    else if(s_Shooter.tarmac){
      s_Shooter.shooterRunToSetting(s_Shooter.shooterTarmac);
      s_Shooter.hoodMoveToSetting(s_Shooter.hoodTarmac);
    }
    else if(s_Shooter.fender){
      s_Shooter.shooterRunToSetting(s_Shooter.shooterFender);
      s_Shooter.hoodMoveToSetting(s_Shooter.hoodFender);
    }
    else if(s_Shooter.launchPad){
      s_Shooter.shooterRunToSetting(s_Shooter.shooterLaunch);
      s_Shooter.hoodMoveToSetting(s_Shooter.hoodLaunch);
    }
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(s_Shooter.shooterAtVelocity() && s_Shooter.hoodAtPosition()){
        done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}

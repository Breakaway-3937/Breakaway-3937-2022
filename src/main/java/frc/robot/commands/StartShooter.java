//// Copyright (c) FIRST and other WPILib contributors.
//// Open Source Software; you can modify and/or share it under the terms of
//// the WPILib BSD license file in the root directory of this project.
//
//package frc.robot.commands;
//
//import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants;
//import frc.robot.subsystems.LimeLight;
//import frc.robot.subsystems.Shooter;
//import frc.robot.subsystems.LimeLight.LightMode;
//
//public class StartShooter extends CommandBase {
//  /** Creates a new StartShooter. */
//  private boolean positionSet = false;
//  private Shooter s_Shooter;
//  private XboxController xboxController;
//  private boolean hoodTucked = false;
//
//  public StartShooter(Shooter s_Shooter, XboxController xboxController) {
//    this.s_Shooter = s_Shooter;
//    this.xboxController = xboxController;
//    // Use addRequirements() here to declare subsystem dependencies.
//    addRequirements(s_Shooter);
//  }
//
//  // Called when the command is initially scheduled.
//  @Override
//  public void initialize() {}
//
//
//  // Called every time the scheduler runs while the command is scheduled.
//  @Override
//  public void execute() {
//
//    if(xboxController.getRawButton(6)){
//      Shuffleboard.selectTab("Shooter");
//      if(s_Shooter.takeLimeLight){
//        if(s_Shooter.currentPosition != s_Shooter.hoodRequestedPosition){
//          s_Shooter.shooterRunToSetting(s_Shooter.shooterLime);
//          s_Shooter.hoodMoveToSetting(s_Shooter.hoodLime);
//        }
//      }
//      else if(s_Shooter.tarmac){
//        s_Shooter.shooterRunToSetting(s_Shooter.shooterTarmac);
//        s_Shooter.hoodMoveToSetting(s_Shooter.hoodTarmac);
//      }
//      else if(s_Shooter.fender){
//        s_Shooter.shooterRunToSetting(s_Shooter.shooterFender);
//        s_Shooter.hoodMoveToSetting(s_Shooter.hoodFender);
//      }
//      else if(s_Shooter.launchPad){
//        s_Shooter.shooterRunToSetting(s_Shooter.shooterLaunch);
//        s_Shooter.hoodMoveToSetting(s_Shooter.hoodLaunch);
//      }
//      positionSet = true;
//    }
//    else if(!xboxController.getRawButton(6) && positionSet){
//      s_Shooter.stopShooter();
//      s_Shooter.hoodSetPosition(Constants.Shooter.NORMAL_RUN);
//      positionSet = false;
//      if(LimeLight.currentState == LightMode.ON){
//        Shuffleboard.selectTab("SmartDashboard");
//      }
//      else{
//        Shuffleboard.selectTab("Drive");
//      }
//    }
//    if (xboxController.getRawButton(4) && !hoodTucked){
//      s_Shooter.hoodSetPosition(-5);
//      hoodTucked = true;
//    }
//  }
//
//  // Called once the command ends or is interrupted.
//  @Override
//  public void end(boolean interrupted) {}
//
//  // Returns true when the command should end.
//  @Override
//  public boolean isFinished() {
//    return false;
//  }
//}

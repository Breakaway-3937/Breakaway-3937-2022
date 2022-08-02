//// Copyright (c) FIRST and other WPILib contributors.
//// Open Source Software; you can modify and/or share it under the terms of
//// the WPILib BSD license file in the root directory of this project.
//
//package frc.robot.autos;
//
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants;
//import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.LimeLight;
//
//public class AutoTargetDetectionAuto extends CommandBase {
//  private DriveTrain s_DriveTrain;
//  private LimeLight s_LimeLight;
//  private double rAxis;
//  private double rotation;
//  private Translation2d translation;
//  private boolean done;
//  private Timer timer, timer1;
//  /** Creates a new AutoTargetDetectionAuto. */
//  public AutoTargetDetectionAuto(DriveTrain s_DriveTrain, LimeLight s_LimeLight) {
//    timer = new Timer();
//    timer1 = new Timer();
//    this.s_DriveTrain = s_DriveTrain;
//    this.s_LimeLight = s_LimeLight;
//    // Use addRequirements() here to declare subsystem dependencies.
//    addRequirements(s_DriveTrain, s_LimeLight);
//  }
//
//  // Called when the command is initially scheduled.
//  @Override
//  public void initialize() {
//    timer.reset();
//    timer.start();
//    done = false;
//    s_LimeLight.turnLightOn();
//    translation = new Translation2d(0, 0);
//  }
//
//  // Called every time the scheduler runs while the command is scheduled.
//  @Override
//  public void execute() {
//    if(!s_LimeLight.hasValidTarget()){
//      rAxis = 0.4;
//      rotation = rAxis * Constants.DriveTrain.MAX_ANGULAR_VELOCITY;
//    }
//    else if(s_LimeLight.hasValidTarget()){
//      if(s_LimeLight.getXAngle() > 30 ){
//        rAxis = 0.8;
//      }
//      else if(s_LimeLight.getXAngle() < -30){
//        rAxis = -0.8;
//      }
//      else{
//        if(s_LimeLight.getXAngle() < 0){
//          rAxis = (0.8/40.0)*(s_LimeLight.getXAngle() - (2/3*s_LimeLight.getYAngle()) + LimeLight.alignmentOffset);
//        }
//        else{
//          rAxis = (0.8/40.0)*(s_LimeLight.getXAngle() - (2/3*s_LimeLight.getYAngle()) + LimeLight.alignmentOffset);
//        }
//      }
//      rotation = rAxis * Constants.DriveTrain.MAX_ANGULAR_VELOCITY;
//    }
//    translation = new Translation2d(0, 0);
//    s_DriveTrain.drive(translation, rotation, true, true);
//    if(s_LimeLight.shotGood()){
//      timer1.reset();
//      timer.start();
//    }
//    if(timer.get() > 2){
//      done = true;
//    }
//    if(s_LimeLight.shotGood() && timer1.get() > 0.5){
//      done = true;
//    }
//  }
//
//  // Called once the command ends or is interrupted.
//  @Override
//  public void end(boolean interrupted) {
//    s_DriveTrain.drive(translation, 0, true, true);
//  }
//
//  // Returns true when the command should end.
//  @Override
//  public boolean isFinished() {
//    return done;
//  }
//}

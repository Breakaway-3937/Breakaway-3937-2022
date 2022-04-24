// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class AutoTargetDetection extends CommandBase {
  private DriveTrain s_DriveTrain;
  private LimeLight s_LimeLight;
  private double xAxis;
  private double yAxis;
  private double rAxis;
  private double rotation;
  private Translation2d translation;
  /** Creates a new AutoTargetDetection. */
  public AutoTargetDetection(DriveTrain s_DriveTrain, LimeLight s_LimeLight) {
    this.s_DriveTrain = s_DriveTrain;
    this.s_LimeLight = s_LimeLight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_DriveTrain, s_LimeLight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_LimeLight.turnLightOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    yAxis = Constants.Controllers.TRANSLATION_CONTROLLER.getRawAxis(Constants.Controllers.STRAFE_AXIS);
    xAxis = Constants.Controllers.TRANSLATION_CONTROLLER.getRawAxis(Constants.Controllers.TRANSLATION_AXIS);
    rAxis = Constants.Controllers.ROTATION_CONTROLLER.getRawAxis(Constants.Controllers.ROTATION_AXIS);
    
    /* Deadbands */
    yAxis = (Math.abs(yAxis) < Constants.Controllers.STICK_DEADBAND) ? 0 : yAxis;
    xAxis = (Math.abs(xAxis) < Constants.Controllers.STICK_DEADBAND) ? 0 : xAxis;
    if(!s_LimeLight.hasValidTarget()){
      rAxis = (Math.abs(rAxis) < Constants.Controllers.STICK_DEADBAND) ? 0 : rAxis;
      rotation = rAxis * Constants.DriveTrain.MAX_ANGULAR_VELOCITY;
    }
    else if(s_LimeLight.hasValidTarget()){
      if(s_LimeLight.getXAngle() > 30 ){
        rAxis = 0.8;
      }
      else if(s_LimeLight.getXAngle() < -30){
        rAxis = -0.8;
      }
      else{
        if(s_LimeLight.getXAngle() < 0){
          rAxis = (0.8/40.0)*(s_LimeLight.getXAngle() - (2/3*s_LimeLight.getYAngle()) + LimeLight.alignmentOffset);
        }
        else{
          rAxis = (0.8/40.0)*(s_LimeLight.getXAngle() - (2/3*s_LimeLight.getYAngle()) + LimeLight.alignmentOffset);
        }
      }
      rotation = rAxis * Constants.DriveTrain.MAX_ANGULAR_VELOCITY;
    }
    translation = new Translation2d(yAxis, xAxis).times(Constants.DriveTrain.MAX_SPEED);
    s_DriveTrain.drive(translation, rotation, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_LimeLight.turnLightOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

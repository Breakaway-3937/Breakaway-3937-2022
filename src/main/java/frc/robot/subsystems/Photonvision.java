// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.ArrayList;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The Limelight camera, running PhotonVision.
 * 
 * 
 */

public class Photonvision extends SubsystemBase {
  PhotonCamera limelight; //FIXME


  
  

  /** Creates a new LimelightS. */
  public Photonvision() {
    limelight = new PhotonCamera("gloworm"); //FIXME    
    limelight.setPipelineIndex(1);
    limelight.setDriverMode(false);
  }


  /**
   * Adds the latest PhotonVision result to the filters.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    PhotonTrackedTarget target = limelight.getLatestResult().hasTargets()
        ? limelight.getLatestResult().getBestTarget();

    

  }
}

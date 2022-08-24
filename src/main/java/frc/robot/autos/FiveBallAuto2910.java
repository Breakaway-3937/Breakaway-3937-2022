// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAuto2910 extends SequentialCommandGroup {
  /** Creates a new FiveBallAuto2910. */

  String trajectoryJSON;
  Trajectory trajectory = new Trajectory();
  public FiveBallAuto2910(DriveTrain s_DriveTrain, Intake s_Intake, Climber s_Climber, Shooter s_Shooter, LimeLight s_LimeLight) {
    TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.KMAX_SPEED_METERS_PER_SECOND,
                    Constants.AutoConstants.KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(Constants.DriveTrain.SWERVE_KINEMATICS);

        Trajectory pathTrajectory =
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(5.19, 2.01, Rotation2d.fromDegrees(42.14)), 
                new Pose2d(1.29, 1.40, Rotation2d.fromDegrees(-137.91))),
                config);
              
            var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.KP_THETA_CONTROLLER, 0, 0, Constants.AutoConstants.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory pathTrajectory1 =
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(1.29, 1.4, Rotation2d.fromDegrees(-137.91)), 
                new Pose2d(1.86, 1.85, Rotation2d.fromDegrees(24.44))),
                config);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                pathTrajectory,
                s_DriveTrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.AutoConstants.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.AutoConstants.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_DriveTrain::setModuleStates,
                s_DriveTrain);

        SwerveControllerCommand swerveControllerCommand1 =
            new SwerveControllerCommand(
                pathTrajectory1,
                s_DriveTrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.AutoConstants.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.AutoConstants.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_DriveTrain::setModuleStates,
                s_DriveTrain);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ThreeBallAuto2910(s_DriveTrain, s_Climber, s_Shooter, s_Intake, s_LimeLight),
      new ParallelCommandGroup(swerveControllerCommand,
      new IntakeAuto(s_Intake, s_Climber)),
      new IntakeAutoWithPause(s_Intake, s_Climber),
      swerveControllerCommand1,
      new AutoFireWithCheck(s_DriveTrain, s_LimeLight, s_Shooter, s_Intake),
      new InstantCommand(() -> s_Shooter.stopShooter())
    );
  }
}
              
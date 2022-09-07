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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HideAndSeek extends SequentialCommandGroup {
  /** Creates a new HideAndSeek. */

  String trajectoryJSON;
  Trajectory trajectory = new Trajectory();

  public HideAndSeek(DriveTrain s_DriveTrain, Intake s_Intake, LimeLight s_LimeLight, Climber s_Climber, Shooter s_Shooter) {

    TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.KMAX_SPEED_METERS_PER_SECOND,
                    Constants.AutoConstants.KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(Constants.DriveTrain.SWERVE_KINEMATICS);

        Trajectory pathTrajectory =
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(5.16, 5.03, Rotation2d.fromDegrees(-59.04)), 
                new Pose2d(4.87, 4.17, Rotation2d.fromDegrees(-109.8)),
                new Pose2d(4.53, 3.32, Rotation2d.fromDegrees(-106.09))),
                config);

        Trajectory pathTrajectory1 =
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(5.98, 7.09, Rotation2d.fromDegrees(51.77)), 
                new Pose2d(6.21, 5.72, Rotation2d.fromDegrees(-47.07))),
                config);


        Trajectory pathTrajectory2 =
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(4.53, 3.32, Rotation2d.fromDegrees(-106.09)),
                new Pose2d(5.24, 5.02, Rotation2d.fromDegrees(68.5)),
                new Pose2d(5.51, 6.01, Rotation2d.fromDegrees(57.62)),
                new Pose2d(5.98, 7.09, Rotation2d.fromDegrees(51.77))),
                config);

        Trajectory pathTrajectory3 =
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(5.51, 6.01, Rotation2d.fromDegrees(57.62)),
                new Pose2d(5.98, 7.09, Rotation2d.fromDegrees(51.77))),
                config);
              
            var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.KP_THETA_CONTROLLER, 0, 0, Constants.AutoConstants.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

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

        SwerveControllerCommand swerveControllerCommand2 =
            new SwerveControllerCommand(
                pathTrajectory2,
                s_DriveTrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.AutoConstants.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.AutoConstants.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_DriveTrain::setModuleStates,
                s_DriveTrain);

        SwerveControllerCommand swerveControllerCommand3 =
            new SwerveControllerCommand(
                pathTrajectory3,
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
      new TwoBallAutoThree(s_DriveTrain, s_Climber, s_Shooter, s_Intake, s_LimeLight),
      new InstantCommand(() -> s_DriveTrain.resetOdometry(pathTrajectory.getInitialPose())),
      new ParallelRaceGroup(new SequentialCommandGroup(swerveControllerCommand, new InstantCommand(() -> s_DriveTrain.resetOdometry(pathTrajectory2.getInitialPose())), swerveControllerCommand2, new InstantCommand(() -> s_DriveTrain.resetOdometry(pathTrajectory3.getInitialPose())), swerveControllerCommand3),
      new IntakeAuto(s_Intake, s_Climber)),
      new InstantCommand(() -> s_DriveTrain.resetOdometry(pathTrajectory1.getInitialPose())),
      swerveControllerCommand1,
      new SpitAuto(s_Climber, s_Intake)
      );
  }
}

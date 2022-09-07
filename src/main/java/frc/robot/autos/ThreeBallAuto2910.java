package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.*;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ThreeBallAuto2910 extends SequentialCommandGroup {

    String trajectoryJSON;
    Trajectory trajectory = new Trajectory();

    public ThreeBallAuto2910(DriveTrain s_Drivetrain, Climber s_Climber, Shooter s_Shooter, Intake s_Intake, LimeLight s_LimeLight){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.KMAX_SPEED_METERS_PER_SECOND,
                    Constants.AutoConstants.KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(Constants.DriveTrain.SWERVE_KINEMATICS);

        Trajectory pathTrajectory =
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(7.48, 1.74, Rotation2d.fromDegrees(-90)), 
                new Pose2d(7.59, 0.64, Rotation2d.fromDegrees(-91.25)),
                new Pose2d(6.36, 0.71, Rotation2d.fromDegrees(178.03))),
                config);

        Trajectory pathTrajectory1 =
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(3.99, 1.34, Rotation2d.fromDegrees(38.88)), 
                new Pose2d(5.19, 2.01, Rotation2d.fromDegrees(42.14))),
                config);

        Trajectory pathTrajectory2 =
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(6.36, 0.71, Rotation2d.fromDegrees(178.03)),
                new Pose2d(3.79, 0.69, Rotation2d.fromDegrees(38)),
                new Pose2d(3.99, 1.34, Rotation2d.fromDegrees(38.88))),
                config);
              
            var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.KP_THETA_CONTROLLER, 0, 0, Constants.AutoConstants.KTHETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                pathTrajectory,
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.AutoConstants.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.AutoConstants.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);

        SwerveControllerCommand swerveControllerCommand1 =
            new SwerveControllerCommand(
                pathTrajectory1,
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.AutoConstants.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.AutoConstants.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);

        SwerveControllerCommand swerveControllerCommand2 =
            new SwerveControllerCommand(
                pathTrajectory2,
                s_Drivetrain::getPose,
                Constants.DriveTrain.SWERVE_KINEMATICS,
                new PIDController(Constants.AutoConstants.KP_X_CONTROLLER, 0, 0),
                new PIDController(Constants.AutoConstants.KP_Y_CONTROLLER, 0, 0),
                thetaController,
                s_Drivetrain::setModuleStates,
                s_Drivetrain);

        addCommands(
            new LowerArmAuto(s_Climber),
            new WaitCommand(1.5),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(pathTrajectory.getInitialPose())),
            new ParallelRaceGroup(new SequentialCommandGroup(swerveControllerCommand, new InstantCommand(() -> s_Drivetrain.resetOdometry(pathTrajectory2.getInitialPose())), swerveControllerCommand2),
            new IntakeAuto(s_Intake, s_Climber)),
            new AutoFireWithCheck(s_Drivetrain, s_LimeLight, s_Shooter, s_Intake),
            new InstantCommand(() -> s_Shooter.hoodSetPosition(Constants.Shooter.NORMAL_RUN)),
            new InstantCommand(() -> s_Shooter.stopShooter()),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(pathTrajectory1.getInitialPose())),
            new ParallelRaceGroup(swerveControllerCommand1,
            new IntakeAuto(s_Intake, s_Climber)),
            new AutoFireWithCheck(s_Drivetrain, s_LimeLight, s_Shooter, s_Intake),
            new InstantCommand(() -> s_Shooter.hoodSetPosition(Constants.Shooter.NORMAL_RUN)),
            new InstantCommand(() -> s_Shooter.stopShooter())
        );
    }
}
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OneBallAuto extends SequentialCommandGroup {

    String trajectoryJSON;
    Trajectory trajectory = new Trajectory();

    public OneBallAuto(DriveTrain s_Drivetrain, Climber s_Climber, Shooter s_Shooter, Intake s_Intake){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.KMAX_SPEED_METERS_PER_SECOND,
                    Constants.AutoConstants.KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(Constants.DriveTrain.SWERVE_KINEMATICS);

        Trajectory pathTrajectory =
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(8.01, 2.32, Rotation2d.fromDegrees(90)), 
                new Pose2d(9.08, 0.48, Rotation2d.fromDegrees(91.07))),
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

        addCommands(
            new LowerArmAuto(s_Climber),
            new InstantCommand(() -> s_Shooter.setFender()),
            new AutoStartUpFender(s_Climber, s_Shooter),
            new WaitCommand(0.375),
            new FireShooterAuto(s_Shooter, s_Intake),
            new InstantCommand(() -> s_Shooter.hoodSetPosition(Constants.Shooter.NORMAL_RUN)),
            new InstantCommand(() -> s_Shooter.stopShooter()),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(pathTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }

    public OneBallAuto(DriveTrain s_Drivetrain, Climber s_Climber, Shooter s_Shooter, Intake s_Intake, int num){
        if(num == 1){
            TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.KMAX_SPEED_METERS_PER_SECOND,
                        Constants.AutoConstants.KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                    .setKinematics(Constants.DriveTrain.SWERVE_KINEMATICS);

            Trajectory pathTrajectory =
                TrajectoryGenerator.generateTrajectory(
                    List.of(new Pose2d(7.15, 4.81, Rotation2d.fromDegrees(-19.57)), 
                    new Pose2d(5.69, 6.69, Rotation2d.fromDegrees(-43.73))),
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

            addCommands(
                new LowerArmAuto(s_Climber),
                new InstantCommand(() -> s_Shooter.setFender()),
                new AutoStartUpFender(s_Climber, s_Shooter),
                new WaitCommand(0.375),
                new FireShooterAuto(s_Shooter, s_Intake),
                new InstantCommand(() -> s_Shooter.hoodSetPosition(Constants.Shooter.NORMAL_RUN)),
                new InstantCommand(() -> s_Shooter.stopShooter()),
                new InstantCommand(() -> s_Drivetrain.resetOdometry(pathTrajectory.getInitialPose())),
                swerveControllerCommand
            );
        }
        else{
            TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.KMAX_SPEED_METERS_PER_SECOND,
                        Constants.AutoConstants.KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                    .setKinematics(Constants.DriveTrain.SWERVE_KINEMATICS);

            Trajectory pathTrajectory =
                TrajectoryGenerator.generateTrajectory(
                    List.of(new Pose2d(7.15, 4.81, Rotation2d.fromDegrees(-19.57)), 
                    new Pose2d(4.05, 3.98, Rotation2d.fromDegrees(8.33))),
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

            addCommands(
                new LowerArmAuto(s_Climber),
                new InstantCommand(() -> s_Shooter.setFender()),
                new AutoStartUpFender(s_Climber, s_Shooter),
                new WaitCommand(0.375),
                new FireShooterAuto(s_Shooter, s_Intake),
                new InstantCommand(() -> s_Shooter.hoodSetPosition(Constants.Shooter.NORMAL_RUN)),
                new InstantCommand(() -> s_Shooter.stopShooter()),
                new InstantCommand(() -> s_Drivetrain.resetOdometry(pathTrajectory.getInitialPose())),
                swerveControllerCommand
            );
        }
    }
}
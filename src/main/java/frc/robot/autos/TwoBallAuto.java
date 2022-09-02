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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoBallAuto extends SequentialCommandGroup {

    String trajectoryJSON;
    Trajectory trajectory = new Trajectory();

    public TwoBallAuto(DriveTrain s_Drivetrain, Climber s_Climber, Shooter s_Shooter, Intake s_Intake, LimeLight s_LimeLight){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.KMAX_SPEED_METERS_PER_SECOND,
                    Constants.AutoConstants.KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(Constants.DriveTrain.SWERVE_KINEMATICS);

        Trajectory pathTrajectory =
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(8.12, 2.73, Rotation2d.fromDegrees(73.61)), 
                new Pose2d(7.72, 1.71, Rotation2d.fromDegrees(-90.0)),
                new Pose2d(7.5, 0.71, Rotation2d.fromDegrees(-90.0)),
                new Pose2d(7.22, 1.85, Rotation2d.fromDegrees(64.51))),
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
            new ParallelCommandGroup(swerveControllerCommand,
            new IntakeAutoWithDelay(s_Intake, s_Climber)),
            new AutoFireWithCheck(s_Drivetrain, s_LimeLight, s_Shooter, s_Intake),
            new InstantCommand(() -> s_Shooter.stopShooter())
        );
    }

    public TwoBallAuto(DriveTrain s_Drivetrain, Climber s_Climber, Shooter s_Shooter, Intake s_Intake, LimeLight s_LimeLight, int num){
        if(num == 1){
            TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.KMAX_SPEED_METERS_PER_SECOND,
                        Constants.AutoConstants.KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                    .setKinematics(Constants.DriveTrain.SWERVE_KINEMATICS);

            Trajectory pathTrajectory = 
                TrajectoryGenerator.generateTrajectory(
                    List.of(new Pose2d(7.58, 3.14, Rotation2d.fromDegrees(70.35)), 
                    new Pose2d(6.78, 2.54, Rotation2d.fromDegrees(-139.0)),
                    new Pose2d(5.3, 2.12, Rotation2d.fromDegrees(-139.03)),
                    new Pose2d(6.72, 2.05, Rotation2d.fromDegrees(41.82))),
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
                new ParallelCommandGroup(swerveControllerCommand,
                new IntakeAutoWithDelay(s_Intake, s_Climber)),
                new AutoFireWithCheck(s_Drivetrain, s_LimeLight, s_Shooter, s_Intake),
                new InstantCommand(() -> s_Shooter.stopShooter())
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
                    List.of(new Pose2d(7.15, 4.89, Rotation2d.fromDegrees(-20.43)), 
                    new Pose2d(6.36, 5.62, Rotation2d.fromDegrees(153.0)),
                    new Pose2d(5.08, 6.14, Rotation2d.fromDegrees(145.30)),
                    new Pose2d(6.19, 5.58, Rotation2d.fromDegrees(-45.0))),
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
                new ParallelCommandGroup(swerveControllerCommand,
                new IntakeAutoWithDelay(s_Intake, s_Climber)),
                new AutoFireWithCheck(s_Drivetrain, s_LimeLight, s_Shooter, s_Intake),
                new InstantCommand(() -> s_Shooter.stopShooter())
            );
        }
    }
}
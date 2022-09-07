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

public class TwoBallAutoThree extends SequentialCommandGroup {

    String trajectoryJSON;
    Trajectory trajectory = new Trajectory();

    public TwoBallAutoThree(DriveTrain s_Drivetrain, Climber s_Climber, Shooter s_Shooter, Intake s_Intake, LimeLight s_LimeLight){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.KMAX_SPEED_METERS_PER_SECOND,
                    Constants.AutoConstants.KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(Constants.DriveTrain.SWERVE_KINEMATICS);

        Trajectory pathTrajectory =
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(6.11, 5.17, Rotation2d.fromDegrees(138.65)), 
                new Pose2d(4.74, 6.48, Rotation2d.fromDegrees(145.01)),
                new Pose2d(5.1, 5.63, Rotation2d.fromDegrees(-16.86))),
                config);

        Trajectory pathTrajectory1 =
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(5.16, 5.03, Rotation2d.fromDegrees(-59.04)),
                new Pose2d(5.16, 5.04, Rotation2d.fromDegrees(-59.04))),
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

        addCommands(
            new LowerArmAuto(s_Climber),
            new WaitCommand(1.5),
            new InstantCommand(() -> s_Drivetrain.resetOdometry(pathTrajectory.getInitialPose())),
            new ParallelRaceGroup(new SequentialCommandGroup(swerveControllerCommand, new InstantCommand(() -> s_Drivetrain.resetOdometry(pathTrajectory1.getInitialPose())), swerveControllerCommand1),
            new IntakeAuto(s_Intake, s_Climber)),
            new AutoFireWithCheck(s_Drivetrain, s_LimeLight, s_Shooter, s_Intake),
            new InstantCommand(() -> s_Shooter.stopShooter())
        );
    }
}
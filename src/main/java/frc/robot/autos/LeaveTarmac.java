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

public class LeaveTarmac extends SequentialCommandGroup {

    String trajectoryJSON;
    Trajectory trajectory = new Trajectory();

    public LeaveTarmac(DriveTrain s_Drivetrain, Climber s_Climber){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.KMAX_SPEED_METERS_PER_SECOND,
                    Constants.AutoConstants.KMAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(Constants.DriveTrain.SWERVE_KINEMATICS);

        Trajectory pathTrajectory =
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(7.87, 2.03, Rotation2d.fromDegrees(85.10)), 
                new Pose2d(7.81, 0.52, Rotation2d.fromDegrees(92.12))),
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
            new InstantCommand(() -> s_Drivetrain.resetOdometry(pathTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}
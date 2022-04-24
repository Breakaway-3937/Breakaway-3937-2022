// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAuto extends SequentialCommandGroup {
  /** Creates a new FiveBallAuto. */
  public FiveBallAuto(DriveTrain s_DriveTrain, LimeLight s_LimeLight, Shooter s_Shooter, Intake s_Intake, Climber s_Climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> s_Shooter.setFender()),
      new AutoStartUpFender(s_Climber, s_Shooter),
      new WaitCommand(0.375),
      new FireShooterAuto(s_Shooter, s_Intake),
      new InstantCommand(() -> s_Shooter.hoodSetPosition(Constants.Shooter.NORMAL_RUN)),
      new AutoIntakePickup(s_DriveTrain, s_Intake, s_Climber, "ThreeBallAuto"),
      new AutoFireWithCheck(s_DriveTrain, s_LimeLight, s_Shooter, s_Intake),
      new AutoIntakePickup(s_DriveTrain, s_Intake, s_Climber, "FiveBallAutoPart2"),
      new IntakeAutoWithPause(s_Intake, s_Climber),
      new Auto(s_DriveTrain, "FiveBallAutoPart3", 6, 4),
      new AutoFireWithCheck(s_DriveTrain, s_LimeLight, s_Shooter, s_Intake),
      new InstantCommand(() -> s_Shooter.stopShooter())
    );
  }
}

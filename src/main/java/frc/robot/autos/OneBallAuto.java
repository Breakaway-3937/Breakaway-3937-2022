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
public class OneBallAuto extends SequentialCommandGroup {
  /** Creates a new OneBallAuto. */
  public OneBallAuto(DriveTrain s_DriveTrain, LimeLight s_LimeLight, Shooter s_Shooter, Intake s_Intake, Climber s_Climber, String whichAuto) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> s_Shooter.setFender()),
      new AutoStartUpFender(s_Climber, s_Shooter),
      new WaitCommand(0.375),
      new FireShooterAuto(s_Shooter, s_Intake),
      new InstantCommand(() -> s_Shooter.hoodSetPosition(Constants.Shooter.NORMAL_RUN)),
      new InstantCommand(() -> s_Shooter.stopShooter()),
      new Auto(s_DriveTrain, whichAuto)
    );
  }
}

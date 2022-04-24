// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAutoWithPause extends ParallelRaceGroup {
  /** Creates a new IntakeAutoWithPause. */
  public IntakeAutoWithPause(Intake s_Intake, Climber s_Climber, boolean intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(0.75),
      new IntakeAuto(s_Intake, s_Climber, intake)
    );
  }
  public IntakeAutoWithPause(Intake s_Intake, Climber s_Climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(0.5),
      new IntakeAuto(s_Intake, s_Climber)
    );
  }
}

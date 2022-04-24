// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntakePickupNoDelay extends ParallelDeadlineGroup {
  /** Creates a new AutoIntakePickupNoDelay. */
  public AutoIntakePickupNoDelay(DriveTrain s_DriveTrain, Intake s_Intake, Climber s_Climber, String whichAuto) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new Auto(s_DriveTrain, whichAuto));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeAuto(s_Intake, s_Climber)
    );
  }
  public AutoIntakePickupNoDelay(DriveTrain s_DriveTrain, Intake s_Intake, Climber s_Climber, String whichAuto, boolean intake) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new Auto(s_DriveTrain, whichAuto));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeAuto(s_Intake, s_Climber, intake)
    );
  }
  public AutoIntakePickupNoDelay(DriveTrain s_DriveTrain, Intake s_Intake, Climber s_Climber, String whichAuto, double maxVel, double maxAccel) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new Auto(s_DriveTrain, whichAuto, maxVel, maxAccel));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeAuto(s_Intake, s_Climber)
    );
  }
  public AutoIntakePickupNoDelay(DriveTrain s_DriveTrain, Intake s_Intake, Climber s_Climber, String whichAuto, boolean intake, double maxVel, double maxAccel) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new Auto(s_DriveTrain, whichAuto, maxVel, maxAccel));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeAuto(s_Intake, s_Climber, intake)
    );
  }
}

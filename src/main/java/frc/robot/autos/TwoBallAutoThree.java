//// Copyright (c) FIRST and other WPILib contributors.
//// Open Source Software; you can modify and/or share it under the terms of
//// the WPILib BSD license file in the root directory of this project.
//
//package frc.robot.autos;
//
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.subsystems.*;
//
//// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
//// information, see:
//// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
//public class TwoBallAutoThree extends SequentialCommandGroup {
//  /** Creates a new TwoBallAutoThree. */
//  public TwoBallAutoThree(Climber s_Climber, Intake s_Intake, DriveTrain s_DriveTrain, LimeLight s_LimeLight, Shooter s_Shooter) {
//    // Add your commands in the addCommands() call, e.g.
//    // addCommands(new FooCommand(), new BarCommand());
//    addCommands(
//      new LowerArmAuto(s_Climber),
//      new WaitCommand(1),
//      new AutoIntakePickupNoDelay(s_DriveTrain, s_Intake, s_Climber, "TwoBallAutoThree"),
//      new AutoFireWithCheck(s_DriveTrain, s_LimeLight, s_Shooter, s_Intake),
//      new InstantCommand(() -> s_Shooter.stopShooter())
//    );
//  }
//}

//// Copyright (c) FIRST and other WPILib contributors.
//// Open Source Software; you can modify and/or share it under the terms of
//// the WPILib BSD license file in the root directory of this project.
//
//package frc.robot.autos;
//
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.subsystems.*;
//
//// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
//// information, see:
//// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
//public class HideAndSeek extends SequentialCommandGroup {
//  /** Creates a new HideAndSeek. */
//  public HideAndSeek(DriveTrain s_DriveTrain, Intake s_Intake, LimeLight s_LimeLight, Climber s_Climber, Shooter s_Shooter) {
//    // Add your commands in the addCommands() call, e.g.
//    // addCommands(new FooCommand(), new BarCommand());
//    addCommands(
//      new TwoBallAutoThree(s_Climber, s_Intake, s_DriveTrain, s_LimeLight, s_Shooter),
//      new AutoIntakePickupNoDelay(s_DriveTrain, s_Intake, s_Climber, "HideAndSeek"),
//      new Auto(s_DriveTrain, "HideAndSeekPart2"),
//      new SpitAuto(s_Climber, s_Intake)
//      );
//  }
//}

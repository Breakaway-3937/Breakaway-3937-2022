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
//public class ThreeBallAuto2910 extends SequentialCommandGroup {
//  /** Creates a new ThreeBallAuto2910. */
//  public ThreeBallAuto2910(DriveTrain s_DriveTrain, Intake s_Intake, Climber s_Climber, Shooter s_Shooter, LimeLight s_LimeLight) {
//    // Add your commands in the addCommands() call, e.g.
//    // addCommands(new FooCommand(), new BarCommand());
//    addCommands(
//      new LowerArmAuto(s_Climber),
//      new WaitCommand(0.375),
//      new AutoIntakePickupNoDelay(s_DriveTrain, s_Intake, s_Climber, "2910ThreeBallAuto", true),
//      new AutoFireWithCheck(s_DriveTrain, s_LimeLight, s_Shooter, s_Intake),
//      new AutoIntakePickupNoDelay(s_DriveTrain, s_Intake, s_Climber, "2910ThreeBallAutoPart2"),
//      new AutoFireWithCheck(s_DriveTrain, s_LimeLight, s_Shooter, s_Intake),
//      new InstantCommand((() -> s_Shooter.stopShooter()))
//    );
//  }
//  public ThreeBallAuto2910(DriveTrain s_DriveTrain, Intake s_Intake, Climber s_Climber, Shooter s_Shooter, LimeLight s_LimeLight, double maxVel, double maxAccel) {
//    // Add your commands in the addCommands() call, e.g.
//    // addCommands(new FooCommand(), new BarCommand());
//    addCommands(
//      new LowerArmAuto(s_Climber),
//      new WaitCommand(0.375),
//      new AutoIntakePickupNoDelay(s_DriveTrain, s_Intake, s_Climber, "2910ThreeBallAuto", true, maxVel, maxAccel),
//      new AutoFireWithCheck(s_DriveTrain, s_LimeLight, s_Shooter, s_Intake),
//      new AutoIntakePickupNoDelay(s_DriveTrain, s_Intake, s_Climber, "2910ThreeBallAutoPart2", maxVel, maxAccel),
//      new AutoFireWithCheck(s_DriveTrain, s_LimeLight, s_Shooter, s_Intake),
//      new InstantCommand((() -> s_Shooter.stopShooter()))
//    );
//  }
//}
//              
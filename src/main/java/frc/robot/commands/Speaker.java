// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class Speaker extends CommandBase {
  private SerialPort arduino;
  /** Creates a new Speaker. */
  public Speaker() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try{
      arduino = new SerialPort(9600, SerialPort.Port.kUSB);
    } catch(Exception e){}

    try{
      arduino = new SerialPort(9600, SerialPort.Port.kUSB1);
    } catch(Exception e){}

    try{
      arduino = new SerialPort(9600, SerialPort.Port.kUSB2);
    } catch(Exception e){}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.m_robotContainer.xboxController.getRawButton(1)){
      arduino.write(new byte[] {0x12}, 1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arduino.write(new byte[] {0x12}, 1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Robot.m_robotContainer.xboxController.getRawButton(2)){
      return true;
    }
    else{
      return false;
    }
  }
}

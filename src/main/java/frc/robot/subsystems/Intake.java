// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  //private final I2C.Port i2cPort = I2C.Port.kOnboard;
  //private Color detectedColor;
  //private String colorString;
  private TalonSRX intakeMotor;
  private TalonSRX stagingMotor;
  private CANSparkMax indexMotor;
  private AnalogInput stagingSensor;
  private AnalogInput intakeSensor;
  //private ColorSensorV3 intakeColorSensor;
  private NetworkTableEntry tableStagingSensor, tableIntakeSensor;
  //private NetworkTableEntry tableColorRed, tableColorBlue, tableColor;
  
  /** Creates a new Intake. */
  public Intake() {
    //colorString = DriverStation.getAlliance().name();
    intakeMotor = new TalonSRX(Constants.Intake.INTAKE_MOTOR_ID);
    intakeMotor.configFactoryDefault();
    indexMotor = new CANSparkMax(Constants.Intake.INDEX_MOTOR_ID, MotorType.kBrushless);
    indexMotor.restoreFactoryDefaults();
    //indexMotor.burnFlash();
    stagingMotor = new TalonSRX(Constants.Intake.STAGING_MOTOR_ID);
    stagingMotor.configFactoryDefault();
    stagingSensor = new AnalogInput(1);
    intakeSensor = new AnalogInput(0);
    //intakeColorSensor = new ColorSensorV3(i2cPort);
    stagingSensor.resetAccumulator();
    stagingSensor.initAccumulator();
    intakeSensor.resetAccumulator();
    intakeSensor.initAccumulator();
    tableStagingSensor = Shuffleboard.getTab("Intake").add("Stage Sensor", getValue("stagingSensor")).withPosition(1, 0).getEntry();
    tableIntakeSensor = Shuffleboard.getTab("Intake").add("Intake Sensor", getValue("intakeSensor")).withPosition(0, 0).getEntry();
    //tableColorRed = Shuffleboard.getTab("Intake").add("Red", 0.0).withPosition(0, 1).getEntry();
    //tableColorBlue = Shuffleboard.getTab("Intake").add("Blue", 0.0).withPosition(1, 1).getEntry();
    //tableColor = Shuffleboard.getTab("Intake").add("Color", colorString).withPosition(2, 1).getEntry(); 

  }

  public int getValue(String analog){
      if(analog.equals("stagingSensor")){
        return stagingSensor.getValue();
      }
      else if(analog.equals("intakeSensor")){
        return intakeSensor.getValue();
      }
      else{
        return -1;
      }
  }

  public void intakeForward(double speed){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void intakeReverse(double speed){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, -speed);
  }
  public void intakeForward(){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.9);
  }

  public void intakeReverse(){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, -0.8);
  }

  public void intakeStop(){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  public void indexForward(double speed){
    indexMotor.set(speed);
  }

  public void indexReverse(double speed){
    indexMotor.set(-speed);
  }

  public void indexForward(){
    indexMotor.set(0.4);
  }

  public void indexReverse(){
    indexMotor.set(-0.5);
  }

  public void indexStop(){
    indexMotor.set(0.0);
  }

  public void stagingForward(double speed){
    stagingMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void stagingReverse(double speed){
    stagingMotor.set(TalonSRXControlMode.PercentOutput, -speed);
  }

  public void stagingForward(){
    stagingMotor.set(TalonSRXControlMode.PercentOutput, 0.35);
  }

  public void stagingReverse(){
    stagingMotor.set(TalonSRXControlMode.PercentOutput, -0.8);
  }

  public void stagingStop(){
    stagingMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  /*public String getColor(){
    return colorString;
  }*/

  /*public void colorMatch(){
    detectedColor = intakeColorSensor.getColor();
    if (detectedColor.blue > 0.3) {
      colorString = "Blue";
    } 
    else if(detectedColor.red > 0.3){
      colorString = "Red";
    } 
    else {
       colorString = DriverStation.getAlliance().name();
    }
  }*/
  public boolean ballDetect(String sensor){
    if(getValue(sensor) < 4000){
      return false;
    }
    else{
      return true;
    }
  }

  public boolean intakeEmpty(){
    if (!ballDetect("intakeSensor") && !ballDetect("stagingSensor")){
      return true;
    }
    else{
      return false;
    }
  }

  public void spitAll(){
    intakeStop();
    indexStop();
    stagingStop();
    intakeReverse();
    indexReverse();
    stagingReverse();
  }

  public void stopAll(){
    intakeStop();
    indexStop();
    stagingStop();
  }

  /*public boolean colorTest(){
    if(ballDetect("intakeSensor")){
      if(colorString.equals(DriverStation.getAlliance().name())){
        return true;
      }
      else{
        return false;
      }
    }
    else{
      return false;
    }
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //colorMatch();
    tableIntakeSensor.setDouble(getValue("intakeSensor"));
    tableStagingSensor.setDouble(getValue("stagingSensor"));
    //tableColor.setString(colorString);
    //tableColorRed.setDouble(detectedColor.red);
    //tableColorBlue.setDouble(detectedColor.blue);
  }
}
  

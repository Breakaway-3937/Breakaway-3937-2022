// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private double defaultShooterkP, defaultShooterkI, defaultShooterkD, defaultShooterkIz, defaultShooterkFF;
  private double defaultHoodkP, defaultHoodkI, defaultHoodkD, defaultHoodkIz, defaultHoodkFF, defaultHoodkMaxOutput, defaultHoodkMinOutput;
  private WPI_TalonFX shooterMotor;
  private CANSparkMax hoodMotor;
  private RelativeEncoder hoodEncoder;
  private SparkMaxPIDController hoodPIDController; 
  private TalonFXSensorCollection shooterEncoder;
  private NetworkTableEntry tableHoodEncoder, tableShooterVelocity;
  public double currentPosition;
  public double shooterVelocity;
  public double hoodRequestedPosition = Constants.Shooter.NORMAL_RUN;
  public double shooterRequestedVelocity = 2000;
  public boolean takeLimeLight = true;
  private LimeLight s_LimeLight;
  private NetworkTableEntry hoodOffsetLimeLight, shooterOffsetLimeLight, hoodOffsetTarmac, shooterOffsetTarmac, hoodOffsetFender, shooterOffsetFender, hoodOffsetLaunch, shooterOffsetLaunch;
  public double hoodLime, shooterLime, hoodTarmac, shooterTarmac, hoodFender, shooterFender, shooterLaunch, hoodLaunch = 0;
  public boolean fender = false;
  public boolean tarmac = false;
  public boolean launchPad = false;
  private int kPIDLoopIdx = 0;
  private double ticksPerRPM = 600.0/2048.0;

  /** Creates a new Shooter. */
  public Shooter(LimeLight s_LimeLight) {
    this.s_LimeLight = s_LimeLight;
    setValues();
    shooterMotor = new WPI_TalonFX(Constants.Shooter.SHOOTER_MOTOR_ID);
    configShooterMotor();
    hoodMotor = new CANSparkMax(Constants.Shooter.HOOD_MOTOR_ID, MotorType.kBrushless);
    configHoodMotor();
    
    
    tableHoodEncoder = Shuffleboard.getTab("Shooter").add("Encoder", currentPosition).withPosition(0, 0).getEntry();
    tableShooterVelocity = Shuffleboard.getTab("Shooter").add("Velocity", shooterVelocity).withPosition(1, 0).getEntry();
    hoodOffsetLimeLight = Shuffleboard.getTab("Shooter").add("Hood Offset LimeLight", -3).withPosition(2, 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -50, "max", 50)).getEntry();
    shooterOffsetLimeLight = Shuffleboard.getTab("Shooter").add("Shooter Offset LimeLight", -85).withPosition(4, 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -500, "max", 500)).getEntry();
    hoodOffsetTarmac = Shuffleboard.getTab("Shooter").add("Hood Offset Tarmac", 0).withPosition(0, 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -50, "max", 50)).getEntry();
    shooterOffsetTarmac = Shuffleboard.getTab("Shooter").add("Shooter Offset Tarmac", 0).withPosition(2, 1).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -500, "max", 500)).getEntry();
    hoodOffsetFender = Shuffleboard.getTab("Shooter").add("Hood Offset Fender", 0).withPosition(0, 2).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -50, "max", 50)).getEntry();
    shooterOffsetFender = Shuffleboard.getTab("Shooter").add("Shooter Offset Fender", 0).withPosition(2, 2).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -500, "max", 500)).getEntry();
    hoodOffsetLaunch = Shuffleboard.getTab("Shooter").add("Hood Offset Launchpad", 0).withPosition(0, 3).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -50, "max", 50)).getEntry();
    shooterOffsetLaunch = Shuffleboard.getTab("Shooter").add("Shooter Offset Launchpad", 0).withPosition(2, 3).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -500, "max", 500)).getEntry();

  }

  public void runShooter(double speed) {
    shooterMotor.set(TalonFXControlMode.Velocity, speed/ticksPerRPM);
  }

  public void stopShooter(){
    shooterMotor.stopMotor();
  }

  public void hoodRunForward(double speed) {
    hoodMotor.set(speed);
  }

  public void hoodRunReverse(double speed) {
    hoodMotor.set(-speed);
  }

  public double getHoodEncoder(){
    return hoodEncoder.getPosition();
  }

  public void stopHood(){
    hoodMotor.stopMotor(); 
  }

  public void hoodSetPosition(double position){
    hoodPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  }


  public void hoodMoveToSetting(){
    hoodSetPosition(hoodRequestedPosition);
  }
  public void hoodMoveToSetting(double hoodOff){
    hoodSetPosition(hoodRequestedPosition + hoodOff);
  }

  public void shooterRunToSetting(){
    runShooter(shooterRequestedVelocity);
  }
  public void shooterRunToSetting(double shooterOff){
    runShooter(shooterRequestedVelocity + shooterOff);
  }

  public void setTarmac(){
    takeLimeLight = false;
    fender = false;
    launchPad = false;
    tarmac = true;
    hoodRequestedPosition = Constants.Shooter.EDGE_OF_TARMAC;
    shooterRequestedVelocity = Constants.Shooter.EDGE_OF_TARMAC_VELOCITY;
  }

  public void setFender(){
    takeLimeLight = false;
    tarmac = false;
    launchPad = false;
    fender = true;
    hoodRequestedPosition = Constants.Shooter.FENDER;
    shooterRequestedVelocity = Constants.Shooter.FENDER_VELOCITY;
  }

  public void setLimeLight(){
    fender = false;
    tarmac = false;
    launchPad = false;
    takeLimeLight = true;
  }


  public void setLaunchPad(){
    takeLimeLight = false;
    fender = false;
    tarmac = false;
    launchPad = true;
    hoodRequestedPosition = Constants.Shooter.LAUNCH_PAD;
    shooterRequestedVelocity = Constants.Shooter.LAUNCH_PAD_VELOCITY;
  }
  
  public boolean shooterAtVelocity(){
    if(fender){
      if(shooterVelocity >= (shooterRequestedVelocity + shooterFender - 0)){
        return true;
      }
      else{
        return false;
      }
    }
    else if(tarmac){
      if(shooterVelocity >= (shooterRequestedVelocity + shooterTarmac - 0)) {
        return true;
      }
      else{
        return false;
      }
    }
    else if(takeLimeLight){
      if(shooterVelocity >= (shooterRequestedVelocity + shooterLime - 0)) {
        return true;
      }
      else{
        return false;
      }
    }
    else if(launchPad){
      if(shooterVelocity >= (shooterRequestedVelocity + shooterLaunch - 0)) {
        return true;
      }
      else{
        return false;
      }
    }
    else{
      return false;
    }
  }
  public boolean hoodAtPosition(){
    if(fender){
      if((((int) currentPosition) >= (hoodRequestedPosition + hoodFender - 1)) && ( ((int) currentPosition) <= (hoodRequestedPosition + hoodFender + 1))){
        return true;
      }
      else{
        return false;
      }
    }
    else if(tarmac){
      if((((int) currentPosition) >= (hoodRequestedPosition + hoodTarmac - 1)) && ( ((int) currentPosition) <= (hoodRequestedPosition + hoodTarmac + 1))){
        return true;
      }
      else{
        return false;
      }
    }
    else if(takeLimeLight){
      if((((int) currentPosition) >= (hoodRequestedPosition + hoodLime - 1)) && ( ((int) currentPosition) <= (hoodRequestedPosition + hoodLime + 1))){
        return true;
      }
      else{
        return false;
      }
    }
    else if(launchPad){
      if((((int) currentPosition) >= (hoodRequestedPosition + hoodLaunch - 1)) && ( ((int) currentPosition) <= (hoodRequestedPosition + hoodLaunch + 1))){
        return true;
      }
      else{
        return false;
      }
    }
    else{
      return false;
    }
  }
  public void clearFaults(){
    shooterMotor.clearStickyFaults();
  }
  private void configShooterMotor(){        
    shooterMotor.setInverted(Constants.Shooter.SHOOTER_MOTOR_INVERT);

    shooterEncoder = shooterMotor.getSensorCollection();
    shooterEncoder.setIntegratedSensorPosition(0, 0);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.configVoltageCompSaturation(12);
    shooterMotor.enableVoltageCompensation(true);

    // set PID coefficients
    shooterMotor.config_kP(kPIDLoopIdx, defaultShooterkP);
    shooterMotor.config_kI(kPIDLoopIdx, defaultShooterkI);
    shooterMotor.config_kD(kPIDLoopIdx, defaultShooterkD);
    shooterMotor.config_IntegralZone(kPIDLoopIdx, defaultShooterkIz);
    shooterMotor.config_kF(kPIDLoopIdx, defaultShooterkFF);
    shooterMotor.configNominalOutputForward(0);
    shooterMotor.configNominalOutputReverse(0);
    shooterMotor.configPeakOutputForward(1);
    shooterMotor.configPeakOutputReverse(-1);


    //shooterMotor.burnFlash();
   }

  private void configHoodMotor(){
    hoodMotor.restoreFactoryDefaults();
    hoodEncoder = hoodMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    hoodPIDController = hoodMotor.getPIDController();
    hoodPIDController.setFeedbackDevice(hoodEncoder);
    hoodEncoder.setPosition(0);
    hoodMotor.setIdleMode(IdleMode.kCoast);

    // set PID coefficients
    hoodPIDController.setP(defaultHoodkP);
    hoodPIDController.setI(defaultHoodkI);
    hoodPIDController.setD(defaultHoodkD);
    hoodPIDController.setIZone(defaultHoodkIz);
    hoodPIDController.setFF(defaultHoodkFF);
    hoodPIDController.setOutputRange(defaultHoodkMinOutput, defaultHoodkMaxOutput);

    //hoodMotor.burnFlash();
  }

  public void setValues(){
    defaultShooterkP = Constants.Shooter.SHOOTER_KP;
    defaultShooterkI = Constants.Shooter.SHOOTER_KI;
    defaultShooterkD = Constants.Shooter.SHOOTER_KD;
    defaultShooterkIz = Constants.Shooter.SHOOTER_KIZ;
    if(Constants.COMP_BOT){
      defaultShooterkFF = Constants.Shooter.SHOOTER_KFF_COMP;
    }
    else{
      defaultShooterkFF = Constants.Shooter.SHOOTER_KFF_PRACTICE;
    }
    defaultHoodkP = Constants.Shooter.HOOD_KP;
    defaultHoodkI = Constants.Shooter.HOOD_KI;
    defaultHoodkD = Constants.Shooter.HOOD_KD;
    defaultHoodkIz = Constants.Shooter.HOOD_KIZ;
    defaultHoodkFF = Constants.Shooter.HOOD_KFF;
    defaultHoodkMaxOutput = Constants.Shooter.HOOD_KMAX_OUTPUT;
    defaultHoodkMinOutput = Constants.Shooter.HOOD_KMIN_OUTPUT;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentPosition =  hoodEncoder.getPosition();
    shooterVelocity = shooterEncoder.getIntegratedSensorVelocity() * ticksPerRPM * 0.8 * -1;
    tableHoodEncoder.setDouble(currentPosition);
    tableShooterVelocity.setDouble(shooterVelocity);
    if(takeLimeLight){
      hoodRequestedPosition = s_LimeLight.limeLightHood();
      shooterRequestedVelocity = s_LimeLight.limeLightShooter();
    }
    hoodLime = hoodOffsetLimeLight.getDouble(0.0);
    shooterLime = shooterOffsetLimeLight.getDouble(0.0);
    hoodTarmac = hoodOffsetTarmac.getDouble(0.0);
    shooterTarmac = shooterOffsetTarmac.getDouble(0.0);
    hoodFender = hoodOffsetFender.getDouble(0.0);
    shooterFender = shooterOffsetFender.getDouble(0.0);
    hoodLaunch = hoodOffsetLaunch.getDouble(0.0);
    shooterLaunch = shooterOffsetLaunch.getDouble(0.0);
  }
}
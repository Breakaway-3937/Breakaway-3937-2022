// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private boolean isRaiseArm, isStageOne, isStageTwo;
  private SparkMaxPIDController m_pidControllerRaiseArm, m_pidControllerStageOne, m_pidControllerStageTwo;
  private RelativeEncoder m_encoderRaiseArm, m_encoderStageOne, m_encoderStageTwo;
  private double defaultkPRaiseArm, defaultkIRaiseArm, defaultkDRaiseArm, defaultkIzRaiseArm, defaultkFFRaiseArm, defaultkMaxOutputRaiseArm,defaultkMinOutputRaiseArm;
  private double defaultkPStageOne, defaultkIStageOne, defaultkDStageOne, defaultkIzStageOne, defaultkFFStageOne, defaultkMaxOutputStageOne, defaultkMinOutputStageOne;
  private double defaultkPStageTwo, defaultkIStageTwo, defaultkDStageTwo, defaultkIzStageTwo, defaultkFFStageTwo, defaultkMaxOutputStageTwo, defaultkMinOutputStageTwo;
  private CANSparkMax raiseArm;
  private CANSparkMax stageOne;
  private CANSparkMax stageTwo;
  private DoubleSolenoid raiseArmPneu;
  private DoubleSolenoid stageOnePneu;
  private DoubleSolenoid stageTwoPneu;
  private NetworkTableEntry tableIsRaiseArm, tableIsStageOne, tableIsStageTwo;
  private NetworkTableEntry tableEncoderRaiseArm, tableEncoderStageOne, tableEncoderStageTwo;
  public double currentPositionStageOne = 0;
  public double currentPositionStageTwo = 0;
  public double currentPositionRaiseArm = 0;


  /** Creates a new Climber. */
  public Climber() {
    setValues();
    raiseArm = new CANSparkMax(Constants.Climber.RAISE_ARM_ID, MotorType.kBrushless);
    stageOne = new CANSparkMax(Constants.Climber.STAGE_ONE_ID, MotorType.kBrushless);
    stageTwo = new CANSparkMax(Constants.Climber.STAGE_TWO_ID, MotorType.kBrushless);
    raiseArmPneu = new DoubleSolenoid(Constants.Climber.PCM_DEVICE_ID, PneumaticsModuleType.CTREPCM, 0, 1);
    stageOnePneu = new DoubleSolenoid(Constants.Climber.PCM_DEVICE_ID, PneumaticsModuleType.CTREPCM, 2, 3);
    stageTwoPneu = new DoubleSolenoid(Constants.Climber.PCM_DEVICE_ID, PneumaticsModuleType.CTREPCM, 4, 5);
    //PreStaging Arms
    raiseArmPneu.set(Value.kReverse);
    stageOnePneu.set(Value.kReverse);
    stageTwoPneu.set(Value.kReverse);
    reset();
    m_encoderRaiseArm.setPosition(0.0);
    m_encoderStageOne.setPosition(0.0);
    m_encoderStageTwo.setPosition(0.0);



    tableEncoderRaiseArm = Shuffleboard.getTab("Climber").add("RaiseArm Encoder", getEncoder(m_encoderRaiseArm)).withPosition(0, 0).getEntry();
    tableEncoderStageOne = Shuffleboard.getTab("Climber").add("StageOne Encoder", getEncoder(m_encoderStageOne)).withPosition(1, 0).getEntry();
    tableEncoderStageTwo = Shuffleboard.getTab("Climber").add("StageTwo Encoder", getEncoder(m_encoderStageTwo)).withPosition(2, 0).getEntry();


    tableIsRaiseArm = Shuffleboard.getTab("Climber").add("RaiseArm", isRaiseArm).withPosition(0, 1).getEntry();
    tableIsStageOne = Shuffleboard.getTab("Climber").add("StageOne", isStageOne).withPosition(1, 1).getEntry();
    tableIsStageTwo = Shuffleboard.getTab("Climber").add("StageTwo", isStageTwo).withPosition(2, 1).getEntry();

  }

  public void engage(String solenoid){
    if(solenoid == "raiseArmPneu"){
      raiseArmPneu.set(Value.kOff);  
      raiseArmPneu.set(Value.kForward);
      isRaiseArm = true;
    }
    else if(solenoid == "stageOnePneu"){
      stageOnePneu.set(Value.kOff);  
      stageOnePneu.set(Value.kForward);
      isStageOne = true;
    }
    else if(solenoid == "stageTwoPneu"){
      stageTwoPneu.set(Value.kOff);  
      stageTwoPneu.set(Value.kForward);
      isStageTwo = true;
    }
    
  }
  public void disengage(String solenoid){
    if(solenoid == "raiseArmPneu"){
      raiseArmPneu.set(Value.kOff);
      raiseArmPneu.set(Value.kReverse);
      isRaiseArm = false;
    }
    else if(solenoid == "stageOnePneu"){
      stageOnePneu.set(Value.kOff);
      stageOnePneu.set(Value.kReverse);
      isStageOne = false;
    }
    else if(solenoid == "stageTwoPneu"){
      stageTwoPneu.set(Value.kOff);
      stageTwoPneu.set(Value.kReverse);
      isStageTwo = false;
    }
  }
  public void stop(String solenoid){
    if(solenoid == "raiseArmPneu"){
      raiseArmPneu.set(Value.kOff);
      isRaiseArm = false;
    }
    else if(solenoid == "stageOnePneu"){
      stageOnePneu.set(Value.kOff);
      isStageOne = false;
    }
    else if(solenoid == "stageTwoPneu"){
      stageTwoPneu.set(Value.kOff);
      isStageTwo = false;
    }
  }
  public void engageAll(){
    engage("raiseArmPneu");
    engage("stageOnePneu");
    engage("stageTwoPneu");
  }
  public void disengageAll(){
    disengage("raiseArmPneu");
    disengage("stageOnePneu");
    disengage("stageTwoPneu");
  }
  public void lowerArm(){
    disengage("raiseArmPneu");
    raiseArmSetPosition(0);
  }

  public void runArm(double speed){
    raiseArm.set(speed);
  }

  public void stopArm(){
    raiseArm.stopMotor();;
  }

  public void runStageOne(double speed){
    stageOne.set(speed);
  }

  public void stopStageOne(){
    stageOne.stopMotor();;
  }

  public void raiseArmSetPosition(double position){
    m_pidControllerRaiseArm.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }

  public void stageOneSetPosition(double position){
    m_pidControllerStageOne.setReference(position, CANSparkMax.ControlType.kPosition);
  }
  public void stageOneSetSmartPosition(double position){
    m_pidControllerStageOne.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }

  public void stageTwoSetPosition(double position){
    m_pidControllerStageTwo.setReference(position, CANSparkMax.ControlType.kSmartMotion);
  }

  public void runStageTwo(double speed){
    stageTwo.set(speed);
  }

  public void stopStageTwo(){
    stageTwo.stopMotor();;
  }

  public void resetEncoder(RelativeEncoder encoder){
    encoder.setPosition(0.0);
  }

  public double getEncoder(RelativeEncoder encoder){
    return encoder.getPosition();
  }
  public void setStageOnePIDPosControl(){
    // set PID coefficients
    m_pidControllerStageOne.setP(defaultkPStageOne);
    m_pidControllerStageOne.setI(defaultkIStageOne);
    m_pidControllerStageOne.setD(defaultkDStageOne);
    m_pidControllerStageOne.setIZone(defaultkIzStageOne);
    m_pidControllerStageOne.setFF(defaultkFFStageOne);
    m_pidControllerStageOne.setOutputRange(defaultkMinOutputStageOne, defaultkMaxOutputStageOne);
  }

  public void setStageOnePIDSmartControl(){
    // set PID coefficients
    m_pidControllerStageOne.setP(Constants.Climber.KP_STAGE_ONE_CLIMB);
    m_pidControllerStageOne.setI(Constants.Climber.KI_STAGE_ONE_CLIMB);
    m_pidControllerStageOne.setD(Constants.Climber.KD_STAGE_ONE_CLIMB);
    m_pidControllerStageOne.setIZone(Constants.Climber.KIZ_STAGE_ONE_CLIMB);
    m_pidControllerStageOne.setFF(Constants.Climber.KFF_STAGE_ONE_CLIMB);
    m_pidControllerStageOne.setSmartMotionMaxVelocity(Constants.Climber.MAX_VELOCITY_STAGE_ONE, 0);
    m_pidControllerStageOne.setSmartMotionMaxAccel(Constants.Climber.MAX_ACCEL_STAGE_ONE, 0);
    m_pidControllerStageOne.setOutputRange(defaultkMinOutputStageOne, defaultkMaxOutputStageOne);
  }


  public void setValues(){
    defaultkPRaiseArm = Constants.Climber.KP_RAISE_ARM;
    defaultkIRaiseArm = Constants.Climber.KI_RAISE_ARM;
    defaultkDRaiseArm = Constants.Climber.KD_RAISE_ARM;
    defaultkIzRaiseArm = Constants.Climber.KIZ_RAISE_ARM;
    defaultkFFRaiseArm = Constants.Climber.KFF_RAISE_ARM;
    defaultkMaxOutputRaiseArm = Constants.Climber.KMAX_OUTPUT_RAISE_ARM;
    defaultkMinOutputRaiseArm = Constants.Climber.KMIN_OUTPUT_RAISE_ARM;
    defaultkPStageOne = Constants.Climber.KP_STAGE_ONE;
    defaultkIStageOne = Constants.Climber.KI_STAGE_ONE;
    defaultkDStageOne = Constants.Climber.KD_STAGE_ONE;
    defaultkIzStageOne = Constants.Climber.KIZ_STAGE_ONE;
    defaultkFFStageOne = Constants.Climber.KFF_STAGE_ONE;
    defaultkMaxOutputStageOne = Constants.Climber.KMAX_OUTPUT_STAGE_ONE;
    defaultkMinOutputStageOne = Constants.Climber.KMIN_OUTPUT_STAGE_ONE;
    defaultkPStageTwo = Constants.Climber.KP_STAGE_TWO;
    defaultkIStageTwo = Constants.Climber.KI_STAGE_TWO;
    defaultkDStageTwo = Constants.Climber.KD_STAGE_TWO;
    defaultkIzStageTwo = Constants.Climber.KIZ_STAGE_TWO;
    defaultkFFStageTwo = Constants.Climber.KFF_STAGE_TWO;
    defaultkMaxOutputStageTwo = Constants.Climber.KMAX_OUTPUT_STAGE_TWO;
    defaultkMinOutputStageTwo = Constants.Climber.KMIN_OUTPUT_STAGE_TWO;
  }

  private void reset(){
    /**
    * The restoreFactoryDefaults method can be used to reset the configuration parameters
    * in the SPARK MAX to their factory default state. If no argument is passed, these
    * parameters will not persist between power cycles
    */
    raiseArm.restoreFactoryDefaults();
    raiseArm.setIdleMode(IdleMode.kBrake);

    /**
    * In order to use PID functionality for a controller, a SparkMaxPIDController object
    * is constructed by calling the getPIDController() method on an existing
    * CANSparkMax object
    */
    m_pidControllerRaiseArm = raiseArm.getPIDController();

    // Encoder object created to display position values
    m_encoderRaiseArm = raiseArm.getEncoder();
    raiseArm.setInverted(true);


    // set PID coefficients
    m_pidControllerRaiseArm.setP(defaultkPRaiseArm);
    m_pidControllerRaiseArm.setI(defaultkIRaiseArm);
    m_pidControllerRaiseArm.setD(defaultkDRaiseArm);
    m_pidControllerRaiseArm.setIZone(defaultkIzRaiseArm);
    m_pidControllerRaiseArm.setFF(defaultkFFRaiseArm);
    m_pidControllerRaiseArm.setSmartMotionMaxVelocity(Constants.Climber.MAX_VELOCITY_RAISE_ARM, 0);
    m_pidControllerRaiseArm.setSmartMotionMaxAccel(Constants.Climber.MAX_ACCEL_RAISE_ARM, 0);
    m_pidControllerRaiseArm.setOutputRange(defaultkMinOutputRaiseArm, defaultkMaxOutputRaiseArm);

  
    stageOne.restoreFactoryDefaults();
    stageOne.setIdleMode(IdleMode.kCoast);
  

    m_pidControllerStageOne = stageOne.getPIDController();

  
    // Encoder object created to display position values
    m_encoderStageOne = stageOne.getEncoder();


    //Set Initial PID to position control
    setStageOnePIDPosControl();

    stageTwo.restoreFactoryDefaults();
    stageTwo.setIdleMode(IdleMode.kCoast);

    m_pidControllerStageTwo = stageTwo.getPIDController();

    // Encoder object created to display position values
    m_encoderStageTwo = stageTwo.getEncoder();
    stageTwo.setInverted(true);


    // set PID coefficients
    m_pidControllerStageTwo.setP(defaultkPStageTwo);
    m_pidControllerStageTwo.setI(defaultkIStageTwo);
    m_pidControllerStageTwo.setD(defaultkDStageTwo);
    m_pidControllerStageTwo.setIZone(defaultkIzStageTwo);
    m_pidControllerStageTwo.setFF(defaultkFFStageTwo);
    m_pidControllerStageTwo.setSmartMotionMaxVelocity(Constants.Climber.MAX_VELOCITY_STAGE_TWO, 0);
    m_pidControllerStageTwo.setSmartMotionMaxAccel(Constants.Climber.MAX_ACCEL_STAGE_TWO, 0);
    m_pidControllerStageTwo.setOutputRange(defaultkMinOutputStageTwo, defaultkMaxOutputStageTwo);

    m_pidControllerRaiseArm.setReference(0, CANSparkMax.ControlType.kPosition);

    m_pidControllerStageOne.setReference(0, CANSparkMax.ControlType.kPosition);

    m_pidControllerStageTwo.setReference(0, CANSparkMax.ControlType.kPosition);

    //raiseArm.burnFlash();
    //stageOne.burnFlash();
    //stageTwo.burnFlash();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentPositionStageOne = getEncoder(m_encoderStageOne);
    currentPositionStageTwo = getEncoder(m_encoderStageTwo);
    currentPositionRaiseArm = getEncoder(m_encoderRaiseArm);
    tableEncoderRaiseArm.setDouble(getEncoder(m_encoderRaiseArm));
    tableEncoderStageOne.setDouble(getEncoder(m_encoderStageOne));
    tableEncoderStageTwo.setDouble(getEncoder(m_encoderStageTwo));
    tableIsRaiseArm.setBoolean(isRaiseArm);
    tableIsStageOne.setBoolean(isStageOne);
    tableIsStageTwo.setBoolean(isStageTwo);
  }
}

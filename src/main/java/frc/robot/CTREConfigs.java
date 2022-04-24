package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;


    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.DriveTrain.ANGLE_ENABLE_CURRENT_LIMIT, 
            Constants.DriveTrain.ANGLE_CONTINUOUS_CURRENT_LIMIT, 
            Constants.DriveTrain.ANGLE_PEAK_CURRENT_LIMIT, 
            Constants.DriveTrain.ANGLE_PEAK_CURRENT_DURATION);

        swerveAngleFXConfig.slot0.kP = Constants.DriveTrain.ANGLE_KP;
        swerveAngleFXConfig.slot0.kI = Constants.DriveTrain.ANGLE_KI;
        swerveAngleFXConfig.slot0.kD = Constants.DriveTrain.ANGLE_KD;
        swerveAngleFXConfig.slot0.kF = Constants.DriveTrain.ANGLE_KF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.DriveTrain.DRIVE_ENABLE_CURRENT_LIMIT, 
            Constants.DriveTrain.DRIVE_CONTINUOUS_CURRENT_LIMIT, 
            Constants.DriveTrain.DRIVE_PEAK_CURRENT_LIMIT, 
            Constants.DriveTrain.DRIVE_PEAK_CURRENT_DURATION);

        swerveDriveFXConfig.slot0.kP = Constants.DriveTrain.DRIVE_KP;
        swerveDriveFXConfig.slot0.kI = Constants.DriveTrain.DRIVE_KI;
        swerveDriveFXConfig.slot0.kD = Constants.DriveTrain.DRIVE_KD;
        swerveDriveFXConfig.slot0.kF = Constants.DriveTrain.DRIVE_KF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = Constants.DriveTrain.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.closedloopRamp = Constants.DriveTrain.CLOSED_LOOP_RAMP;

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.DriveTrain.CANCODER_INVERT;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        

    }

}
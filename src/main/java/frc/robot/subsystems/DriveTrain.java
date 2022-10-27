package frc.robot.subsystems;



import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;
    private NetworkTableEntry mod0Cancoder, mod1Cancoder, mod2Cancoder, mod3Cancoder;
    private NetworkTableEntry gyroHeading;
    private boolean jeffords;

    public DriveTrain() {
        gyro = new PigeonIMU(Constants.DriveTrain.PIGEON_ID);
        gyro.configFactoryDefault();
        zeroGyro();
        
        swerveOdometry = new SwerveDriveOdometry(Constants.DriveTrain.SWERVE_KINEMATICS, getYaw());
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.DriveTrain.Mod0.CONSTANTS),
            new SwerveModule(1, Constants.DriveTrain.Mod1.CONSTANTS),
            new SwerveModule(2, Constants.DriveTrain.Mod2.CONSTANTS),
            new SwerveModule(3, Constants.DriveTrain.Mod3.CONSTANTS)
        };
        mod0Cancoder = Shuffleboard.getTab("Drive").add("Mod 0 Cancoder", mSwerveMods[0].getState().angle.getDegrees()).withPosition(0, 0).getEntry();
        mod1Cancoder = Shuffleboard.getTab("Drive").add("Mod 1 Cancoder", mSwerveMods[1].getState().angle.getDegrees()).withPosition(1, 0).getEntry();
        mod2Cancoder = Shuffleboard.getTab("Drive").add("Mod 2 Cancoder", mSwerveMods[2].getState().angle.getDegrees()).withPosition(2, 0).getEntry();
        mod3Cancoder = Shuffleboard.getTab("Drive").add("Mod 3 Cancoder", mSwerveMods[3].getState().angle.getDegrees()).withPosition(3, 0).getEntry();
        gyroHeading = Shuffleboard.getTab("Drive").add("Gyro", gyro.getYaw()).withPosition(0, 1).getEntry();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.DriveTrain.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveTrain.MAX_SPEED);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.DriveTrain.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void gyro180() {
        gyro.setCompassAngle(180);
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public void zeroWheels(){
        gyro.setFusedHeading(0);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.DriveTrain.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }


    public boolean test1()
    {
        jeffords = false;
        return jeffords;
    }

    public boolean test2()
    {
        jeffords = true;
        return jeffords;
    }

    public boolean test3()
    {
        return jeffords;
    }

    

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getStates());  
        
        mod0Cancoder.setDouble(mSwerveMods[0].getCanCoder().getDegrees());
        mod1Cancoder.setDouble(mSwerveMods[1].getCanCoder().getDegrees());
        mod2Cancoder.setDouble(mSwerveMods[2].getCanCoder().getDegrees());
        mod3Cancoder.setDouble(mSwerveMods[3].getCanCoder().getDegrees());
        
        gyroHeading.setDouble(gyro.getYaw());
    }   
}
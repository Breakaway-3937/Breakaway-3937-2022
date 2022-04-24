package frc.lib.util;

import frc.robot.Constants;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final double angleOffset;
    public final double angleOffsetComp;
    public final double angleOffsetPractice;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffsetPractice
     * @param angleOffsetComp
     */
    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, double angleOffsetPractice, double angleOffsetComp) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffsetPractice = angleOffsetPractice;
        this.angleOffsetComp = angleOffsetComp;
        if(Constants.COMP_BOT){
            angleOffset = this.angleOffsetComp;
        }
        else{
            angleOffset = this.angleOffsetPractice;
        }
    }
}

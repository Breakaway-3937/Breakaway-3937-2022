package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private DriveTrain s_DriveTrain;
    private Joystick translationalController;
    private Joystick rotationalController;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    /**
     * Driver control
     */
    public TeleopSwerve(DriveTrain s_DriveTrain, Joystick controller, Joystick controller1, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_DriveTrain = s_DriveTrain;
        addRequirements(s_DriveTrain);

        this.translationalController = controller;
        this.rotationalController = controller1;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        double yAxis = translationalController.getRawAxis(strafeAxis);
        double xAxis = translationalController.getRawAxis(translationAxis);
        double rAxis = rotationalController.getRawAxis(rotationAxis);
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.Controllers.STICK_DEADBAND) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.Controllers.STICK_DEADBAND) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.Controllers.STICK_DEADBAND) ? 0 : rAxis;

        translation = new Translation2d(yAxis, xAxis).times(Constants.DriveTrain.MAX_SPEED);
        rotation = rAxis * Constants.DriveTrain.MAX_ANGULAR_VELOCITY;
        s_DriveTrain.drive(translation, rotation, fieldRelative, openLoop);
    }
}

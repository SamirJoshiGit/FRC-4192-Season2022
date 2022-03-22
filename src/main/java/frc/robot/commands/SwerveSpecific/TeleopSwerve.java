package frc.robot.commands.SwerveSpecific;

import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    private double rightTrigger;

    /**
     * Driver control
     */
    //constructor which sets up the controller, the axis, and the swerve
    public TeleopSwerve(Swerve s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, double rightTrigger, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.rightTrigger = rightTrigger;
    }

    @Override
    public void execute() {
        //double slowMultiplier = 1 - (rightTrigger.getAsDouble()*.5);

        //every scheduled loop it will get the raw data from controller 
        double yAxis = controller.getRawAxis(translationAxis) * .75;
        double xAxis = controller.getRawAxis(strafeAxis) * .75;
        double rAxis = controller.getRawAxis(rotationAxis) * .5;
        
        
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        //takes the translation2d and multiplies by the wanted speed
        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed).times(rightTrigger);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        //s_Swerve.drive(translation, rotation*rightTrigger, fieldRelative, openLoop);
        s_Swerve.drive(translation, rotation*.9, Globals.fieldBased, openLoop);
    }
}

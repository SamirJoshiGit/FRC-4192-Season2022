package frc.robot.commands.SwerveSpecific;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SwerveDoubleSupp extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;

    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier r;


    /**
     * Driver control
     */
    public SwerveDoubleSupp(Swerve s_Swerve, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.x = x;
        this.y = y;
        this.r =r;
    }

    @Override
    public void execute() {
        double yAxis = y.getAsDouble();
        double xAxis = x.getAsDouble();
        double rAxis = -r.getAsDouble();
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        xAxis *= .5;
        yAxis *= .5;
        rAxis *= .5;
        //plug values into swerve base drive command
        translation = new Translation2d(-yAxis, -xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}

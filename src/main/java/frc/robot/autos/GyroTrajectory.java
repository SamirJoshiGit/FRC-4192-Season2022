// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

//import javax.net.ssl.TrustManagerFactory;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GyroTrajectory extends PIDCommand {
  /** Creates a new GyroTrajectory. */
  private Swerve s_Swerve;
  private double distance;
  public GyroTrajectory(Swerve s_Swerve, double distance) {
    super(
        // The controller that the command will use
        new PIDController(1, 0, 0),
        // This should return the measurement
        () -> s_Swerve.getDistanceX(),
        // This should return the setpoint (can also be a constant)
        () -> distance,
        // This uses the output
        output -> {
          // Use the output here
          s_Swerve.drive(new Translation2d(output, 0), 0, true, true);
        });
        
        this.s_Swerve = s_Swerve;
        this.distance = distance;
        addRequirements(s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(s_Swerve.getDistanceX() - distance) < .1;
  }
}

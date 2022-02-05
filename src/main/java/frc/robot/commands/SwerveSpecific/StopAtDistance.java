// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveSpecific;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopAtDistance extends PIDCommand {
  /** Creates a new StopAtDistance. */
  public StopAtDistance(Swerve swerve, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(.5, 0, 0),
        // This should return the measurement
        () -> setpoint,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          swerve.drive(new Translation2d(output, 0), 0, false, true);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(swerve);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

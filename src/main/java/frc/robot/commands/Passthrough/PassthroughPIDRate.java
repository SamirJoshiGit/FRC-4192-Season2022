// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Passthrough;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Passthrough;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PassthroughPIDRate extends PIDCommand {
  /** Creates a new PassthroughPIDRate. */
  public PassthroughPIDRate(Passthrough passthrough, double setpoint) {
    super(
        // The controller that the command will use
        new PIDController(.2, 0, 0),//test for values
        // This should return the measurement
        () -> passthrough.getEncoderRate(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint,
        // This uses the output
        output -> {
          // Use the output here
          passthrough.runMotor(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

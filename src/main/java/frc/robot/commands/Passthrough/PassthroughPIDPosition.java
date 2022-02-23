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
public class PassthroughPIDPosition extends PIDCommand {
  /** Creates a new PassthroughPIDPosition. */
  public PassthroughPIDPosition(Passthrough passthrough, double setpoint, double initialPos) {
    super(
        // The controller that the command will use
        new PIDController(0.2, 0, 0),
        // This should return the measurement
        () -> passthrough.getInternalEncoder(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint + initialPos,
        // This uses the output
        output -> {
          // Use the output here
          passthrough.runMotor(output);
        });
    addRequirements(passthrough);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

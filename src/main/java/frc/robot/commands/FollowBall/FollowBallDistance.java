// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FollowBall;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Globals;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowBallDistance extends PIDCommand {
  /** Creates a new FollowBallDistance. */
  public FollowBallDistance(double m_stopPoint) {
    super(
        // The controller that the command will use
        new PIDController(1.2, 0, 0),
        // This should return the measurement
        () -> Globals.ballSize,
        // This should return the setpoint (can also be a constant)
        () -> m_stopPoint,
        // This uses the output
        output -> {
          if(Globals.ballSize != 0){
            Globals.movingOutput = output;
          }
          else Globals.movingOutput = 0;
          // Use the output here
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

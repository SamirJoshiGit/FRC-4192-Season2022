// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FollowBall;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Globals;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowBallAngle extends PIDCommand {
  /** Creates a new FollowBallAngle. */
  public FollowBallAngle(Swerve m_Swerve) {
    super(
        // The controller that the command will use
        new PIDController(.1, 0, 0),
        // This should return the measurement
        () -> Globals.ballCenterX,
        // This should return the setpoint (can also be a constant)
        () -> 315/2,
        // This uses the output
        output -> {
          // Use the output here
          //Globals.rotatingOutput = output;
          m_Swerve.drive(new Translation2d(0, 0), output, true, true);
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

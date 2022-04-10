// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EncoderBasedRun extends PIDCommand {
  /** Creates a new EncoderBasedRun. */
  public EncoderBasedRun(double setpointSpeed, Shooter shooter, boolean lowGoal) {
    super(
        // The controller that the command will use
        new PIDController(.0000625, 0.0000000, 0),
        // This should return the measurement
        () -> shooter.getMainRate(),
        // This should return the setpoint (can also be a constant)
        () -> setpointSpeed,
        // This uses the output
        output -> {
          // Use the output here
          if(!lowGoal){
            shooter.twoMotorPower(-output);
          }
          else{
            shooter.twoMotorPowerLow(-output);
          }
          
          
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

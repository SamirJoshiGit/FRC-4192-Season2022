// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.Climb;

public class StopLineDetect extends CommandBase {
  private Climb climb;
  private frc.robot.subsystems.Swerve swerve;
  /** Creates a new StopLineDetect. */
  public StopLineDetect(Climb climb, frc.robot.subsystems.Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climb = climb;
    this.swerve = swerve;
    addRequirements(climb, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //drive straight from the begining
    swerve.drive(new Translation2d(1, 0), 0, false, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //when the command is ended the robot will stop
    swerve.drive(new Translation2d(0,0), 0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if it finds the gaffe tape, then it ends the command
    return climb.getReflective();
  }
}

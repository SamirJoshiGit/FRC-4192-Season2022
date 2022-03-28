// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Passthrough;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Passthrough;

public class ShortRunTripped extends CommandBase {
  /** Creates a new ShortRunTripped. */
  private Passthrough passthrough;
  private double power;
  public ShortRunTripped(Passthrough passthrough, double power) {
    this.passthrough = passthrough;
    this.power = power;
    addRequirements(passthrough);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    passthrough.runMotor(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    passthrough.runMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !passthrough.getBeamBreak();
  }
}

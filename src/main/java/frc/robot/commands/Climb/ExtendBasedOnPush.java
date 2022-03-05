// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class ExtendBasedOnPush extends CommandBase {
  /** Creates a new ExtendBasedOnPush. */
  private DoubleSupplier powerOutput;
  private Climb climb;
  private double multiplier;
  public ExtendBasedOnPush(Climb climb, DoubleSupplier powerOutput, double multiplier) {
    this.climb = climb;
    this.powerOutput = powerOutput;
    this.multiplier = multiplier;
    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.extendClimb(powerOutput.getAsDouble()*multiplier);
    climb.extendClimbRight(powerOutput.getAsDouble()*multiplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.extendClimb(0);
    climb.extendClimbRight(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

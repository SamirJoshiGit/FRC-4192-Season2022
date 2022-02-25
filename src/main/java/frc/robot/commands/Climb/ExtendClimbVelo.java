// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class ExtendClimbVelo extends CommandBase {
  /** Creates a new ExtendClimbVelo. */
  private Climb climb;
  private double velo;
  public ExtendClimbVelo(Climb climb, double velo) {
    this.climb = climb;
    this.velo = velo;
    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.extendLeftVelo(velo);
    climb.extendRightVelo(velo);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.extendLeftVelo(0);
    climb.extendRightVelo(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

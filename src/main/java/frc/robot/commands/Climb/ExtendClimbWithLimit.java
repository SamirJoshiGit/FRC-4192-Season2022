// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Globals;
import frc.robot.subsystems.Climb;

public class ExtendClimbWithLimit extends CommandBase {
  /** Creates a new ExtendClimb. */
  private Climb climb;
  private double velo;
  private boolean triggerStop;
  public ExtendClimbWithLimit(Climb climb, double velo) {
    this.climb = climb;
    this.velo = velo;
    triggerStop = false;
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
    climb.extendClimb(velo);
    climb.extendClimbRight(velo);
    if(Globals.climberStartPosition+10 < climb.getPlacement()){
      triggerStop = true;
    }
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
    return triggerStop;
  }
}

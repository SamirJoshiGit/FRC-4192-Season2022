// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class ChangeClimbAngle extends CommandBase {
  /** Creates a new ChangeClimbAngle. */
  private Climb climb;
  public ChangeClimbAngle(Climb climb) {
    this.climb = climb;
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
    if(!climb.getAngle()){
      climb.setAngleUp();
    }
    else{
      climb.setAngleDown();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //climb.setAngleDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

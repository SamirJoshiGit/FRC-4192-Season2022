// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climb;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbAngleInstant extends InstantCommand {
  private Climb climb;
  private boolean up;
  public ClimbAngleInstant(Climb climb, boolean up) {
    this.climb =climb;
    this.up = up;
    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //extend the climb if it is already down
    //retract if it is up
    if(climb.getAngle()){
      climb.setAngleDown();
    }
    else{
      climb.setAngleUp();
    }
  }
}

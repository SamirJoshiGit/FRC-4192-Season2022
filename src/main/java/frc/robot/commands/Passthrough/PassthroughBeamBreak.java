// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Passthrough;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Passthrough;

public class PassthroughBeamBreak extends CommandBase {
  /** Creates a new PassthroughBeamBreak. */
  private Passthrough passthrough; 
  public PassthroughBeamBreak(Passthrough passthrough){
    this.passthrough = passthrough;
    addRequirements(passthrough);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(passthrough.getBeamBreak()){
      passthrough.runMotor(.2);
    }
    else{
      passthrough.runMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

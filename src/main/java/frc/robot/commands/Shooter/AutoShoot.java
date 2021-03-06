// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Globals;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends CommandBase {
  /** Creates a new AutoShoot. */

  private Passthrough passthrough;
  private Timer timer;
  private double setpoint;
  public AutoShoot(Passthrough passthrough, double setpoint) {
    this.passthrough = passthrough;
    this.setpoint = setpoint;
    timer = new Timer();
    addRequirements(passthrough);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(Math.abs(Globals.topSpinRate) - setpoint) <= 300){
      passthrough.runMotor(.3);
    } 
    else{
      passthrough.runMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    passthrough.runMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

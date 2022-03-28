// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Globals;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.Shooter;

public class ShootTwo extends CommandBase {
  /** Creates a new AutoShoot. */

  private Passthrough passthrough;
  private double setpoint;
  private double countedBalls;
  private boolean alreadyRun;
  private boolean alreadyDropped;
  private Timer timer;
  public ShootTwo(Passthrough passthrough, double setpoint) {
    this.passthrough = passthrough;
    this.setpoint = setpoint;
    countedBalls = 0;
    alreadyRun = false; 
    alreadyDropped = false;
    timer = new Timer();
    addRequirements(passthrough);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    passthrough.runMotor(0);
    countedBalls = 0;
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((Math.abs(Math.abs(Globals.topSpinRate) - setpoint) <= 300) && timer.get() >= 1){
      passthrough.runMotor(.4);
      if(!alreadyRun){
        alreadyRun = true; 
      }
    } 
    else if(alreadyRun && Math.abs(Math.abs(Globals.topSpinRate) - setpoint) >= 300){
      alreadyDropped = true;
    }
    else{
      passthrough.runMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    countedBalls = 0;
    passthrough.runMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return countedBalls ==2;
  }
}

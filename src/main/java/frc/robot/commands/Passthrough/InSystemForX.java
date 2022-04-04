// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Passthrough;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Passthrough;

public class InSystemForX extends CommandBase {
  /** Creates a new RunningWithoutBreaks. */
  private Passthrough passthrough;
  private boolean brokenAtStart;
  private double power;
  private Intake intake;
  private Timer timer = new Timer();
  public InSystemForX(Passthrough passthrough, double power) {
    this.passthrough = passthrough;
    this.power = power;

    addRequirements(passthrough);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    if(!passthrough.getBeamBreak()){
      brokenAtStart = true;
    }
    passthrough.runMotor(power);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    passthrough.runMotor(0);
    
    brokenAtStart = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!passthrough.getBeamBreak() && !brokenAtStart) || ((brokenAtStart && timer.get() > .75)&&!passthrough.getBeamBreak());
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Passthrough;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Globals;
import frc.robot.subsystems.Passthrough;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class runMotor extends CommandBase {
  /** Creates a new runMotor. */
  private Passthrough passthrough;
  private double output;

  public runMotor(Passthrough passthrough, double output) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.passthrough = passthrough;
    this.output = output;
    addRequirements(passthrough);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

    //SmartDashboard.putBoolean("Motor running", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Globals.indexBeam = passthrough.getBeamBreak();
    passthrough.runMotor(output);
    //SmartDashboard.putBoolean("Motor running", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    passthrough.runMotor(0);
    //SmartDashboard.putBoolean("Motor running", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

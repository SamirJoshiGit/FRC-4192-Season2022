// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Passthrough;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Passthrough;

public class RunUntilTripped extends CommandBase {
  /** Creates a new RunUntilTripped. */
  private Intake intake;
  private Passthrough passthrough;
  private double power;
  public RunUntilTripped(Intake intake, Passthrough passthrough, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.passthrough = passthrough;
    this.power = power;
    addRequirements(intake, passthrough);
  }
  //RunUntilTripped(m_intake, m_passthrough)
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.getBeamBreak()){
      passthrough.runMotor(power);
    }
    if(passthrough.getBeamBreak()){
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

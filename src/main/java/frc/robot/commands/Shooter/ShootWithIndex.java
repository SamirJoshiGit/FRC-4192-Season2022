// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.Shooter;

public class ShootWithIndex extends CommandBase {
  /** Creates a new ShootWithIndex. */
  private Shooter shooter;
  private Passthrough passthrough;
  private double shooterVelo;
  private double passthroughVelo;
  public ShootWithIndex(Shooter shooter, Passthrough passthrough, double shooterVelo, double passthroughVelo) {
    this.passthrough = passthrough;
    this.shooter = shooter;
    this.shooterVelo = shooterVelo;
    this.passthroughVelo = passthroughVelo;
    addRequirements(passthrough, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.velocityBasedControl(shooterVelo);
    passthrough.setVelocity(passthroughVelo);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.velocityBasedControl(0);
    passthrough.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

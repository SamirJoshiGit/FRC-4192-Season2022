// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Passthrough;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Globals;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.Shooter;

public class RunUntilTripped extends CommandBase {
  /** Creates a new RunUntilTripped. */
  private Intake intake;
  private Passthrough passthrough;
  private double power;
  private double counter;
  private Shooter shooter;
  private boolean intakeTripped;
  public RunUntilTripped(Intake intake, Passthrough passthrough, Shooter shooter, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.passthrough = passthrough;
    this.power = power;
    this.shooter = shooter;
    counter = 0;
    intakeTripped = !intake.debounceBeam();
    addRequirements(intake, passthrough, shooter);
  }
  //RunUntilTripped(m_intake, m_passthrough)
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    passthrough.runMotor(0);
    SmartDashboard.putBoolean("Motor Running", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeTripped = !intake.debounceBeam();
    if(Globals.countedIndex <= 2){
      passthrough.runMotor(.3);
      SmartDashboard.putBoolean("Motor Running", true);
    }
    if(Globals.countedIndex == Globals.countedSecond){
      passthrough.runMotor(0);
      SmartDashboard.putBoolean("Motor Running", false);
    }
    if((Globals.countedIndex == Globals.countedSecond)&&(Globals.countedSecond >= 2)){
      Globals.countedIndex = 0;
      Globals.countedSecond = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    passthrough.runMotor(0);
    counter = 0; 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

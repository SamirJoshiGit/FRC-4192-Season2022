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
  public RunUntilTripped(Intake intake, Passthrough passthrough, Shooter shooter, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.passthrough = passthrough;
    this.power = power;
    this.shooter = shooter;
    counter = 0;
    addRequirements(intake, passthrough, shooter);
  }
  //RunUntilTripped(m_intake, m_passthrough)
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Globals.countedIndex==0 || Globals.countedIndex == 1){
      if(!passthrough.getBeamBreak()){
        SmartDashboard.putBoolean("Motor Running", true);
      }
      SmartDashboard.putBoolean("Motor Running", false);
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

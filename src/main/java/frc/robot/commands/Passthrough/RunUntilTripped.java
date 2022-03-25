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
  private Passthrough passthrough;
  private double counter;
  private boolean intakeTripped;
  private double power; 
  public RunUntilTripped(Passthrough passthrough, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.passthrough = passthrough;
    counter = 0;
    this.power = power;
    intakeTripped = !Globals.intakeBeam;
    addRequirements(passthrough);
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
    //intakeTripped = !Globals.intakeBeam;
    if(Globals.countedIntake <= 1 && (Globals.countedSecond == 0)){
      passthrough.runMotor(power);
      SmartDashboard.putBoolean("Motor Running", true);
    }

    else if((Globals.countedIntake == Globals.countedSecond)||(Globals.countedIntake <= 2)){
      passthrough.runMotor(0);
      SmartDashboard.putBoolean("Motor Running", false);
    }

    if((Globals.countedIntake == Globals.countedSecond)&&(Globals.countedSecond >= 2)){
      Globals.countedIntake = 0;
      Globals.countedSecond = 0;
    }
    intakeTripped = !Globals.intakeBeam;
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

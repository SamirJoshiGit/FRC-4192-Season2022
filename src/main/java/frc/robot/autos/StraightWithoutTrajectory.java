// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.Wait;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Passthrough.runMotor;
import frc.robot.commands.SwerveSpecific.moveWithManualInput;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StraightWithoutTrajectory extends ParallelRaceGroup {
  /** Creates a new StraightWithoutTrajectory. */
  public StraightWithoutTrajectory(Intake m_intake, Shooter m_shooter, Passthrough m_passthrough, Swerve s_Swerve, double wait, double direction) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunIntake(m_intake, .4), new runMotor(m_passthrough, .3), new moveWithManualInput(s_Swerve, 0, -2*direction, 0), new Wait(wait));
  }
}

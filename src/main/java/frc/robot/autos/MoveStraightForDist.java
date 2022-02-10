// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

//import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
//import frc.robot.Constants.Swerve;
import frc.robot.commands.Wait;
import frc.robot.commands.SwerveSpecific.moveWithManualInput;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveStraightForDist extends ParallelRaceGroup {
  /** Creates a new MoveStraightForDist. */
  public MoveStraightForDist(frc.robot.subsystems.Swerve s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new moveWithManualInput(s_Swerve, 3, .8, -.2), new Wait(1.9));
  }
}

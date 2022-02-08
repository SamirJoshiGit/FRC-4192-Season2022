// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Wait;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopExtend extends SequentialCommandGroup {
  /** Creates a new StopExtend. */
  public StopExtend(Swerve swerve, Climb climb) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    /*
      Stop at benchmark line

      1. main climb goes up
      2. main climb retracts
      3. passive hook pistons activate
      4. main climb angle moves back main climb extends
      
      5.once main climb is on, the passive hooks retract
      repeat 
    */
    addCommands(new StopLineDetect(climb, swerve)
    ,new ParallelRaceGroup(new Wait(3), new ExtendClimb(climb, .5))
    , new ParallelRaceGroup(new Wait(3), new ExtendClimb(climb, -.5))
    );
  }
}

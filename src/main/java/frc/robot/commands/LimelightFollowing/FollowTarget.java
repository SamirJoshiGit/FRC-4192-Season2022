// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LimelightFollowing;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Globals;
import frc.robot.commands.SwerveSpecific.moveWithManualInput;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowTarget extends ParallelCommandGroup {
  /** Creates a new FollowTarget. */
  public FollowTarget(Swerve m_Swerve, Limelight m_Limelight, boolean finishAtEnd, double m_stopPoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new LimelightFollowToPoint(m_Swerve, m_Limelight, finishAtEnd, m_stopPoint, true), 
      new LimelightFollower(m_Swerve, m_Limelight, finishAtEnd, true), 
      new moveWithManualInput(m_Swerve, Globals.rotatingOutput, Globals.movingOutput, 0));
  }
}
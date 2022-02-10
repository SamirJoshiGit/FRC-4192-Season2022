// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.List;

//import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Wait;
import frc.robot.commands.Intake.ChangeIntakeInstant;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.LimelightFollowing.LimelightFollowToPoint;
import frc.robot.subsystems.Intake;
//import frc.robot.commands.LimelightFollowing.LimelightFollower;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StraightBack extends SequentialCommandGroup {
  public StraightBack(Swerve s_Swerve, Limelight m_Limelight, Intake intake){
    TrajectoryConfig config =
        new TrajectoryConfig(
                1.0,//max spped
                1.0)//max aceleration squared
            .setKinematics(Constants.Swerve.swerveKinematics);

    // An example trajectory to follow.  All units in meters.


    Trajectory firstWaypoint = 
    TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, new Rotation2d(0)),
            //new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-90))) 
            new Pose2d(Units.feetToMeters(9.4), -Units.feetToMeters(4), new Rotation2d(0))
            //,new Pose2d(1.7, -1.3, new Rotation2d(0))
            //new Pose2d(-Units.feetToMeters(5)-AutoConstants.kOffset, -AutoConstants.kOffsetSide, new Rotation2d(Units.degreesToRadians(-90)))
            ), 
        config);

    Trajectory goToMid = TrajectoryGenerator.generateTrajectory(
    List.of(new Pose2d(0, 0, new Rotation2d(0)), 
    new Pose2d(Units.feetToMeters(9.4), -Units.feetToMeters(2), new Rotation2d(0))
    //,new Pose2d(0,0, new Rotation2d(0))
    ),    
    config);

    Trajectory goBackToScore = TrajectoryGenerator.generateTrajectory(
        List.of(new Pose2d(0, 0, new Rotation2d(0)), 
        new Pose2d(-Units.feetToMeters(9.4), Units.feetToMeters(.5), new Rotation2d(0)))
    
    , config);


    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand1 =
        new SwerveControllerCommand(
            firstWaypoint,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

            SwerveControllerCommand swerveControllerCommand2 =
            new SwerveControllerCommand(
                goBackToScore,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

    SwerveControllerCommand swerveControllerCommand3 =
        new SwerveControllerCommand(
            goToMid,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);


    addCommands(
        new ChangeIntakeInstant(intake, true),
        new InstantCommand(() -> s_Swerve.resetOdometry(goToMid.getInitialPose())),
        //new LimelightFollower(s_Swerve, m_Limelight, true, false)
        new ParallelRaceGroup(swerveControllerCommand3, new RunIntake(intake, .2)),
        new InstantCommand(() ->  s_Swerve.drive(new Translation2d(0, 0), 0, true, true)), 
        new Wait(2),
        swerveControllerCommand2
        //,swerveControllerCommand3,
        /*, 
        new InstantCommand(() ->  s_Swerve.drive(new Translation2d(0, 0), 0, true, true)), 
        new Wait(2),
        new InstantCommand(() -> s_Swerve.resetOdometry(goBackToScore.getInitialPose())), swerveControllerCommand2,
        new InstantCommand(() ->  s_Swerve.drive(new Translation2d(0, 0), 0, true, true)), 
        new Wait(1)*/

    );
}
}
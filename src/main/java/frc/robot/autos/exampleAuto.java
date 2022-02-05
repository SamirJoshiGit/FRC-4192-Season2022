package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Wait;
import frc.robot.commands.SwerveSpecific.TurnToSpecifiedAngle;
import frc.robot.subsystems.Swerve;

import java.util.List;

//import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    1,
                    1)
                    .setReversed(true)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(.5, new Rotation2d(Units.degreesToRadians(180)))),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-2, .1, new Rotation2d(Units.degreesToRadians(720))),
                config);

        //Trajectory driveWithRot = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(.5, new Rotation2d(Units.degreesToRadians(180)))), new Pose2d(2, .2, new Rotation2d(360), config)        
        Trajectory triangleLegOne = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))), 
                new Pose2d(-Units.feetToMeters(2), Units.feetToMeters(.1), new Rotation2d(Units.degreesToRadians(90)))),    
            config);

        Trajectory triangleLegTwo = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, new Rotation2d(0)), 
                new Pose2d(Units.feetToMeters(.1), Units.feetToMeters(2), new Rotation2d(0))),    
            config);

        Trajectory triangleLegThree = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, new Rotation2d(0)), 
                new Pose2d(Units.feetToMeters(2), -Units.feetToMeters(2), new Rotation2d(0))),    
            config);

        Trajectory waypointlist = 
            TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, new Rotation2d(0)),
                    //new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-90))) 
                    new Pose2d(-Units.feetToMeters(5)-AutoConstants.kOffset, -AutoConstants.kOffsetSide, new Rotation2d(0))
                    //new Pose2d(-Units.feetToMeters(5)-AutoConstants.kOffset, -AutoConstants.kOffsetSide, new Rotation2d(Units.degreesToRadians(-90)))
                    ), 
                config);
        

        Trajectory turnrightTrajectory = 
            TrajectoryGenerator.generateTrajectory(
                List.of(
                new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0))), 
                new Pose2d(-Units.feetToMeters(1.5), 0, new Rotation2d(0)),
                new Pose2d(-Units.feetToMeters(0.2), Units.feetToMeters(5.1)+AutoConstants.kOffset, new Rotation2d(0))), 
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        SwerveControllerCommand swerveControllerLeft =
            new SwerveControllerCommand(
                triangleLegTwo,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        SwerveControllerCommand swerveController3 =
            new SwerveControllerCommand(
                triangleLegThree,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);        

        

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand, 
            new InstantCommand(() ->  s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
            new Wait(3) 

           /* new InstantCommand(() -> s_Swerve.resetOdometry(triangleLegTwo.getInitialPose())),
            swerveControllerLeft,
            new InstantCommand(() ->  s_Swerve.drive(new Translation2d(0, 0), 0, true, true)),
            new Wait(3), 

            new InstantCommand(() -> s_Swerve.resetOdometry(triangleLegThree.getInitialPose())),
            swerveController3, 
            new InstantCommand(() ->  s_Swerve.drive(new Translation2d(0, 0), 0, true, true))*/
        );
    }
}
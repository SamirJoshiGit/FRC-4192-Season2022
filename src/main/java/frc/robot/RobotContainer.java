// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//import java.util.function.DoubleSupplier;

import frc.robot.autos.*;
//import frc.robot.commands.*;
import frc.robot.commands.Climb.ChangeClimbAngle;
import frc.robot.commands.Climb.StopExtend;
import frc.robot.commands.Climb.UpandDown;
import frc.robot.commands.Climb.ExtendClimb;
import frc.robot.commands.Climb.ExtendClimbRight;
import frc.robot.commands.Climb.MoveThreeBars;
import frc.robot.commands.Climb.ExtendClimbLeft;
import frc.robot.commands.FollowBall.FollowBallTogether;
//import frc.robot.commands.FollowBall.FollowBallAngle;
import frc.robot.commands.Intake.ChangeIntakePosition;
//import frc.robot.commands.Intake.IntakeVelocityControl;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.TestRunIntake;
import frc.robot.commands.LimelightFollowing.FollowTarget;
import frc.robot.commands.LimelightFollowing.LimelightFollowToPoint;
import frc.robot.commands.LimelightFollowing.LimelightFollower;
import frc.robot.commands.Passthrough.PassthroughBeamBreak;
import frc.robot.commands.Passthrough.RunUntilTripped;
import frc.robot.commands.Passthrough.runMotor;
import frc.robot.commands.Shooter.EncoderBasedRun;
import frc.robot.commands.Shooter.RunShooterMotor;
import frc.robot.commands.Shooter.ShootWithIndex;
import frc.robot.commands.Shooter.Velocity;
import frc.robot.commands.SwerveSpecific.StopAtDistance;
import frc.robot.commands.SwerveSpecific.SwerveDoubleSupp;
import frc.robot.commands.SwerveSpecific.TeleopSwerve;
//import frc.robot.commands.SwerveSpecific.TurnToSpecifiedAngle;
//import frc.robot.commands.SwerveSpecific.moveWithManualInput;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final XboxController xDrive = new XboxController(0);
  private final XboxController systemsController = new XboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton yButton = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton aButton = new JoystickButton(driver, XboxController.Button.kA.value);
  private final JoystickButton bButton = new JoystickButton(driver, Button.kB.value);
  private final JoystickButton rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  //driver dpad buttons
  private final POVButton zero = new POVButton(driver, 0);  
  private final POVButton ninety = new POVButton(driver, 90);
  private final POVButton oneEighty = new POVButton(driver, 180);
  private final POVButton twoSeventy = new POVButton(driver, 270);



  /*systems controller*/
  private final JoystickButton aButtonSystems = new JoystickButton(systemsController, XboxController.Button.kA.value);
  private final JoystickButton bButtonSystems = new JoystickButton(systemsController, XboxController.Button.kB.value);
  private final JoystickButton xButtonSystems = new JoystickButton(systemsController, XboxController.Button.kX.value);
  private final JoystickButton yButtonSystems = new JoystickButton(systemsController, XboxController.Button.kY.value);
  private final JoystickButton rightBumperSystems = new JoystickButton(systemsController, XboxController.Button.kRightBumper.value);
  private final JoystickButton leftBumperSystems = new JoystickButton(systemsController, XboxController.Button.kLeftBumper.value);
  //POV Buttons
  private final POVButton zeroSystems = new POVButton(systemsController, 0);
  private final POVButton oneEightySystems = new POVButton(systemsController, 180);
  private final POVButton twoSeventySystems = new POVButton(systemsController, 270);
  private final POVButton ninetySystems = new POVButton(systemsController, 90);

  //Triggers

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Limelight m_limelight = new Limelight();
  private final Passthrough m_passthrough = new Passthrough();
  private final Climb m_climb = new Climb();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  //commands
  private final SwerveDoubleSupp swerveControl = new SwerveDoubleSupp(s_Swerve, () -> xDrive.getLeftX(), () -> xDrive.getLeftY(), () -> xDrive.getRightX(), true, true);
  private final TeleopSwerve nonDoubSupp = new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, true, true);
  private final LimelightFollower follow = new LimelightFollower(s_Swerve, m_limelight, false, false);
  private final LimelightFollowToPoint followToPoint = new LimelightFollowToPoint(s_Swerve, m_limelight, false, 1, false);
  private final FollowTarget followTarget = new FollowTarget(s_Swerve, m_limelight, false, 1.1);
  private final FollowBallTogether followBall = new FollowBallTogether(s_Swerve);
  //private final TurnToSpecifiedAngle turn90 = new TurnToSpecifiedAngle(s_Swerve, s_Swerve.startAngle, 90, true);
  //private final TurnToSpecifiedAngle turn180 = new TurnToSpecifiedAngle(s_Swerve, s_Swerve.startAngle, -90, true);
  //private final TurnToSpecifiedAngle turn0 = new TurnToSpecifiedAngle(s_Swerve, s_Swerve.startAngle, 0, true);
  //private final TurnToSpecifiedAngle turn270 = new TurnToSpecifiedAngle(s_Swerve, s_Swerve.startAngle, 180, true);
  private final runMotor runPassthrough = new runMotor(m_passthrough, .2);
  private final runMotor runPassthroughTwo = new runMotor(m_passthrough, .4);

  private final StopExtend climbMacro = new StopExtend(s_Swerve, m_climb);
  private final RunIntake runIntake = new RunIntake(m_intake, .5);
  private final ChangeIntakePosition intakePos = new ChangeIntakePosition(m_intake);
  private final ChangeClimbAngle climbAngle = new ChangeClimbAngle(m_climb);
  private final StopAtDistance stopDist = new StopAtDistance(s_Swerve, Units.feetToMeters(5));
  private final PassthroughBeamBreak passthroughBeamBreak = new PassthroughBeamBreak(m_passthrough);
  private final ExtendClimb extend =  new ExtendClimb(m_climb, .5);
  private final ExtendClimb extendBack = new ExtendClimb(m_climb, -.5);

  private final ExtendClimbRight extendright = new ExtendClimbRight(m_climb, -.8);
  private final ExtendClimbRight extendrightback = new ExtendClimbRight(m_climb,.8);
  
  private final ExtendClimbLeft extendleft = new ExtendClimbLeft(m_climb, -.8);
  private final ExtendClimbLeft extendleftBack = new ExtendClimbLeft(m_climb, .8);

  //private final EncoderBasedRun encoderBasedRun = new EncoderBasedRun(500, m_shooter);

  //private final Velocity velocity = new Velocity(500, m_shooter);
  //private final IntakeVelocityControl intakeVelocityControl = new IntakeVelocityControl(500, m_shooter);
  private final TestRunIntake runForwardIntake = new TestRunIntake(0.6, m_intake);
  private final TestRunIntake runBackIntake = new TestRunIntake(-0.6, m_intake);

  private final UpandDown upAndDown = new UpandDown(m_climb, 720);
  private final MoveThreeBars mThreeBars = new MoveThreeBars(m_climb);

  private final runMotor runPassthroughForward = new runMotor(m_passthrough, .5);
  private final runMotor runPassthroughBackward = new runMotor(m_passthrough, -.5);

  //private final RunUntilTripped runUntilTripped = new RunUntilTripped(m_intake, m_passthrough, m_shooter, .2);
  private final RunShooterMotor runShooterMotor = new RunShooterMotor(m_shooter, .8);
  private final RunShooterMotor runShooterMotorBack = new RunShooterMotor(m_shooter, -.8);


  private final ShootWithIndex shootWithIndex = new ShootWithIndex(m_shooter, m_passthrough, 500, 500);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(nonDoubSupp);
    //m_passthrough.setDefaultCommand(runUntilTripped);
    //s_Swerve.setDefaultCommand(nonDoubSupp);
    //s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    leftBumper.whenActive(new InstantCommand(() -> s_Swerve.zeroGyro()));
    rightBumper.toggleWhenPressed(intakePos);
    
    //yButton.whileHeld(extend, false);
    //xButton.whileHeld(extendBack, false);
    yButton.toggleWhenPressed(runPassthroughForward);
    xButton.toggleWhenPressed(runPassthroughBackward);
    aButton.toggleWhenPressed(runForwardIntake);
    bButton.toggleWhenPressed(runBackIntake);
    //DPAD Buttons 
    //zero.whileHeld(extendleft);
    //ninety.whileHeld(extendleftBack);
    //oneEighty.whileHeld(extendright);
    //twoSeventy.whileHeld(extendrightback);
    
    zero.toggleWhenPressed(runShooterMotor);
    oneEighty.toggleWhenPressed(runShooterMotorBack);
    //yButton.whileHeld(extendright, false);
    //aButton.whenHeld(runPassthrough, true);
    //xButton.whileHeld(extendrightback, false);
    //zero.whenHeld(new moveWithManualInput(s_Swerve, 0, 1, 0), true);
    //ninety.whenPressed(turn90, true);
    //oneEighty.whenHeld(new moveWithManualInput(s_Swerve, 0, -1, 0), true);
    //twoSeventy.whenPressed(turn270, true);

    //Systems Buttons
    aButtonSystems.whenHeld(extend);
    bButtonSystems.whenHeld(extendBack);
    xButtonSystems.toggleWhenPressed(intakePos);
    yButtonSystems.toggleWhenPressed(climbAngle);

    rightBumperSystems.whenHeld(runShooterMotor);
    leftBumperSystems.whenHeld(runShooterMotorBack);
    //rightBumperSystems.whenHeld(runShooterMotor);
    //xButtonSystems.whenHeld(runForward, true);
    //yButtonSystems.whenHeld(runBack, true);
    //rightBumperSystems.whenHeld(stopDist, true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String[] autons = {"OneBall", "TwoBall"};
    SmartDashboard.putStringArray("Auto List", autons);
    String selected = SmartDashboard.getString("Auto Selector", "OneBall");
    // An ExampleCommand will run in autonomous
    //return new exampleAuto(s_Swerve);

    if(selected.equals("TwoBall")){
      //return new DoubleBallAuto(s_Swerve, m_limelight);
      return new ResetAndMove(s_Swerve, 1);
    }

    else if(selected.equals("OneBall")){
      return new StraightBack(s_Swerve, m_limelight, m_intake);
    }
    return new MoveStraightForDist(s_Swerve);
    //return new exampleAuto(s_Swerve);
    //return new LineWith180Flip(s_Swerve, m_limelight);
    //return new AccuracyTest(s_Swerve);
    //return new MoveStraightForDist(s_Swerve);



  }
}
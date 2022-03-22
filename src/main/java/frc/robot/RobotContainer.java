// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.XboxController.Axis;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//import java.util.function.DoubleSupplier;

import frc.robot.autos.*;
import frc.robot.commands.Wait;
import frc.robot.commands.Climb.AngleBoolean;
//import frc.robot.commands.*;
import frc.robot.commands.Climb.ChangeClimbAngle;
import frc.robot.commands.Climb.ClimbAngleInstant;
import frc.robot.commands.Climb.ExtendBasedOnPush;
import frc.robot.commands.Climb.StopExtend;
import frc.robot.commands.Climb.UpandDown;
import frc.robot.commands.Climb.ExtendClimb;
import frc.robot.commands.Climb.ExtendClimbRight;
import frc.robot.commands.Climb.ExtendClimbVelo;
import frc.robot.commands.Climb.MoveThreeBars;
import frc.robot.commands.Climb.PassiveHookActivate;
import frc.robot.commands.Climb.PassiveHookInstant;
import frc.robot.commands.Climb.ExtendClimbLeft;
import frc.robot.commands.FollowBall.FollowBallTogether;
import frc.robot.commands.Intake.ChangeIntakeInstant;
//import frc.robot.commands.FollowBall.FollowBallAngle;
import frc.robot.commands.Intake.ChangeIntakePosition;
//import frc.robot.commands.Intake.IntakeVelocityControl;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.TestRunIntake;
import frc.robot.commands.LimelightFollowing.FollowTarget;
import frc.robot.commands.LimelightFollowing.LimelightFollowToPoint;
import frc.robot.commands.LimelightFollowing.LimelightFollower;
import frc.robot.commands.Passthrough.PassthroughBeamBreak;
import frc.robot.commands.Passthrough.PassthroughPIDPosition;
import frc.robot.commands.Passthrough.RunUntilTripped;
import frc.robot.commands.Passthrough.runMotor;
import frc.robot.commands.Shooter.EncoderBasedRun;
import frc.robot.commands.Shooter.RunShooterMotor;
import frc.robot.commands.Shooter.ShootWithIndex;
import frc.robot.commands.Shooter.Velocity;
import frc.robot.commands.SwerveSpecific.StopAtDistance;
import frc.robot.commands.SwerveSpecific.SwerveDoubleSupp;
import frc.robot.commands.SwerveSpecific.TankToggle;
import frc.robot.commands.SwerveSpecific.TeleopSwerve;
import frc.robot.commands.SwerveSpecific.TurnToSpecifiedAngle;
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

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Limelight m_limelight = new Limelight();
  private final Passthrough m_passthrough = new Passthrough();
  private final Climb m_climb = new Climb();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();

  //compressor object
  //private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH );

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
  private final JoystickButton startButtonSystems = new JoystickButton(systemsController, XboxController.Button.kStart.value);
  private final JoystickButton optionsButtonSystems = new JoystickButton(systemsController, XboxController.Button.kBack.value);
  //POV Buttons
  private final POVButton zeroSystems = new POVButton(systemsController, 0);
  private final POVButton oneEightySystems = new POVButton(systemsController, 180);
  private final POVButton twoSeventySystems = new POVButton(systemsController, 270);
  private final POVButton ninetySystems = new POVButton(systemsController, 90);

  /*Triggers*/

  //driver hand triggers
  private final Trigger driverRightTrigger = new Trigger(()->driver.getRawAxis(XboxController.Axis.kRightTrigger.value)>0.2);
  private final Trigger driverLeftTrigger = new Trigger(()->driver.getRawAxis(XboxController.Axis.kLeftTrigger.value)>0.2);

  //systems hand 
  private final Trigger systemsRightTrigger = new Trigger(()->((systemsController.getRawAxis(XboxController.Axis.kRightTrigger.value))>0.2));
  private final Trigger systemsLeftTrigger = new Trigger(()->((systemsController.getRawAxis(XboxController.Axis.kLeftTrigger.value))>0.2));

  //sensor based triggers
  private Trigger secondTrigger = new Trigger(()->m_intake.debounceBeam());
  private Trigger intakeTrigger = new Trigger(()->m_intake.debounceBeam());
  private Trigger indexTrigger = new Trigger(()->m_passthrough.getBeamBreak());
  private Trigger shooterTrigger = new Trigger(()->m_shooter.getBeamBreak());
  private Trigger allBallsInIndex = new Trigger(()->Globals.countedIndex==2);
  private Trigger inTheSystem = new Trigger(()->Globals.countedIndex==1);
  private Trigger noneInTheSystem = new Trigger(()->Globals.countedIndex==0);

  //commands
  private final SwerveDoubleSupp swerveControl = new SwerveDoubleSupp(s_Swerve, () -> xDrive.getLeftX(), () -> xDrive.getLeftY(), () -> xDrive.getRightX(), true, true);
  private final TeleopSwerve nonDoubSupp = new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, 1, true, true);
  private final TeleopSwerve nonDoubSuppSlow = new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, .5, true, true);
  private final LimelightFollower follow = new LimelightFollower(s_Swerve, m_limelight, false, false);
  private final LimelightFollowToPoint followToPoint = new LimelightFollowToPoint(s_Swerve, m_limelight, false, 1, false);
  private final FollowTarget followTarget = new FollowTarget(s_Swerve, m_limelight, false, 1.1);
  private final FollowBallTogether followBall = new FollowBallTogether(s_Swerve);
  private final TurnToSpecifiedAngle turn90 = new TurnToSpecifiedAngle(s_Swerve, s_Swerve.startAngle, 90, true);
  private final TurnToSpecifiedAngle turn180 = new TurnToSpecifiedAngle(s_Swerve, s_Swerve.startAngle, -90, true);
  private final TurnToSpecifiedAngle turn0 = new TurnToSpecifiedAngle(s_Swerve, s_Swerve.startAngle, 0, true);
  private final TurnToSpecifiedAngle turn270 = new TurnToSpecifiedAngle(s_Swerve, s_Swerve.startAngle, 180, true);
  private final ExtendClimbVelo climbVelocityUp = new ExtendClimbVelo(m_climb, 200);
  private final ExtendClimbVelo climbVelocityDown = new ExtendClimbVelo(m_climb, -200);

  private final StopExtend climbMacro = new StopExtend(s_Swerve, m_climb);
  private final RunIntake runIntake = new RunIntake(m_intake, .5);
  private final ChangeIntakePosition intakePos = new ChangeIntakePosition(m_intake);
  private final ChangeIntakeInstant instantIntake = new ChangeIntakeInstant(m_intake, true);
  private final PassiveHookActivate togglePassiveHooks = new PassiveHookActivate(m_climb);
  private final ChangeClimbAngle climbAngle = new ChangeClimbAngle(m_climb);
  private final StopAtDistance stopDist = new StopAtDistance(s_Swerve, Units.feetToMeters(5));
  private final PassthroughBeamBreak passthroughBeamBreak = new PassthroughBeamBreak(m_passthrough);
  private final ExtendClimb extend =  new ExtendClimb(m_climb, .85);
  private final ExtendClimb extendBack = new ExtendClimb(m_climb, -.85);
  
  private final ExtendClimbVelo extendClimbVelo = new ExtendClimbVelo(m_climb, -500);
  private final ExtendClimbVelo detractClimbVelo = new ExtendClimbVelo(m_climb, 500);

  private final ExtendClimbRight extendright = new ExtendClimbRight(m_climb, -.8);
  private final ExtendClimbRight extendrightback = new ExtendClimbRight(m_climb,.8);
  
  private final ExtendClimbLeft extendleft = new ExtendClimbLeft(m_climb, -.8);
  private final ExtendClimbLeft extendleftBack = new ExtendClimbLeft(m_climb, .8);

  private ExtendBasedOnPush pushUp = new ExtendBasedOnPush(m_climb, ()->systemsController.getRawAxis(XboxController.Axis.kRightTrigger.value), .7);
  private ExtendBasedOnPush pushDown = new ExtendBasedOnPush(m_climb, ()->systemsController.getRawAxis(XboxController.Axis.kLeftTrigger.value), -.7);
  //private final EncoderBasedRun encoderBasedRun = new EncoderBasedRun(500, m_shooter);

  //private final Velocity velocity = new Velocity(500, m_shooter);
  //private final IntakeVelocityControl intakeVelocityControl = new IntakeVelocityControl(500, m_shooter);
  private final TestRunIntake runForwardIntake = new TestRunIntake(0.25, m_intake);
  private final TestRunIntake runBackIntake = new TestRunIntake(-0.9, m_intake);

  private final UpandDown upAndDown = new UpandDown(m_climb, 720);
  private final MoveThreeBars mThreeBars = new MoveThreeBars(m_climb);

  private final runMotor runPassthroughForward = new runMotor(m_passthrough, .5);
  private final runMotor runPassthroughBackward = new runMotor(m_passthrough, -.5);
  private final runMotor stopPassthrough = new runMotor(m_passthrough, 0);
  private final PassthroughPIDPosition positionIndexing = new PassthroughPIDPosition(m_passthrough, 2000, 0);

  //private final RunUntilTripped runUntilTripped = new RunUntilTripped(m_intake, m_passthrough, m_shooter, .2);
  private final RunShooterMotor runShooterMotor = new RunShooterMotor(m_shooter, .78);
  private final RunShooterMotor runShooterMotorBack = new RunShooterMotor(m_shooter, -.78);


  private final ShootWithIndex shootWithIndex = new ShootWithIndex(m_shooter, m_passthrough, 500, 500);
  private final RunUntilTripped runUntilTripped = new RunUntilTripped(m_intake
  , m_passthrough, m_shooter, .1);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.zeroGyro();
    s_Swerve.setDefaultCommand(nonDoubSupp);

    //compressor.enableHybrid(0, 100);
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

    //testing Bindings
    
    /* Driver Buttons */
    /*
    leftBumper.whenActive(new InstantCommand(() -> s_Swerve.zeroGyro()));
    rightBumper.toggleWhenPressed(intakePos);
    
    //yButton.whileHeld(extend, false);
    //xButton.whileHeld(extendBack, false);
    yButton.toggleWhenPressed(runPassthroughForward);
    //xButton.toggleWhenPressed(runPassthroughBackward);
    //aButton.toggleWhenPressed(runForwardIntake);
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
    */

    //Systems Buttons
    /*
    aButtonSystems.whenHeld(extend);
    bButtonSystems.toggleWhenPressed(togglePassiveHooks);
    xButtonSystems.toggleWhenPressed(intakePos);
    yButtonSystems.toggleWhenPressed(climbAngle);
    */

    //uncomment later
    //rightBumperSystems.whenHeld(runShooterMotor);
    //leftBumperSystems.whenHeld(runShooterMotorBack);
    //rightBumperSystems.whenHeld(runShooterMotor);
    //xButtonSystems.whenHeld(runForward, true);
    //yButtonSystems.whenHeld(runBack, true);
    //rightBumperSystems.whenHeld(stopDist, true);

    //triggered bindings change the interuptables during testing: uncomment once line breaks are on
    //intakeTrigger.and(noneInTheSystem.or(inTheSystem)).whileActiveOnce(runPassthroughForward);
    //indexTrigger.and(noneInTheSystem.or(inTheSystem).negate()).whileActiveOnce(stopPassthrough);
    //allBallsInIndex.whileActiveOnce(positionIndexing);
    //note for later, create an override system for this. 
    //intakeTrigger.whileActiveOnce(new InstantCommand(()->m_passthrough.changeBallCount()));
    //indexTrigger.whileActiveOnce(new InstantCommand(()->m_passthrough.changeIndexCount()));
    //secondTrigger.and((noneInTheSystem.or(inTheSystem)).and(indexTrigger)).whileActiveContinuous(runPassthroughForward, false);
    

    
    //prototyped Button Bindings
    leftBumper.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));
    yButton.toggleWhenPressed(intakePos);
    bButton.toggleWhenPressed(new TankToggle());
    //oneEighty.toggleWhenPressed(togglePassiveHooks);
    driverRightTrigger.whileActiveContinuous(runBackIntake);
    rightBumper.whenHeld(nonDoubSuppSlow);
    driverLeftTrigger.whileActiveContinuous(runForwardIntake);
    driverLeftTrigger.whileActiveContinuous(runPassthroughBackward);
    /*driverLeftTrigger.whileActiveOnce(new InstantCommand(()->{
      Globals.countedIndex -= 1;
      if(Globals.countedIndex <= 0){
        Globals.countedIndex = 0;
      }
    }));*/
    //bButton.toggleWhenActive(runShooterMotorBack);
    //yButton.toggleWhenPressed(runPassthroughForward);
    //aButton.whenPressed(runPassthroughBackward);
    //right bumper or trigger for intake
    //yButton.whenPressed(instantIntake);
    //b button for tank thing
    //systmes prototype bindings
    //zeroSystems.whenHeld(extend);
    //oneEightySystems.whenHeld(extendBack);
    //yButtonSystems.toggleWhenActive(climbAngle, false);
    //bButtonSystems.toggleWhenActive(togglePassiveHooks, false);
    //startButtonSystems.whenPressed(mThreeBars);
    aButtonSystems.toggleWhenPressed(runShooterMotorBack);
    
    //change later to the requirements
    leftBumperSystems.whenHeld(extendleftBack);
    rightBumperSystems.whenHeld(extendrightback);
    //systemsLeftTrigger.whileActiveContinuous(extendleftBack);
    //systemsRightTrigger.whileActiveContinuous(extendrightback);
    twoSeventySystems.whenHeld(extendleft);    
    ninetySystems.whenHeld(extendright);

    systemsLeftTrigger.whileActiveContinuous(pushDown);
    systemsRightTrigger.whileActiveContinuous(pushUp);


    //oneEighty.whenHeld(extendleftBack);
    //twoSeventy.whenHeld(extendrightback);

    xButtonSystems.whenHeld(runPassthroughForward); 
    
    //optionsButtonSystems.whenPressed(new ClimbAngleInstant(m_climb, true));
    //startButtonSystems.whenPressed(new ClimbAngleInstant(m_climb, false));

    optionsButtonSystems.whenPressed(new ClimbAngleInstant(m_climb, m_climb.getAngle()));
    yButtonSystems.whenPressed(new PassiveHookInstant(m_climb, m_climb.getHooks()));
    
    bButtonSystems.whenHeld(runBackIntake);
    //yButtonSystems.whenPressed(new PassiveHookInstant(m_climb, false));
    //bButtonSystems.whenPressed(new PassiveHookInstant(m_climb, true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String[] autons = {"OneBall", "TwoBall", "TwoNearWall", "OneNearWall"};
    double[] offset = {0, 33.33, 45, 60, 90};
    SmartDashboard.putStringArray("Auto List", autons);
    SmartDashboard.putNumberArray("Offset", offset);
    String selected = SmartDashboard.getString("Auto Selector", "OneBall");
    Double selectedOffset = SmartDashboard.getNumber("Offset", 33.3);
    // An ExampleCommand will run in autonomous
    //return new exampleAuto(s_Swerve);

    //if(selected.equals("TwoBall")){
      //return new DoubleBallAuto(s_Swerve, m_limelight, m_intake, m_passthrough, m_shooter);
      //return new ResetAndMove(s_Swerve, 1);
    //}

    //else if(selected.equals("OneBall")){
    if(selected.equals("TwoBall")){
        return new SequentialCommandGroup(
        new InstantCommand(()->s_Swerve.zeroGyro()),
        //new InstantCommand(()->s_Swerve.storeOffset()), 
        new AngleBoolean(m_climb, true), 
        new ChangeIntakeInstant(m_intake, false), 
        new ParallelRaceGroup(new StraightWithoutTrajectory(m_intake, m_shooter, m_passthrough, s_Swerve, 2.4,1.5)),
        new Wait(1),
        new ParallelRaceGroup(new StraightWithoutTrajectory(m_intake, m_shooter, m_passthrough, s_Swerve, 3,-1.5)),
        new Wait(1),
        new ParallelRaceGroup(new RunShooterMotor(m_shooter, -.75), new TestRunIntake( -.9, m_intake), new runMotor(m_passthrough, .3), new Wait(2)
        ,new InstantCommand(()->s_Swerve.setGyroOffset(selectedOffset))
        )
      ); 
    }

    else if(selected.equals("TwoNearWall")){
      return new SequentialCommandGroup(
        new InstantCommand(()->s_Swerve.zeroGyro()), 
        //new InstantCommand(()->s_Swerve.storeOffset()),
        new AngleBoolean(m_climb, true), 
        new ChangeIntakeInstant(m_intake, false), 
        new ParallelRaceGroup(new StraightWithoutTrajectory(m_intake, m_shooter, m_passthrough, s_Swerve, 1.9,1.5)),
        new Wait(1),
        new ParallelRaceGroup(new StraightWithoutTrajectory(m_intake, m_shooter, m_passthrough, s_Swerve, 2.5,-1.5)),
        new Wait(1),
        new ParallelRaceGroup(new RunShooterMotor(m_shooter, -.75), new TestRunIntake( -.9, m_intake), new runMotor(m_passthrough, .3), new Wait(2)
        ,new InstantCommand(()->s_Swerve.setGyroOffset(-selectedOffset))
        )
      ); 
    }

    else if(selected.equals("OneNearWall")){
      return new SequentialCommandGroup(
      new InstantCommand(()->s_Swerve.zeroGyro()),
      //new InstantCommand(()->s_Swerve.storeOffset()), 
      new AngleBoolean(m_climb, true), new Wait(.1), new AutonShootOut(m_shooter, m_passthrough, m_intake, 1), 
      new Wait(2), 
      new StraightWithoutTrajectory(m_intake, m_shooter, m_passthrough, s_Swerve, 3.7,1)
      ,new InstantCommand(()->s_Swerve.setGyroOffset(-selectedOffset))
      );
    }

    //go back (121.19)
    //turn (-157)
    //move forward (100)
    //
    else if(selected.equals("ThreeBall")){
      return new SequentialCommandGroup(
      new InstantCommand(()->s_Swerve.zeroGyro()),
      //new InstantCommand(()->s_Swerve.storeOffset()), 
      new AngleBoolean(m_climb, true), new Wait(.1), new AutonShootOut(m_shooter, m_passthrough, m_intake, 1), 
      new Wait(2), 
      new StraightWithoutTrajectory(m_intake, m_shooter, m_passthrough, s_Swerve, 3.7,1)
      ,new InstantCommand(()->s_Swerve.setGyroOffset(selectedOffset))
      );
    }


    else{
      return new SequentialCommandGroup(
      new InstantCommand(()->s_Swerve.zeroGyro()), 
      //new InstantCommand(()->s_Swerve.storeOffset()),
      new AngleBoolean(m_climb, true), 
      new Wait(.1), 
      new AutonShootOut(m_shooter, m_passthrough, m_intake, 1), 
      new Wait(2), 
      new StraightWithoutTrajectory(m_intake, m_shooter, m_passthrough, s_Swerve, 4,1)
      ,new InstantCommand(()->s_Swerve.setGyroOffset(selectedOffset))
      );
    }
      //return new SequentialCommandGroup(new InstantCommand(()->s_Swerve.zeroGyro()), new AngleBoolean(m_climb, true), new Wait(.1), new AutonShootOut(m_shooter, m_passthrough, m_intake, 1), new Wait(2), new StraightWithoutTrajectory(m_intake, m_shooter, m_passthrough, s_Swerve, 4,1));
      //return new StraightBack(s_Swerve, m_limelight, m_intake, m_passthrough, m_shooter);
    //}
    //return new MoveStraightForDist(s_Swerve);
    //return new exampleAuto(s_Swerve);
    //return new LineWith180Flip(s_Swerve, m_limelight);
    //return new AccuracyTest(s_Swerve);
    //return new MoveStraightForDist(s_Swerve);



  }
}
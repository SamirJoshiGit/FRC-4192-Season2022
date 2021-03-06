// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.util.net.PortForwarder;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

//import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.vision.VisionThread;
//import edu.wpi.first.wpilibj.TimedRobot;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 240;

  private VisionThread visionThread;
  private double maxCenterX = 0.0;
  private double centerX = 0.0;
  private double area = 0.0;
  private CvSink sink;
  private CvSource source;
  private final Object imgLock = new Object();
  private Mat mat;
  NetworkTableEntry s_centerX, s_frameCnt, s_area;
  int frameCnt = 0;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ctreConfigs = new CTREConfigs();
    //RobotContainer.s_Swerve.zeroGyro();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    PortForwarder.add(5800, "10.41.92.11", 5800);

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5805, "limelight.local", 5805);
/*    UsbCamera camera = CameraServer.startAutomaticCapture(0);
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    
    //sink = CameraServer.getVideo();
    CvSink cvSink = CameraServer.getVideo();
    CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);

    Mat source = new Mat();
    Mat output = new Mat();
    //source = CameraServer.putVideo("CVStream", 600, 600);
    visionThread = new VisionThread(camera, new GetBlueBall(), pipeline -> {
      
      ArrayList<MatOfPoint> countourList = pipeline.filterContoursOutput();
      //sink.grabFrame(mat);
      //pipeline.process(mat);

      //source.putFrame(mat);
      cvSink.grabFrame(source);
      
      Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
      //outputStream.putFrame(countourList.get(0));
      if (!pipeline.filterContoursOutput().isEmpty()) {
          int numBalls = countourList.size();
          double maxArea =0;
          int maxIndex = 0;
          for (int i = 0; i < numBalls; i++) {
            Rect r = Imgproc.boundingRect(countourList.get(i));
            double a = r.area();
            if (a > maxArea) {
              maxArea = a;
              maxIndex = i;
            }
          }
          Rect a = Imgproc.boundingRect(pipeline.filterContoursOutput().get(maxIndex));
          synchronized (imgLock) {
              Globals.ballCenterX = a.x + (a.width / 2);
              Globals.ballCenterY = a.y + (a.height /2);
              Globals.ballSize =a.area();
          }
      }
    });
    visionThread.start();
    */
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}

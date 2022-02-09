// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private NetworkTable table;
  private NetworkTableEntry tx, ty, ta;
  
  private HttpCamera limelightFeed;
  /** Creates a new Limelight. */
  public Limelight() {
    //these commands post values to network tables
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    //opens a new http camera on the roborio
    limelightFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  //gives our offset from target
  public double limelightOffset() {
    return tx.getDouble(0.0);
  }

  //gives back our limelight area
  public double limelightArea() {
    return ta.getDouble(0.0);
  }

}

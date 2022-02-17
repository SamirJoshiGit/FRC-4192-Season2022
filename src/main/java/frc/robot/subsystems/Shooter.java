// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX shooterMotor = new TalonFX(ShooterConstants.shooterMotor); 

  private CANCoder encoder = new CANCoder(ShooterConstants.shooterEncoderID);
  private final DigitalInput beam = new DigitalInput(ShooterConstants.beamBreakShooterID);
  public Shooter() {
  }

  public void setPower(double power){
    shooterMotor.set(ControlMode.PercentOutput, power);
  }

  public void velocityBasedControl(double velo){
    shooterMotor.set(ControlMode.Velocity, velo);
  }

  public boolean getBeamBreak(){
    return beam.get();
  }

  public double getRate(){
    return encoder.getVelocity();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
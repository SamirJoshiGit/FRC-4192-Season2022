// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX shooterMotor = new TalonFX(ShooterConstants.shooterMotorID); 
  private TalonFX shooterFollower = new TalonFX(ShooterConstants.shooterFollowerID);
  private TalonFX shooterMain = new TalonFX(ShooterConstants.shooterFollowerID);
  
  private CANCoder encoder = new CANCoder(ShooterConstants.shooterEncoderID);
  private final DigitalInput beam = new DigitalInput(ShooterConstants.beamBreakShooterID);
  private Debouncer beamDebouncer = new Debouncer(.1);
  public Shooter() {
    //shooterFollower.follow(shooterMotor);
    shooterFollower.setInverted(true);
  }

  public void setPower(double power){
    if(power != 0){
      SmartDashboard.putBoolean("DB/LED 0", true);
    }
    else{
      SmartDashboard.putBoolean("DB/LED 0", false);
    }
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

  public double getMainRate(){
    return shooterMotor.getSelectedSensorVelocity();
  }

  public double getFollowerRate(){
    return shooterFollower.getSelectedSensorVelocity();
  }

  public void twoMotorVelocity(double velo){
    shooterMotor.set(ControlMode.Velocity, velo);
    shooterFollower.set(ControlMode.Velocity, -velo);
  }

  public void twoMotorPower(double power){
    shooterMotor.set(ControlMode.PercentOutput, -power);
    shooterFollower.set(ControlMode.PercentOutput, power);
  }

  public void twoMotorCurrent(double curr){
    shooterMotor.set(ControlMode.Current, curr);
    shooterFollower.set(ControlMode.Current, curr);
  }
  public boolean debounceBeam(){
    return beamDebouncer.calculate(beam.get());
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Main motor rate", getMainRate());
    SmartDashboard.putNumber("FollowerRate", getFollowerRate());
    // This method will be called once per scheduler run
  }
}

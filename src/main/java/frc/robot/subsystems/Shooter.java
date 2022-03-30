// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Globals;
import frc.robot.Constants.TurretPIDConstants;
import frc.robot.Constants.Swerve.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX shooterMotor = new TalonFX(ShooterConstants.shooterMotorID); 
  private TalonFX shooterFollower = new TalonFX(ShooterConstants.shooterFollowerID);

  private CANCoder encoder = new CANCoder(ShooterConstants.shooterEncoderID);
  private final DigitalInput beam = new DigitalInput(ShooterConstants.beamBreakShooterID);
  private Debouncer beamDebouncer = new Debouncer(.001);

  private TalonFXConfiguration mainConfig = new TalonFXConfiguration();
  private TalonFXConfiguration followerConfig = new TalonFXConfiguration();

  public Shooter() {
    //shooterMotor config hardcode (put in CTRE configs later)
    shooterMotor.configFactoryDefault();
    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    shooterMotor.config_kP(0, TurretPIDConstants.topShooterkP);
    shooterMotor.config_kI(0, TurretPIDConstants.topShooterkI);
    shooterMotor.config_kD(0, TurretPIDConstants.topShooterkD);
    shooterMotor.config_kF(0, TurretPIDConstants.topShooterkF);
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.setSensorPhase(TurretPIDConstants.topSensorPhase);

    //shooterFollower config hardcode (put in CTRE configs later)
    shooterFollower.configFactoryDefault();
    shooterFollower.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
    shooterFollower.config_kP(0, TurretPIDConstants.bottomShooterkP);
    shooterFollower.config_kI(0, TurretPIDConstants.bottomShooterkI);
    shooterFollower.config_kD(0, TurretPIDConstants.bottomShooterkD);
    shooterFollower.config_kF(0, TurretPIDConstants.bottomShooterkF);
    shooterFollower.setNeutralMode(NeutralMode.Coast);
    shooterFollower.setSensorPhase(TurretPIDConstants.bottomSensorPhase);
    shooterFollower.setInverted(true);

    mainConfig.peakOutputForward = .7;
    mainConfig.peakOutputForward = -.7;
    
    followerConfig.peakOutputForward = .7;
    followerConfig.peakOutputReverse = -.7;

    //shooterMotor.configAllSettings(defualt)
    //shooterFollower.configAllSettings(followerConfig);
    //shooterMotor.configAllSettings(mainConfig);
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
    shooterMotor.set(ControlMode.Velocity, -velo*1);
    shooterFollower.set(ControlMode.Velocity, velo*1.4);
  }

  public void twoMotorPower(double power){
    //top
    shooterMotor.set(ControlMode.PercentOutput, -power*1);
    
    //bottom
    shooterFollower.set(ControlMode.PercentOutput, power*1.4);
  }

  public void mainPower(double power){
    shooterMotor.set(ControlMode.PercentOutput, power);
  }

  public void followerPower(double power){
    shooterFollower.set(ControlMode.PercentOutput, power);
  }

  public void twoMotorCurrent(double curr){
    shooterMotor.set(ControlMode.Current, curr*1.2);
    shooterFollower.set(ControlMode.Current, curr);
  }

  public boolean debounceBeam(){
    return beamDebouncer.calculate(beam.get());
  }

  public void changeGlobalRate(){
    Globals.topSpinRate = getMainRate();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Main motor rate", getMainRate());
    SmartDashboard.putNumber("FollowerRate", getFollowerRate());
    SmartDashboard.putBoolean("DB/LED 2", (Math.abs(Math.abs(getRate())-7000) <= 500));
    SmartDashboard.putBoolean("At Setpoint", (Math.abs(Math.abs(getRate())-6000) <= 500));

    changeGlobalRate();
    // This method will be called once per scheduler run
  }
}

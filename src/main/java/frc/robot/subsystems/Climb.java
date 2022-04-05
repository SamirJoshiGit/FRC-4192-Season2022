// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.SpeedController;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Globals;
import frc.robot.Constants.Swerve.ClimbConstants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */

  //private final DoubleSolenoid angleClimb = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimbConstants.doubleSolenoidForwardAngle,ClimbConstants.doubleSolenoidReverseAngle);//placeholders
  private final DoubleSolenoid angleClimb = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimbConstants.doubleSolenoidForwardAngle, ClimbConstants.doubleSolenoidReverseAngle);//placeholders

  private final DoubleSolenoid climber = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClimbConstants.doubleSolenoidForwardClimb, ClimbConstants.doubleSolenoidReverseClimb);
  private DigitalInput retroreflective = new DigitalInput(ClimbConstants.retroreflectiveID);
  private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  private TalonFX climbMotorLeft = new TalonFX(ClimbConstants.climbMotorLeft);
  private TalonFX climbMotorRight = new TalonFX(ClimbConstants.climbMotorRight);
  
  //private SpeedControllerGroup climbMotors = new SpeedControllerGroup(climbMotorLeft, climbMotorRight);
  public Climb() {
    //climber.set(Value.kReverse);
    //angleClimb.set(Value.kReverse);
    climbMotorRight.setInverted(true);
    climbMotorLeft.setNeutralMode(NeutralMode.Brake);
    climbMotorRight.setNeutralMode(NeutralMode.Brake);
    Globals.climberStartPosition = getRightEncoder();
    angleClimb.set(Value.kForward);
    //climbMotorRight.follow(climbMotorLeft);
  }

  //sets the angle of the climb above the robot lower
  public void setAngleDown(){
    angleClimb.set(Value.kReverse);
  }

  //sets the angle of the climb above the robot to 90
  public void setAngleUp(){
    angleClimb.set(Value.kForward);
  }

  //secondary hooks activated
  public void setClimberUp(){
    climber.set(Value.kForward);
  }

  //secondary hooks de-activated
  public void setClimberDown(){
    climber.set(Value.kReverse);
  }

  //returns when gaffe tape is visible
  public boolean getReflective(){
    return retroreflective.get();
  }

  //set the climb and follower through power
  public void extendClimb(double velo){
    climbMotorLeft.set(ControlMode.PercentOutput, velo);
  }

  //sets only the right climb 
  public void extendClimbRight(double power){
    climbMotorRight.set(ControlMode.PercentOutput, power);
  }

  public void extendLeftVelo(double velocity){
    climbMotorLeft.set(ControlMode.Velocity, velocity);
  }

  public void extendRightVelo(double velocity){
    climbMotorRight.set(ControlMode.Velocity, velocity);
  }
  
  public void resetInternalEncoder(){
    climbMotorLeft.setSelectedSensorPosition(0);
  }
  //gets the placement of the climb, in rotation

  //set through motion magic
  public void setMotionMagic(double setpoint){
    climbMotorRight.follow(climbMotorLeft);
    climbMotorLeft.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, 0.1);
  }

  public boolean getAtHeightLimit(double setpoint){
    return getLeftEncoder() >= setpoint; 
  }

  public double getRightEncoder(){
    return climbMotorRight.getSelectedSensorPosition();
  }
  
  public double getLeftEncoder(){
    return climbMotorLeft.getSelectedSensorPosition();
  }

  public void bangBangControl(){
    //climbMotorLeft.set(ControlMode.bangBangControl, .5);
  }

  public boolean getAngle(){
    return angleClimb.get().equals(DoubleSolenoid.Value.kForward);
  }

  public boolean getHooks(){
    return climber.get().equals(DoubleSolenoid.Value.kForward);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //put smart dashboard numbers
    SmartDashboard.putNumber("Right Climb Position", getRightEncoder());
    SmartDashboard.putNumber("Left Climb Position", getLeftEncoder());
    if(DriverStation.getMatchTime() == 30){
      compressor.disable();
    }
  }
}

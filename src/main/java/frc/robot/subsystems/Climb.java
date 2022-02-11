// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
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

  private TalonFX climbMotorLeft = new TalonFX(ClimbConstants.climbMotorLeft);
  private TalonFX climbMotorRight = new TalonFX(ClimbConstants.climbMotorRight);
  
  //private SpeedControllerGroup climbMotors = new SpeedControllerGroup(climbMotorLeft, climbMotorRight);
  public Climb() {
    //climber.set(Value.kReverse);
    //angleClimb.set(Value.kReverse);
    climbMotorRight.setInverted(true);
    Globals.climberStartPosition = getPlacement();
    //climbMotorRight.follow(climbMotorLeft);
  }

  public void setAngleDown(){
    angleClimb.set(Value.kReverse);
  }

  public void setAngleUp(){
    angleClimb.set(Value.kForward);
  }

  public void setClimberUp(){
    climber.set(Value.kForward);
  }

  public void setClimberDown(){
    climber.set(Value.kReverse);
  }

  public boolean getReflective(){
    return retroreflective.get();
  }

  public void extendClimb(double velo){
    climbMotorLeft.set(ControlMode.PercentOutput, velo);
  }

  public void extendClimbRight(double power){
    climbMotorRight.set(ControlMode.PercentOutput, power);
  }

  public double getPlacement(){
    return climbMotorLeft.getSelectedSensorPosition();
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climb Position", getPlacement());
  }
}

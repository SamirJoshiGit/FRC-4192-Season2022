// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
//import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CTREConfigs;
import frc.robot.Globals;
import frc.robot.Constants.Swerve.IntakeConstants;

public class Intake extends SubsystemBase {

  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.doubleSolenoidForward, IntakeConstants.doubleSolenoidReverse);//filler values for plug in locations
  //private final CANSparkMax intakeMotors;
  private final TalonFX intakeMotor = new TalonFX(IntakeConstants.intakeMotorID);
  private final DigitalInput beamBreaker = new DigitalInput(IntakeConstants.beamBreakIntakeID);
  private final Debouncer beamDebouncer = new Debouncer(.1);
  /** Creates a new Intake. */
  public Intake() {
    //intakeMotors = new CANSparkMax(1, MotorType.kBrushed);
    intakeMotor.configFactoryDefault();
    intakeMotor.configAllSettings(CTREConfigs.intakeFXConfig);
    intakeSolenoid.set(Value.kReverse);
  }

  //sets down the intake by retracting the piston
  public void setDown(){
    intakeSolenoid.set(Value.kReverse);
  }

  //sets up the intake by extending the intake 
  public void setUp(){
    intakeSolenoid.set(Value.kForward);
  }

    //sets power to the intake motor based on percent output 
  public void setPower(double power){
    //intakeMotors.set(power);
    intakeMotor.set(ControlMode.PercentOutput, power);
  }

  //digital out outputname.get();

  public boolean getBeamBreak(){
      return beamBreaker.get();
  }
  
  public double getCountedBalls(){
    
    if(getBeamBreak()){
      Globals.countedIndex++;
      if(Globals.countedIndex == 3){
        Globals.countedIndex = 1;
      }
    }
    return Globals.countedIndex;
  }

  public boolean debounceBeam(){
    return beamDebouncer.calculate(beamBreaker.get());
  }
  //moves intake motors based on velocity 
  public void velocityBasedControl(double velocity){
    intakeMotor.set(ControlMode.Velocity, velocity);
  }
  
  public boolean getIntake(){
    return intakeSolenoid.get().equals(DoubleSolenoid.Value.kForward);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Balls_In_System", Globals.countedIndex);
    SmartDashboard.putBoolean("Intake_Beam_Broken", getBeamBreak());
    // This method will be called once per scheduler run
  }
}

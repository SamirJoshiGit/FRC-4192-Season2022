// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Globals;
import frc.robot.Constants.Swerve.PassthroughConstants;
import frc.robot.Constants.Swerve.Sensors;

public class Passthrough extends SubsystemBase {
  /** Creates a new Passthrough. */
  private final TalonFX passthroughMotor = new TalonFX(PassthroughConstants.passthroughMotorID);

  private final CANCoder encoder = new CANCoder(PassthroughConstants.passthroughEncoderID);

  private Debouncer breakDebounce = new Debouncer(.1);
  private DigitalInput beamBreak = new DigitalInput(Sensors.beamBreakPassthroughID); 
  public Passthrough() {
    
  }

  //runs motor of conveyor belt using percent power
  public void runMotor(double power){
    passthroughMotor.set(ControlMode.PercentOutput, power);
  }

  public void setVelocity(double velo){
    passthroughMotor.set(ControlMode.Velocity, velo);
  }

  //gets the value of the beam break
  public boolean getBeamBreak(){
    return !beamBreak.get();
  }

  public double getCountedBalls(){
    if(!getBeamBreak()){
      Globals.countedIndex++;
      if(Globals.countedIndex == 3){
        Globals.countedIndex = 1;
      }
    }
    return Globals.countedIndex;
  }

  //gets the velocity of the motor
  public double getEncoderRate(){
    return encoder.getVelocity();
  }

  public double getInternalEncoder(){
    return passthroughMotor.getSelectedSensorPosition();
  }
  public void magicMotion(){
    passthroughMotor.set(TalonFXControlMode.MotionMagic, .2);
  }

  public boolean debouncerGet(){
    return breakDebounce.calculate(beamBreak.get());
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Passthrough Encoder Rate", getInternalEncoder());
    SmartDashboard.putBoolean("line broken", getBeamBreak());
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve.PassthroughConstants;
import frc.robot.Constants.Swerve.Sensors;

public class Passthrough extends SubsystemBase {
  /** Creates a new Passthrough. */
  private final TalonFX passthroughMotor = new TalonFX(20);

  private final CANCoder encoder = new CANCoder(PassthroughConstants.passthroughEncoderID);

  private final DigitalOutput beamBreak = new DigitalOutput(Sensors.beamBreakPassthroughID); 
  public Passthrough() {
    
  }

  //runs motor of conveyor belt using percent power
  public void runMotor(double power){
    passthroughMotor.set(ControlMode.PercentOutput, power);
  }

  //gets the value of the beam break
  public boolean getBeamBreak(){
    return beamBreak.get();
  }

  //gets the velocity of the motor
  public double getEncoderRate(){
    return encoder.getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Passthrough Encoder Rate", getEncoderRate());
    // This method will be called once per scheduler run
  }
}

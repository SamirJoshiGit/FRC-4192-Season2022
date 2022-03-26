package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants.Swerve.IntakeConstants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public static TalonFXConfiguration intakeFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;
    public static TalonFXConfiguration topShooterFXConfig;
    public static TalonFXConfiguration bottomShooterFXConfig;


    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        intakeFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        topShooterFXConfig = new TalonFXConfiguration();
        bottomShooterFXConfig = new TalonFXConfiguration();
        

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.angleEnableCurrentLimit, 
            Constants.Swerve.angleContinuousCurrentLimit, 
            Constants.Swerve.anglePeakCurrentLimit, 
            Constants.Swerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;



        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.driveEnableCurrentLimit, 
            Constants.Swerve.driveContinuousCurrentLimit, 
            Constants.Swerve.drivePeakCurrentLimit, 
            Constants.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;

        SupplyCurrentLimitConfiguration intakeSupplyLimit = new SupplyCurrentLimitConfiguration(
            IntakeConstants.intakeEnableCurrentLimit,
            IntakeConstants.intakeContinuousCurrentLimit,
            IntakeConstants.intakePeakCurrentLimit,
            IntakeConstants.intakePeakCurrentDuration
        );
        intakeFXConfig.supplyCurrLimit = intakeSupplyLimit;
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        /*TopShooter Config put values in constants later*/
        topShooterFXConfig.slot0.kP = 0.1;
        topShooterFXConfig.slot0.kI = 0.00025;  
        topShooterFXConfig.slot0.kD = 0.0;
        topShooterFXConfig.slot0.kF = 0.0345;
        topShooterFXConfig.slot0.maxIntegralAccumulator = .99;  
        
        /*BottomShooter Config put values in constants later*/
        bottomShooterFXConfig.slot0.kP = 0.1;
        bottomShooterFXConfig.slot0.kI = 0.00025;  
        bottomShooterFXConfig.slot0.kD = 0.0;
        bottomShooterFXConfig.slot0.kF = 0.0345; 
        bottomShooterFXConfig.slot0.maxIntegralAccumulator = .99;
    }

}
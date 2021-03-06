package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.commands.Passthrough.runMotor;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        //public static final int pigeonID = 20;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(28);
        public static final double wheelBase = Units.inchesToMeters(28);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0); //6.86:1  //6.75 //6.5 (seems to be the ratio with the least amount of clicks)
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1 //12.78

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit =  15;//25
        public static final int anglePeakCurrentLimit = 30;//40
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;//35
        public static final int drivePeakCurrentLimit = 65;//65
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;


        
        /* Angle Motor PID Values */
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 13;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        
        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        public static final class Sensors {
            //sensor ID constant
            public static final int firstPortUltrasonic = 7;
            public static final int secondPortUltrasonic = 8;

            public static final int beamBreakPassthroughID = 3;//placeholders
        }

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 14;//7
            public static final int angleMotorID = 15;//8
            public static final int canCoderID = 3;//12
            public static final double angleOffset = 29;//70.4 //25
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final double angleOffset = 193.0;//8.4 //189
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 2;
            public static final double angleOffset = 363.0;//213.7 //359
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 0;
            public static final int canCoderID = 0;
            public static final double angleOffset = 300;//238.5 //295
            public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static final class PassthroughConstants {
            public static final int passthroughMotorID = 8;

            public static final int passthroughEncoderID = 0;
        }

        public static final class IntakeConstants {
            //filler values
            public static final int intakeMotorID = 7;
            //public static final int doubleSolenoidIntake = 0;
            public static final int doubleSolenoidReverse = 11;
            public static final int doubleSolenoidForward = 4;
            //sensors
            public static final int beamBreakIntakeID = 1;//temporary

            //intake current limiting 
            public static final int intakeContinuousCurrentLimit = 15;
            public static final int intakePeakCurrentLimit = 30;
            public static final double intakePeakCurrentDuration = .1;
            public static final boolean intakeEnableCurrentLimit = false;
        }

        public static final class ClimbConstants{
            //piston filler values
            //public static final int doubleSolenoidClimb = 0;
            //public static final int doubleSolenoidAngle = 0;

            public static final int doubleSolenoidReverseClimb = 10;
            public static final int doubleSolenoidForwardClimb = 5;

            public static final int doubleSolenoidReverseAngle = 9;
            public static final int doubleSolenoidForwardAngle = 6;
            //sensor (filler values)

            public static final int retroreflectiveID = 9;

            //motors
            public static final int climbMotorLeft = 5;
            public static final int climbMotorRight = 6;

        }

        public static final class ShooterConstants{
            public static final int shooterMotorID = 9;
            public static final int shooterFollowerID = 18;
            public static final int shooterMainID = 19;

            public static final int beamBreakShooterID = 4;//temporary
            
        
            public static final int shooterEncoderID = 1;//temporary
        }
    }

    public static final class TurretPIDConstants{
        public static double topShooterkP = .002;
        public static double topShooterkI = 0;
        public static double topShooterkD = 0;
        public static double topShooterkF = .0345;
        public static boolean topSensorPhase = true; 

        public static double bottomShooterkP = .002;
        public static double bottomShooterkI = 0;
        public static double bottomShooterkD = 0;
        public static double bottomShooterkF = .0345;
        public static boolean bottomSensorPhase = true; 
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        public static final double kOffset = Units.inchesToMeters(9);
        public static final double kOffsetSide = Units.inchesToMeters(9); 
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

}

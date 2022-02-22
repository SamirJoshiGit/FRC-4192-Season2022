package frc.robot.subsystems;

//import com.analog.adis16448.frc.ADIS16448_IMU;
//import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.Globals;
import frc.robot.Constants.Swerve.Sensors;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Ultrasonic;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public static AHRS gyro;
    public static double startAngle;

    private Ultrasonic ultrasonic;
    //private static final ADIS16448_IMU imu = new ADIS16448_IMU();

    public Swerve() {
        gyro = new AHRS(SPI.Port.kMXP);
        //gyro.configFactoryDefault();
        zeroGyro();
        resetDistance();
        startAngle = getDoubleYaw();
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(true));
        ultrasonic = new Ultrasonic(Sensors.firstPortUltrasonic, Sensors.secondPortUltrasonic);
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        //SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    //overloaded
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean hasInvert) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw(hasInvert)
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        //SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        //SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    
    //returns the pose
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }
    //resets the odemetry for auton
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    //returns all the swerve module states
    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }
    //recents the gyro yaw
    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public double getDistFromUltra(){
        return ultrasonic.getRangeInches();
    }
    public Rotation2d getYaw() {
        //double[] ypr = new double[3];
        //ypr[0] = gyro.getYaw();
        //ypr[1] = gyro.getPitch();
        //ypr[2] = gyro.getRoll();
        double yaw = gyro.getYaw();
        //return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
        return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
        //return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
    }

    public Rotation2d getYaw(boolean hasInvert){
        double yaw = gyro.getYaw();
        if(hasInvert){
            return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw); 
        }
        return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
    }
    
    public double getDoubleYaw(){
        return gyro.getYaw();
    }
    //x displacment from the navx gyro
    public double getDistanceX(){
        return gyro.getDisplacementX();
    }
    //gives us y displacement from navx
    public double getDistanceY(){
        return gyro.getDisplacementY();
    }

    public void resetDistance(){
        gyro.resetDisplacement();
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(true), getStates());  
        SmartDashboard.putNumber("Y displacement", getDistanceY());
        SmartDashboard.putNumber("X displacement", getDistanceX());
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Gyro", getDoubleYaw());
            SmartDashboard.putNumber("BallCenterX", Globals.ballCenterX);
            SmartDashboard.putNumber("BallArea", Globals.ballSize);
            SmartDashboard.putNumber("BallCenterY", Globals.ballCenterY);
        }
    }
}
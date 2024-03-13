package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    //Crofts
    private SwerveModulePosition[] arrSMP = {frontLeft.getSMP(), frontRight.getSMP(), backLeft.getSMP(), backRight.getSMP()};
    
    
    private SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), arrSMP);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    // TODO: Be able to set angle offset based on starting position
    public void zeroHeading() {
        gyro.reset();
        //gyro.setAngleAdjustment(getHeading());
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public double getGyroVelocity()
    {
        return Math.sqrt(Math.pow(gyro.getVelocityX(),2) + Math.pow(gyro.getVelocityY(),2));
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
        //return Rotation2d.fromRadians(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), arrSMP,pose);
    }

    /*public void resetOdometry(Pose2d pose) {
        Rotation2d[] moduleAngles = new Rotation2d[] {
            frontLeft.getSMP().angle,
            frontRight.getSMP().angle,
            backLeft.getSMP().angle,
            backRight.getSMP().angle
        };
    
        /*SwerveModuleState[] states = new SwerveModuleState[] {
            new SwerveModuleState(frontLeft.getDriveVelocity(), moduleAngles[0]),
            new SwerveModuleState(frontRight.getDriveVelocity(), moduleAngles[1]),
            new SwerveModuleState(backLeft.getDriveVelocity(), moduleAngles[2]),
            new SwerveModuleState(backRight.getDriveVelocity(), moduleAngles[3])
        };

        SwerveModulePosition[] temp = {
            new SwerveModulePosition(frontLeft.getDrivePosition(), moduleAngles[0]),
            new SwerveModulePosition(frontRight.getDrivePosition(), moduleAngles[1]),
            new SwerveModulePosition(backLeft.getDrivePosition(), moduleAngles[2]),
            new SwerveModulePosition(backRight.getDrivePosition(), moduleAngles[3])
        };
    
        odometer.resetPosition(getRotation2d(), temp, pose);
    }*/
    

    //@Override
    /*public void periodic() {
        odometer.update(getRotation2d(), arrSMP);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        frontLeft.updateSwerveModulePosition();
        frontRight.updateSwerveModulePosition();
        backLeft.updateSwerveModulePosition();
        backRight.updateSwerveModulePosition();
    }*/
    @Override
    public void periodic() {
        arrSMP[0] = frontLeft.updateSwerveModulePosition(); 
        arrSMP[1]= frontRight.updateSwerveModulePosition();
        arrSMP[2] = backLeft.updateSwerveModulePosition();
        arrSMP[3] = backRight.updateSwerveModulePosition();  
        //arrSMP = new SwerveModulePosition[]{frontLeft.getSMP(), frontRight.getSMP(), backLeft.getSMP(), backRight.getSMP()};   
        odometer.update(getRotation2d(), arrSMP);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }
    

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /*public void setModuleStatesInit(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredStateInit(desiredStates[0]);
        frontRight.setDesiredStateInit(desiredStates[1]);
        backLeft.setDesiredStateInit(desiredStates[2]);
        backRight.setDesiredStateInit(desiredStates[3]);
    }*/

    public SwerveModule getFrontLeft()
    {
        return frontLeft;
    }
    public SwerveModule getBackLeft()
    {
        return backLeft;
    }
    public SwerveModule getBackRight()
    {
        return backRight;
    }
    public SwerveModule getFrontRight()
    {
        return frontRight;
    }
}
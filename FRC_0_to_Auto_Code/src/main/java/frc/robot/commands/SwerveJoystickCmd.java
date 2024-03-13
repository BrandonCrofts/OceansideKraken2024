package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
//import edu.wpi.first.math.geometry.*;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
            /*SwerveModuleState[] originalModuleStates = {new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0))};
            swerveSubsystem.setModuleStates(originalModuleStates);*/
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 3b. Quiero cocinar
        /*double vNorth, vEast, theta;
        vNorth = xSpeed;
        vEast = ySpeed;
        theta = swerveSubsystem.getHeading() * (Math.PI/180.0);
        theta += swerveSubsystem.getGyroVelocity()*(Math.PI/180.0) * 0.25;
        xSpeed = Math.cos(theta)*vNorth + Math.sin(theta)*vEast;
        ySpeed = -Math.sin(theta)*vNorth + Math.cos(theta)*vEast;*/
        
        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        //Removed for testing on 1/24/24
        /*if (fieldOrientedFunction.get()) {
            // Relative to field*/
            //chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            //        xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        //} else {
            
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        //}

        //chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        // Crofts Edit 1/23/24
        // Goal: eliminate setModuleStates and use something involving SwerveModulePosition objs
        /*Rotation2d[] angles = {
            moduleStates[0].angle,
            moduleStates[1].angle,
            moduleStates[2].angle,
            moduleStates[3].angle
        };*/
        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
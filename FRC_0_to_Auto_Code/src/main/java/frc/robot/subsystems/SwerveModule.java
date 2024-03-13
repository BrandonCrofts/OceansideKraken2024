package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
//import edu.wpi.first.wpilibj.AnalogEncoder;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;
    // Made public on 1/24/24 for testing
    public final AnalogInput absoluteEncoder;
    //private final AnalogEncoder absoluteEncoder2;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    //Crofts
    private SwerveModulePosition swerModPos;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        swerModPos = new SwerveModulePosition();
        
        resetEncoders();
        //System.out.println("SwerveModule is ON");
    }

    //Crofts
    public SwerveModulePosition getSMP()
    {
        return swerModPos;
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity(); //.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity(); //.getVelocity();
    }
    
    public double getPositionOfAbsEncoder()
    {
        return absoluteEncoder.getVoltage()/RobotController.getVoltage5V() * 2 * Math.PI;
    }


    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
        //setDesiredState(new SwerveModuleState(0, new Rotation2d(getTurningPosition())));
    }

    public SwerveModuleState getState() {
        //swerModPos = new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
        //Line below worked decently well
        //return new SwerveModuleState(getDriveVelocity(), swerModPos.angle);
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }


    public SwerveModulePosition updateSwerveModulePosition() {
        // Get the current angle from the turning encoder
        Rotation2d currentAngle = new Rotation2d(getTurningPosition());
    
        // Get the current distance from the drive encoder
        double currentDistance = getDrivePosition();
    
        // Update the SwerveModulePosition object
        swerModPos = new SwerveModulePosition(currentDistance, currentAngle);
        return swerModPos;
    }

    public void setDesiredState(SwerveModuleState state) {        
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        // 1/29/24: The other line assigning the value of state used to be what we used; I'm 
        // curious if the one we are now using will work better.
        state = SwerveModuleState.optimize(state, getState().angle); //GOOD
        //state = SwerveModuleState.optimize(state, new Rotation2d(getTurningPosition()));
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians())); //used to be getTurningPosition()*Math.PI*2*(1/21.5)
        //We added 0.293 as a factor = gear ratio*2pi
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }
    
    /*public void setDesiredStateInit(SwerveModuleState state) { 
        if(Math.abs(getTurningPosition() -state.angle.getRotations()) > 0.01)
        {
            turningMotor.set(turningPidController.calculate(getTurningPosition()*Math.PI*2*(1/21.5), state.angle.getRadians()));
        }     
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }*/

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public CANSparkMax getDriveMotor()
    {
        return driveMotor;
    }
}
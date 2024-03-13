package frc.robot.commands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateDegreesCmd extends Command
{
    private SwerveSubsystem swerveSub;
    private double degreesTurned; //Put positive degrees for turning right, negative for left
    private static final double secondsPerDegree = 0.1; //"0.1" IS PLACEHOLDER, CALCULATE THIS
    private Timer timer;
    private SwerveModuleState[] states;

    public RotateDegreesCmd(SwerveSubsystem swerveSub, double degreesTurned)
    {
        this.swerveSub = swerveSub;
        this.degreesTurned = degreesTurned;
        timer = new Timer();
    }

    @Override
    public void initialize()
    {
        timer.reset();
        timer.start();
        if(degreesTurned > 0)
        {
            states = new SwerveModuleState[]{new SwerveModuleState(1, new Rotation2d(swerveSub.getFrontLeft().getAbsoluteEncoderRad() + Math.PI/-4)), 
            new SwerveModuleState(-1, new Rotation2d(swerveSub.getFrontRight().getAbsoluteEncoderRad() + Math.PI/4)), 
            new SwerveModuleState(1, new Rotation2d(swerveSub.getBackLeft().getAbsoluteEncoderRad() + Math.PI/4)), 
            new SwerveModuleState(-1, new Rotation2d(swerveSub.getBackRight().getAbsoluteEncoderRad() + Math.PI/-4))};
        }
        else
        {
            states = new SwerveModuleState[]{new SwerveModuleState(-1, new Rotation2d(swerveSub.getFrontLeft().getAbsoluteEncoderRad() + Math.PI/-4)), 
            new SwerveModuleState(1, new Rotation2d(swerveSub.getFrontRight().getAbsoluteEncoderRad() + Math.PI/4)), 
            new SwerveModuleState(-1, new Rotation2d(swerveSub.getBackLeft().getAbsoluteEncoderRad() + Math.PI/4)), 
            new SwerveModuleState(1, new Rotation2d(swerveSub.getBackRight().getAbsoluteEncoderRad() + Math.PI/-4))};
        }
    }

    @Override
    public void execute()
    {
        swerveSub.setModuleStates(states);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(Math.abs(degreesTurned)*secondsPerDegree);
    }

    @Override
    public void end(boolean interrupted)
    {
        swerveSub.stopModules();
        timer.stop();
    }
}

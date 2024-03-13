package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class DriveBackwardCmd extends Command
{
    private final SwerveSubsystem swerveSub;
    private double timeActive;
    Timer timer = new Timer();
    private SwerveModuleState[] states;

    public DriveBackwardCmd(SwerveSubsystem swerveSub, double timeActive)
    {
        this.swerveSub = swerveSub;
        this.timeActive = timeActive;
        addRequirements(swerveSub);
    }
    @Override
    public void initialize()
    { 
        timer.reset();
        timer.start();
        states = new SwerveModuleState[]{new SwerveModuleState(-1, new Rotation2d(swerveSub.getFrontLeft().getAbsoluteEncoderRad())), 
                                        new SwerveModuleState(-1, new Rotation2d(swerveSub.getFrontRight().getAbsoluteEncoderRad())), 
                                        new SwerveModuleState(-1, new Rotation2d(swerveSub.getBackLeft().getAbsoluteEncoderRad())), 
                                        new SwerveModuleState(-1, new Rotation2d(swerveSub.getBackRight().getAbsoluteEncoderRad()))};
        //start the motors

    }
    @Override
    public void execute()
    {
        swerveSub.setModuleStates(states);
    }
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(timeActive); 
    }
    @Override
    public void end(boolean interrupted)
    {
        swerveSub.stopModules();
        timer.stop();
    }
}

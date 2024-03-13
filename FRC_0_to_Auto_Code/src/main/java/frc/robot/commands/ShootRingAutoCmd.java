package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonEncoderSubsystem;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootRingAutoCmd  extends Command
{
    private TalonEncoderSubsystem talonEncSub;
    Timer timer;
    private double timeElapsed = 4;

    public ShootRingAutoCmd(TalonEncoderSubsystem tes)
    {
        talonEncSub = tes;
        addRequirements(tes);
    }
    @Override
    public void initialize()
    {
        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute()
    {
        if(!timer.hasElapsed(timeElapsed/2))
        {
            talonEncSub.startOuttakeMotors();
        }
        else if(!timer.hasElapsed(timeElapsed))
        {
            talonEncSub.startOuttakeMotors();
            talonEncSub.startIntakeMotor();
        }
        else
        {
            talonEncSub.stopAllMotors();
        }
    }

    @Override
    public boolean isFinished()
    {
        return timer.hasElapsed(timeElapsed);
    }

    @Override
    public void end(boolean interrupted)
    {
        timer.stop();
    }
}

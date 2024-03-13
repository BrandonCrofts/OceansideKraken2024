package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonEncoderSubsystem;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DumpRingAutoCmd extends Command
{
    private TalonEncoderSubsystem talonEncSub;
    Timer timer;

    public DumpRingAutoCmd(TalonEncoderSubsystem tes)
    {
        talonEncSub = tes;
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
        if(!timer.hasElapsed(2))
        {
            talonEncSub.activateDownwardOuttake();
        }
        else if(!timer.hasElapsed(4))
        {
            talonEncSub.activateDownwardOuttake();
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
        return false;
    }
}

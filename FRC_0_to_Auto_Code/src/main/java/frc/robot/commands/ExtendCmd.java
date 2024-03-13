package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PneumaticSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ExtendCmd extends Command{
    private final PneumaticSubsystem pneumaticSubsystem;
    private final Timer timer = new Timer();
    private final double durationSeconds = 3; 

    public ExtendCmd(PneumaticSubsystem subsystem) {
        pneumaticSubsystem = subsystem;
        addRequirements(pneumaticSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute()
    {
        pneumaticSubsystem.extend();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSeconds); 
    }

    @Override
    public void end(boolean interrupted)
    {
        timer.stop();
    }

}
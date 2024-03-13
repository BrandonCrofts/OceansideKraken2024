package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PneumaticSubsystem;

import frc.robot.commands.RetractCmd;
public class RetractCmd extends Command{
    private final PneumaticSubsystem pneumaticSubsystem;

    public RetractCmd(PneumaticSubsystem subsystem) {
        pneumaticSubsystem = subsystem;
        addRequirements(pneumaticSubsystem);
    }

    @Override
    public void execute()
    {
        pneumaticSubsystem.retract();
    }
}
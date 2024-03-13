package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonEncoderSubsystem;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ReverseIntakeManCmd extends Command
{
    private TalonEncoderSubsystem talonEncSub;

    public ReverseIntakeManCmd(TalonEncoderSubsystem tes)
    {
        talonEncSub = tes;
    }

    @Override
    public void execute()
    {
        talonEncSub.startReverseIntakeMotor();
    }
}

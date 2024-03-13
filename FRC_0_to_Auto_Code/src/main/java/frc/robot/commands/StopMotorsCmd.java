package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonEncoderSubsystem;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StopMotorsCmd extends Command
{
    private TalonEncoderSubsystem talonEncSub;

    public StopMotorsCmd(TalonEncoderSubsystem tes)
    {
        talonEncSub = tes;
    }

    @Override
    public void execute()
    {
        talonEncSub.stopAllMotors();
    }
}

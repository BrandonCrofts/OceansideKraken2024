package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TalonEncoderSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class AutonomousCommand2 extends SequentialCommandGroup
{
    private final SwerveSubsystem swerveSub;
    private final TalonEncoderSubsystem tes;
    private final PneumaticSubsystem pSub;
    public AutonomousCommand2(SwerveSubsystem ss, TalonEncoderSubsystem ts, PneumaticSubsystem ps)
    {
        swerveSub = ss;
        tes = ts;
        pSub = ps;
        addCommands(new DriveForwardCmd(ss,0.2));
    }
}
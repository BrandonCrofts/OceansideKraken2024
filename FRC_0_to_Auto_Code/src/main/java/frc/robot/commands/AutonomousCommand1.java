package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TalonEncoderSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

public class AutonomousCommand1 extends SequentialCommandGroup
{
 //Ideal: Move x distance, turn x distance depnding on starting pos,
    private final SwerveSubsystem swerveSub;
    private final TalonEncoderSubsystem tes;
    private final PneumaticSubsystem pSub;
    public AutonomousCommand1(SwerveSubsystem ss, TalonEncoderSubsystem ts, PneumaticSubsystem ps)
    {
        swerveSub = ss;
        tes = ts;
        pSub = ps;
        addCommands(new ExtendCmd(ps), 
        new ShootRingAutoCmd(tes),
        new DriveForwardCmd(ss, 3),
        new IntakeAutoCmd(tes),
        new DriveBackwardCmd(ss, 2.5),
        new ShootRingAutoCmd(tes)
        );
        //addCommands(new RotateDegreesCmd(ss, 7200));
    }
}
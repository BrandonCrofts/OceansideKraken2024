package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IntakeAutoCmd;
import frc.robot.commands.ShootRingAutoCmd;
import frc.robot.commands.AutonomousCommand1;
import frc.robot.commands.AutonomousCommand2;
import frc.robot.commands.DriveForwardCmd;
import frc.robot.commands.DumpRingAutoCmd;
import frc.robot.commands.ExtendCmd;
import frc.robot.commands.ReverseIntakeManCmd;
import frc.robot.commands.StopMotorsCmd;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//For Talon SRX Motor Controller
//import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.TalonEncoderSubsystem;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.commands.RetractCmd;


public class RobotContainer {
        // Made public on 1/24/24 for testing
    private final PneumaticSubsystem pneumaticSubsystem = new PneumaticSubsystem();
    //private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    private SendableChooser<Command> chooser;

    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick shooterJoystick = new Joystick(OIConstants.kShooterControllerPort);

    private final TalonEncoderSubsystem talonEncoderSubsystem = new TalonEncoderSubsystem();

    
    public RobotContainer() {

            //Making the chooser
            chooser = new SendableChooser<>();

            // Add commands to the chooser
            chooser.setDefaultOption("Middle pos optimal circumstance", new AutonomousCommand1(swerveSubsystem, talonEncoderSubsystem, pneumaticSubsystem));
            chooser.addOption("Test Command", new AutonomousCommand2(swerveSubsystem, talonEncoderSubsystem, pneumaticSubsystem));
            chooser.addOption("Quiero cocinar", new AutonomousCommand2(swerveSubsystem, talonEncoderSubsystem, pneumaticSubsystem));
            // Add more options as needed
            
            // Put the chooser on the dashboard
            SmartDashboard.putData("Auto Chooser", chooser);

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx))); 

        configureButtonBindings();
    }

    private void configureButtonBindings() 
    {
        //NEW B-Button: press and hold for reverse intake
        Trigger reverseIntakeTrigger = new Trigger(() -> shooterJoystick.getRawButton(2));
        reverseIntakeTrigger.whileTrue(new ReverseIntakeManCmd(talonEncoderSubsystem)).whileFalse(new StopMotorsCmd(talonEncoderSubsystem));
        
        //New Y-Button: single press starts intake and will auto-stop when ring is detected;
        //Second press will forcibly stop
        Trigger intakeButtonAuto = new Trigger(() -> shooterJoystick.getRawButton(4));
        intakeButtonAuto.toggleOnTrue(new IntakeAutoCmd(talonEncoderSubsystem));

        //New A-Button: press and hold will rev up outtake, then feed with intake
        Trigger outtakeSequenceTrigger = new Trigger(() -> shooterJoystick.getRawButton(1));
        outtakeSequenceTrigger.whileTrue(new ShootRingAutoCmd(talonEncoderSubsystem)).whileFalse(new StopMotorsCmd(talonEncoderSubsystem));

        //New X-Button: press and hold will slightly rev up outtake, then feed with intake
        Trigger launcherButtonTrigger2 = new Trigger(() -> shooterJoystick.getRawButton(3));
        launcherButtonTrigger2.whileTrue(new DumpRingAutoCmd(talonEncoderSubsystem)).whileFalse(new StopMotorsCmd(talonEncoderSubsystem));

        //Pneumatics aim downward controls (retract piston) (active while button is pressed)
        Trigger retractPistonTrigger = new Trigger(() -> shooterJoystick.getRawButton(5));
        retractPistonTrigger.toggleOnTrue(new RetractCmd(pneumaticSubsystem));

        //Pneumatics aim upward controls (extend piston) (active while button is pressed)
        Trigger extendPistonTrigger = new Trigger(() -> shooterJoystick.getRawButton(6));
        extendPistonTrigger.toggleOnTrue(new ExtendCmd(pneumaticSubsystem));

        //JoystickButton pneumaticButton = new JoystickButton(driverJoystick, 6);
        // Bind extend command while the button is pressed
        //pneumaticButton.whileTrue(new ExtendCommand(pneumaticSubsystem))
        //               .whileFalse(new RetractCommand(pneumaticSubsystem));
    }
    
    

    

    

    //Crofts
    public void updateSmartDashboard()
    {
        SmartDashboard.putNumber("AA FLD Encoder Value: ", swerveSubsystem.getFrontLeft().getDrivePosition());
        SmartDashboard.putNumber("AA FLT Encoder Value: ", swerveSubsystem.getFrontLeft().getTurningPosition());
        SmartDashboard.putNumber("AA FRD Encoder Value: ", swerveSubsystem.getFrontRight().getDrivePosition());
        SmartDashboard.putNumber("AA FRT Encoder Value: ", swerveSubsystem.getFrontRight().getTurningPosition());
        SmartDashboard.putNumber("AA BLD Encoder Value: ", swerveSubsystem.getBackLeft().getDrivePosition());
        SmartDashboard.putNumber("AA BLT Encoder Value: ", swerveSubsystem.getBackLeft().getTurningPosition());
        SmartDashboard.putNumber("AA BRD Encoder Value: ", swerveSubsystem.getBackRight().getDrivePosition());
        SmartDashboard.putNumber("AA BRT Encoder Value: ", swerveSubsystem.getBackRight().getTurningPosition());
        SmartDashboard.putNumber("NavX Angle: ", swerveSubsystem.getHeading());
        SmartDashboard.putNumber("FLD Speed: ", swerveSubsystem.getFrontLeft().getDriveVelocity());
        SmartDashboard.putNumber("FRD Speed: ", swerveSubsystem.getFrontRight().getDriveVelocity());
        SmartDashboard.putNumber("BLD Speed: ", swerveSubsystem.getBackLeft().getDriveVelocity());
        SmartDashboard.putNumber("BRD Speed: ", swerveSubsystem.getBackRight().getDriveVelocity());
        
        //Display Absolute Encoder Positions
        SmartDashboard.putNumber("AA Front Left Absolute Encoder Pos", swerveSubsystem.getFrontLeft().getPositionOfAbsEncoder());
        SmartDashboard.putNumber("AA Front Right Absolute Encoder Pos", swerveSubsystem.getFrontRight().getPositionOfAbsEncoder());
        SmartDashboard.putNumber("AA Back Left Absolute Encoder Pos", swerveSubsystem.getBackLeft().getPositionOfAbsEncoder());
        SmartDashboard.putNumber("AA Back Right Absolute Encoder Pos", swerveSubsystem.getBackRight().getPositionOfAbsEncoder());
        // Log absolute encoder offsets
        double fla = swerveSubsystem.getFrontLeft().getAbsoluteEncoderRad();
        SmartDashboard.putNumber("Front Left Absolute Encoder:",fla);
        double fra = swerveSubsystem.getFrontRight().getAbsoluteEncoderRad();
        SmartDashboard.putNumber("Front Right Absolute Encoder:",fra);
        double bla = swerveSubsystem.getBackLeft().getAbsoluteEncoderRad();
        SmartDashboard.putNumber("Back Left Absolute Encoder:",bla);
        double bra = swerveSubsystem.getBackRight().getAbsoluteEncoderRad();
        SmartDashboard.putNumber("Back Right Absolute Encoder:",bra);

    }
    
    

    /*public Command getAutonomousCommand() 
    {
        return new AutonomousCommand1(swerveSubsystem, talonEncoderSubsystem, pneumaticSubsystem);
    }*/

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
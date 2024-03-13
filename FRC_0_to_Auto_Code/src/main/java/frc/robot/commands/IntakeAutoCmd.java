package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TalonEncoderSubsystem;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeAutoCmd extends Command
{

    private final TalonEncoderSubsystem talonEncSub;
    //private static boolean toggleOn = false;
    private boolean shouldEnd;

    private double lastTimeChecked = 0;
    private double colorCheckInterval = 0.1;
    private double currentRed, currentBlue, currentGreen = 0;

    public IntakeAutoCmd(TalonEncoderSubsystem tes)
    {
        talonEncSub = tes; 
    }

    @Override
    public void initialize()
    {
        if(Math.abs(talonEncSub.getIntakeMotor()) > 0.1)
        {
            shouldEnd = true;
        }
        else
        {
            shouldEnd = false;
        }
    }

    @Override
    public void execute()
    {
        //double currentTime = Timer.getFPGATimestamp();
        //if((currentTime - lastTimeChecked) > colorCheckInterval)
        //{
            currentRed = TalonEncoderSubsystem.getCurrentColor().red;
            currentGreen = TalonEncoderSubsystem.getCurrentColor().green;
            currentBlue = TalonEncoderSubsystem.getCurrentColor().blue;
        //}
        //Orig: 0.545, 0.365, 0.090
        if((Math.abs(currentRed-0.377) >= colorCheckInterval ||
        Math.abs(currentGreen-0.412) >= colorCheckInterval ||
        Math.abs(currentBlue-0.205) >= colorCheckInterval))
        {
            talonEncSub.startIntakeMotor();
        }
        else{
            talonEncSub.stopIntakeMotor();
            shouldEnd = true;
        }
        //lastTimeChecked = currentTime;
        
    }

    @Override
    public boolean isFinished()
    {
        return shouldEnd;
    }

    @Override
    public void end(boolean interrupted)
    {
        talonEncSub.stopAllMotors();
    }

    
}

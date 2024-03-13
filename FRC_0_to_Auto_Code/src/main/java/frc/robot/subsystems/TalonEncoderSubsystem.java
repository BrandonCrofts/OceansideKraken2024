package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeAutoCmd;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.Timer;

public class TalonEncoderSubsystem extends SubsystemBase {
    private PWMTalonSRX talon0, talon1, talon2;
    private static Color currentColor;
    private boolean isOuttakeOn = false;
    private boolean isIntakeOn = false;
    private double colorCheckInterval = 0.1;

    private Timer timer;

    //Colors
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);


    public TalonEncoderSubsystem() {
        talon0 = new PWMTalonSRX(0); // Replace 0 with your Talon ID
        talon1 = new PWMTalonSRX(3);
        talon2 = new PWMTalonSRX(2);
        timer = new Timer();
        timer.start();
    }

    // Method to update color
    public static synchronized void updateColor(Color newColor) {
        currentColor = newColor;
    }

    // Optionally, a method to get the current color
    public static synchronized Color getCurrentColor() {
        return currentColor;
    }

    @Override
    public void periodic() {
    // Assuming you have a method to read the sensor and update the color
    
    
    Color detectedColor = m_colorSensor.getColor();; // Implement this method based on your sensor
    TalonEncoderSubsystem.updateColor(detectedColor);

    // Now, this updated color will be used by startIntakeMotor() when it's called
    }

   public void startOuttakeMotors()
   {
    //These values worked at end of day 2/12/2023
    talon0.set(1);
    talon1.set(0.5);
   }

   public double getIntakeMotor()
   {
        return talon2.get();
   }

   public void stopAllMotors()
   {
    talon0.set(0);
    talon1.set(0);
    talon2.set(0);
   }
    
    public void activateLauncherEncoders() {
        // Add logic to activate or control the encoder
        if(!isOuttakeOn)
        {
            talon0.set(1);
            talon1.set(1);
            isOuttakeOn = true;
        }
        else
        {
            talon0.set(0.0);
            talon1.set(0.0);
            isOuttakeOn = false;
        }
            
    }

    public void activateIntakeAuto() {
        // Add logic to activate or control the encoder
        if(isIntakeOn)
        {
            talon2.set(0);
            isIntakeOn = false;
        }
        else
        {
            talon2.set(-1);
            isIntakeOn = true;
        }
            
    }

    public void activateDownwardLauncherEncoders() {
        if(!isOuttakeOn)
        {
            talon0.set(0.4);
            talon1.set(0.2);
            isOuttakeOn = true;
        }
        else
        {
            talon0.set(0.0);
            talon1.set(0.0);
            isOuttakeOn = false;
        }
            
    }

    public void activateDownwardOuttake(){
        talon0.set(0.4);
        talon1.set(0.2);
    }
    
    public void startIntakeMotor()
    {
       talon2.set(1);

    }

    public void startReverseIntakeMotor()
    {
        talon2.set(-0.5);
    }

    public void stopIntakeMotor()
    {
        talon2.set(0.0);
    }
}
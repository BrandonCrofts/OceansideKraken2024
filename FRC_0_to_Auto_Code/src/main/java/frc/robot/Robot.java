// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.TalonEncoderSubsystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    public final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        m_robotContainer.updateSmartDashboard();
        /*Color detectedColor = m_colorSensor.getColor();
        // Update the color in TalonEncoderSubsystem
        TalonEncoderSubsystem.updateColor(detectedColor);
        double IR = m_colorSensor.getIR();

        SmartDashboard.putNumber("I2C Red", TalonEncoderSubsystem.getCurrentColor().red);
        SmartDashboard.putNumber("I2C Green", TalonEncoderSubsystem.getCurrentColor().green);
        SmartDashboard.putNumber("I2C Blue", TalonEncoderSubsystem.getCurrentColor().blue);
        SmartDashboard.putNumber("Compressor Amperage:", PneumaticSubsystem.getCurrent());*/
        //SmartDashboard.putNumber("I2C IR", IR);

        //SmartDashboard.putNumber("I2C Red", detectedColor.red);
        //SmartDashboard.putNumber("I2C Green", detectedColor.green);
        //SmartDashboard.putNumber("I2C Blue", detectedColor.blue);
        /*SmartDashboard.putNumber("I2C IR", IR);

        int proximity = m_colorSensor.getProximity();

        SmartDashboard.putNumber("Proximity", proximity);*/
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = (Command) m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        //CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        /*double fla = m_robotContainer.swerveSubsystem.getFrontLeft().absoluteEncoder.getVoltage();
        SmartDashboard.putNumber("00 FL ABS VAL: ", fla);
        double fra = m_robotContainer.swerveSubsystem.getFrontRight().absoluteEncoder.getVoltage();
        SmartDashboard.putNumber("00 FR ABS VAL: ", fra);
        double bla = m_robotContainer.swerveSubsystem.getBackLeft().absoluteEncoder.getVoltage();
        SmartDashboard.putNumber("00 BL ABS VAL: ", bla);
        double bra = m_robotContainer.swerveSubsystem.getBackRight().absoluteEncoder.getVoltage();
        SmartDashboard.putNumber("00 BR ABS VAL: ", bra);*/

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase {
    private final DoubleSolenoid doubleSolenoid1;
    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    private static double compressorAmps;

    public PneumaticSubsystem() {
        doubleSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,0,1);
    }

    public void extend() {
        doubleSolenoid1.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        doubleSolenoid1.set(DoubleSolenoid.Value.kReverse);
    }

    @Override
    public void periodic()
    {
        double ca = m_compressor.getCurrent();
        updateCurrent(ca);
    }

    public static synchronized double getCurrent()
    {
        return compressorAmps;
    }

    public static synchronized void updateCurrent(double ca)
    {
        compressorAmps = ca;
    }
}

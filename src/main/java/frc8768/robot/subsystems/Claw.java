package frc8768.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private static final PneumaticHub hub = new PneumaticHub(20);
    private static final DoubleSolenoid clawSolenoid = hub.makeDoubleSolenoid(0, 1);
    private static final Compressor compressor = hub.makeCompressor();


    public void openClaw() {
        clawSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void closeClaw() {
        clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void init() {
        compressor.enableAnalog(90, 100);
    }
}

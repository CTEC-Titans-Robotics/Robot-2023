package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  private static final PneumaticHub hub = new PneumaticHub(Constants.IDs.PNEUMATIC_CAN_ID);
  private static final DoubleSolenoid clawSolenoid =
      hub.makeDoubleSolenoid(Constants.IDs.CLAW_OPEN_PORT, Constants.IDs.CLAW_CLOSE_PORT);
  private static final Compressor compressor = hub.makeCompressor();

  public void openClaw() {
    clawSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void closeClaw() {
    clawSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void init() {
    compressor.enableAnalog(85, 95);
    clawSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
}

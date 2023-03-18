package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // private static final PneumaticHub hub = new PneumaticHub(20);
    // private static final DoubleSolenoid clawSolenoid = hub.makeDoubleSolenoid(0, 1);
    // private static final Compressor compressor = hub.makeCompressor();
    CANSparkMax intake = new CANSparkMax(35, MotorType.kBrushless);

    public void in() {
        intake.set(0.75);
    }

    public void out() {
        intake.set(-0.75);
    }
    public void stopMovement() {
        intake.set(0);
    }

    
}

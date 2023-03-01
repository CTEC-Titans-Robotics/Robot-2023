package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class ArmExtension extends SubsystemBase {
    public static final CANSparkMax extension = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

    public ArmExtension() {
        //mainPIDMotor.setReference(0, )
    }
    public void zeroingProtocall() {
        extension.set(.05);
    }
    public void negative(){
        extension.set(-.05);
    }
}

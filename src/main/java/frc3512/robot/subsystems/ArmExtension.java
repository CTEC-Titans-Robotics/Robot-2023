package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

public class ArmExtension extends SubsystemBase {
    public static final CANSparkMax extension = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static final RelativeEncoder relavitveEncoder = extension.getAlternateEncoder(Type.kQuadrature, 2048*4);
    //public static final Encoder relavitveEncoder = new Encoder(8, 9);

    public static double sampleValues[]; 
    public static double zeroingCurrent = 10;
    public static boolean isZeroed;
    public static double rotationCounter;
    public double extDistance;
    public double maxPos = 2.5; //4.8
    public double minPos = 0.4; // 0.1
    public boolean reachedMax;
    public boolean reachedMin;
    public BooleanSupplier reachedMaxSup = () -> reachedMax;
    public BooleanSupplier reachedMinSup = () -> reachedMin;



    public ArmExtension() {
        isZeroed = false; 
        relavitveEncoder.setPosition(0);
        relavitveEncoder.setInverted(true);


        extension.setIdleMode(IdleMode.kBrake);
    }
    public void periodic(){
        //SmartDashboard.putNumber("relativeEncoder", relavitveEncoder.getPosition());
        //SmartDashboard.putNumber("Output in Amps", extension.getOutputCurrent());
        //SmartDashboard.putBoolean("Zeroed", isZeroed);
        //SmartDashboard.putNumber("minPosExt", extDistance);
        extDistance = relavitveEncoder.getPosition();
        SmartDashboard.putNumber("extension rotations", extDistance);

        reachedMax = extDistance > maxPos;

        reachedMin = extDistance < minPos;

        // SmartDashboard.putBoolean("reachedMinExt", reachedMin);
        // SmartDashboard.putBoolean("reachedMaxExt", reachedMax);
    }

    public void zeroingProtocol() {
            int loop_counter = 0;
            // double current_sum = 0;
            extension.set(-.1);
            // current_sum += extension.getOutputCurrent();
            while (true) {
              if (++loop_counter >= 10000 && extension.getOutputCurrent() > 25) {
                extension.stopMotor();
                relavitveEncoder.setPosition(0);
                break;
              }
            //   current_sum += extension.getOutputCurrent();
            }
          }
    public void positiveMovement(BooleanSupplier max){
        if(!max.getAsBoolean()) {
            extension.set(0.25);
        } else {
            stopMovement();
        }
    }
    public void negativeMovement(BooleanSupplier min){
        if(!min.getAsBoolean()) {
            extension.set(-0.25);
        } else {
            stopMovement();
        }
    }

    public void stopMovement() {
        extension.set(0);
    }

    public Command stopMovementCommand() {
        return run(() -> extension.set(-0.01));
    }
}

package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmExtension extends SubsystemBase {
    public static final CANSparkMax extension = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static DutyCycleEncoder relavitveEncoder = new DutyCycleEncoder(new DigitalInput(0));

    public static double sampleValues[]; 
    public static double zeroingCurrent = 10;
    public static boolean isZeroed;
    public static double rotationCounter;
    public double extDistance;
    public double maxPos = 4.8;
    public double minPos = 0.3; 
    public boolean reachedMax;
    public boolean reachedMin;
    public BooleanSupplier reachedMaxSup = () -> reachedMax;
    public BooleanSupplier reachedMinSup = () -> reachedMin;



    public ArmExtension() {
        isZeroed = false; 
        relavitveEncoder.reset();

        extension.setIdleMode(IdleMode.kBrake);
    }
    public void periodic(){
        SmartDashboard.putNumber("relativeEncoder", relavitveEncoder.get());
        SmartDashboard.putNumber("Output in Amps", extension.getOutputCurrent());
        SmartDashboard.putBoolean("Zeroed", isZeroed);
        SmartDashboard.putNumber("minPosExt", extDistance);
        extDistance = relavitveEncoder.get();
        
        if (extDistance > maxPos) {
            reachedMax = true;
        } else {
            reachedMax = false;
        }

        if (extDistance < minPos) {
            reachedMin = true;
        } else {
            reachedMin = false;
        }

        SmartDashboard.putBoolean("reachedMinExt", reachedMin);
        SmartDashboard.putBoolean("reachedMaxExt", reachedMax);
    }

    public void zeroingProtocall() {
        extension.set(-.05);
        //double startingValue = extension.getOutputCurrent(); 
        while (true){
            if(extension.getOutputCurrent() > .2) {
                relavitveEncoder.reset();
                isZeroed = true;
                extension.stopMotor();
                break;
        }
    }
    }
    public Command positiveMovement(BooleanSupplier max){
        return run(() -> {
            if(!max.getAsBoolean()) {
                extension.set(0.15);
            } else {
                stopMovement();
            }
        });
    }
    public Command negativeMovement(BooleanSupplier min){
        return run(() -> {
            if(!min.getAsBoolean()) {
                extension.set(-0.15);
            } else {
                stopMovement();
            }
        });
    }

    public void stopMovement() {
        extension.set(0);
    }

    public Command stopMovementCommand() {
        return run(() -> extension.set(0));
    }
}

package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
    private static final CANSparkMax frontMotor = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax backMotor = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final SparkMaxPIDController followerPIDMotor = frontMotor.getPIDController();
    private static final SparkMaxPIDController mainPIDMotor = backMotor.getPIDController();
    private static final ArmFeedforward armFeedForward = new ArmFeedforward(/* TODO: Values */);

    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;
    private static double kIz = 0;
    private static double kFF = 0;
    private static double kMaxOutput = 1;
    private static double kMinOutput = -1;
    private static double minVel = -9;
    private static double maxVel = 9;
    private static double maxAccel = 9;

    public Arm() {
        initDashboard();
    }

    private void moveToPos(DoubleSupplier position, DoubleSupplier vel, DoubleSupplier accel) {
        double defaultVel = 5;
        double defaultAccel = 2;
        REVLibError res = mainPIDMotor.setD(armFeedForward.calculate(position.getAsDouble(),
                vel == null ? defaultVel : vel.getAsDouble(),
                accel == null ? defaultAccel : accel.getAsDouble()));
        if(res.value != REVLibError.kOk.value) {
            System.out.println(new IllegalStateException().getMessage());
        }
    }

    // Accel and Vel can be null
    public Command move(DoubleSupplier position, DoubleSupplier vel, DoubleSupplier accel) {
        return run(() -> moveToPos(position, vel, accel));
    }

    public static void initDashboard() {
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Max Velocity", maxVel);
        SmartDashboard.putNumber("Min Velocity", minVel);
        SmartDashboard.putNumber("Max Acceleration", maxAccel);
        SmartDashboard.putNumber("Set Position", 0);
        SmartDashboard.putNumber("Set Velocity", 0);

        // button to toggle between velocity and smart motion modes
        SmartDashboard.putBoolean("Mode", true);
    }

    public void periodic() {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double maxV = SmartDashboard.getNumber("Max Velocity", 0);
        double minV = SmartDashboard.getNumber("Min Velocity", 0);
        double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
        double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { mainPIDMotor.setP(p); kP = p; }
        if((i != kI)) { mainPIDMotor.setI(i); kI = i; }
        if((d != kD)) { mainPIDMotor.setD(d); kD = d; }
        if((iz != kIz)) { mainPIDMotor.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { mainPIDMotor.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) {
            mainPIDMotor.setOutputRange(min, max);
            kMinOutput = min; kMaxOutput = max;
        }
        if((maxV != maxVel)) { mainPIDMotor.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
        if((minV != minVel)) { mainPIDMotor.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
        if((maxA != maxAccel)) { mainPIDMotor.setSmartMotionMaxAccel(maxA,0); maxAccel = maxA; }

        double setPoint, processVariable;
        boolean mode = SmartDashboard.getBoolean("Mode", false);
        if(mode) {
            setPoint = SmartDashboard.getNumber("Set Velocity", 0);
            mainPIDMotor.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
        } else {
            setPoint = SmartDashboard.getNumber("Set Position", 0);
            /**
             * As with other PID modes, Smart Motion is set by calling the
             * setReference method on an existing pid object and setting
             * the control type to kSmartMotion
             */
            mainPIDMotor.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
        }
    }

    static {
        frontMotor.follow(backMotor);

        mainPIDMotor.setP(kP);
        mainPIDMotor.setI(kI);
        mainPIDMotor.setD(kD);
        mainPIDMotor.setIZone(kIz);
        mainPIDMotor.setFF(kFF);
    }
}

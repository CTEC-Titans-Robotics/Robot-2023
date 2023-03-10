/*package frc3512.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.robot.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmPID extends ProfiledPIDSubsystem {
    public TrapezoidProfile.State High = new State(1, 2);
    public TrapezoidProfile.State Low = new State()

    public static final CANSparkMax m_motorController = new CANSparkMax(Constants.ArmConstants, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DutyCycleEncoder m_encoder = new CANCoder(Constants.ArmConstants);
    private ProfiledPIDController m_pidController;
    // need to do sysid profiling
    private final ArmFeedforward m_feedforward =
       new ArmFeedforward(
        Constants.ArmConstants.kSVolts, Constants.ArmConstants.kGVolts,
        Constants.ArmConstants.kVVoltSecondPerRad, Constants.ArmConstants.kAVoltSecondSquaredPerRad);

    public ArmPID() {
        super(
        new ProfiledPIDController(
            Constants.ArmConstants.kP,
            Constants.ArmConstants.kI,
            Constants.ArmConstants.kD,
            new TrapezoidProfile.Constraints(
                Constants.ArmConstants.kMaxVelocityRadPerSecond,
                Constants.ArmConstants.kMaxAccelerationRadPerSecSquared),
            Constants.ArmConstants.kDt),
        1);
        
        m_pidController = getController();
        m_pidController.setTolerance(Constants.ArmConstants.kPosTolerance);
        m_encoder.reset();
        //zeroingProtocall();
        m_motorController.setIdleMode(IdleMode.kBrake);
        setGoal(Constants.ArmConstants.MIN_POS);
    }

    private void zeroingProtocall() {
        int loop_counter = 0;
        double current_sum = 0; 
        m_motorController.set(-.05);
        current_sum += m_motorController.getOutputCurrent();
        while (true){
            if(m_motorController.getOutputCurrent() > 2 * (current_sum / ++loop_counter)) {
                m_motorController.stopMotor();
                m_encoder.reset();
                break;
            }
            current_sum += m_motorController.getOutputCurrent();
        }
    }
    
    @Override
    public double getMeasurement() {
      // TODO: set distance conversion
      return m_encoder.getDistance();
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
     double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_motorController.setVoltage(output);
    }
*/

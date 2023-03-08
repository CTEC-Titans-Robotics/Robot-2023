package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.lib.util.CANCoderUtil;
import frc.robot.Constants;

public class ArmNew extends ProfiledPIDSubsystem {
    public static final CANSparkMax m_mainController = new CANSparkMax(Constants.IDs.kArmMainCANID, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static final CANSparkMax m_followerController = new CANSparkMax(Constants.IDs.kArmFollowerCANID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static SparkMaxLimitSwitch m_limitSwitch =
      m_mainController.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    public static final CANCoder m_rotationEncoder = new CANCoder(Constants.IDs.kArmCancoderCANID);
    private ProfiledPIDController m_pidController;
    // need to do sysid profiling
    private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
        Constants.Extension.kS, Constants.Extension.kG,
        Constants.Extension.kV, Constants.Extension.kA);

    public ArmNew() {
        super(
        new ProfiledPIDController(
            Constants.Extension.kP,
            Constants.Extension.kI,
            Constants.Extension.kD,
            new TrapezoidProfile.Constraints(
                Constants.Extension.kMaxVelocity,
                Constants.Extension.kMaxAcceleration),
            Constants.Extension.kDt),
        30);
        
        m_pidController = getController();
        m_pidController.setTolerance(Constants.Extension.kPosTolerance);
        
        m_mainController.setIdleMode(IdleMode.kBrake);
        m_followerController.setIdleMode(IdleMode.kBrake);
        m_followerController.follow(m_mainController);
        armConfig();
        zeroingProtocol();
        setGoal(Constants.Extension.MIN_POS);
        m_limitSwitch.enableLimitSwitch(true);
    }

    private void armConfig() {
        CANCoderConfiguration armconfig = new CANCoderConfiguration();
        armconfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        armconfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        armconfig.sensorTimeBase = SensorTimeBase.PerSecond;
        armconfig.sensorCoefficient = 0.087890625;
    
        m_rotationEncoder.configFactoryDefault();
        m_rotationEncoder.configAllSettings(armconfig);
        CANCoderUtil.setCANCoderBusUsage(m_rotationEncoder, CANCoderUtil.CANCoderUsage.kMinimal);
    
        m_rotationEncoder.setPositionToAbsolute();
      }
    

    private void zeroingProtocol() {
        m_limitSwitch.enableLimitSwitch(false);
        while (!m_limitSwitch.isPressed()) {
            m_mainController.set(-0.05);
        }
        m_mainController.set(0);
        m_rotationEncoder.setPosition(0);        
    }
    
    @Override
    public double getMeasurement() {
      // TODO: set distance conversion
      return m_rotationEncoder.getPosition();
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_mainController.setVoltage(output + feedforward);
    }
}

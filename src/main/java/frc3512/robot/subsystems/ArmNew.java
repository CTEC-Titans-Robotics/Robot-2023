// package frc3512.robot.subsystems;

// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoder;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import com.ctre.phoenix.sensors.SensorTimeBase;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel;
// import com.revrobotics.SparkMaxLimitSwitch;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
// import frc3512.lib.util.CANCoderUtil;
// import frc3512.robot.Constants;
// import frc3512.robot.Constants.ArmConstants;
// import java.util.function.DoubleSupplier;

// public class ArmNew extends ProfiledPIDSubsystem {
//   public static final CANSparkMax m_mainController =
//       new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);
//   public static final CANSparkMax m_followerController =
//       new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
//   private static SparkMaxLimitSwitch m_limitSwitch =
//       m_followerController.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
//   public static final CANCoder m_rotationEncoder = new CANCoder(7);
//   private ProfiledPIDController m_pidController;
//   // need to do sysid profiling
//   private final ArmFeedforward m_feedforward =
//       new ArmFeedforward(
//           Constants.Arm.kS, Constants.Arm.kG,
//           Constants.Arm.kV, Constants.Arm.kA);

// //   private final TrapezoidProfile.State High = new State(ArmConstants.highGoalPosition, 6);
// //   private final TrapezoidProfile.State Mid = new State(ArmConstants.midGoalPosition, 6);
// //   private final TrapezoidProfile.State Low = new State(ArmConstants.lowGoalPosition, 6);

//   public ArmNew() {
//     super(
//         new ProfiledPIDController(
//             Constants.Arm.kP,
//             Constants.Arm.kI,
//             Constants.Arm.kD,
//             new TrapezoidProfile.Constraints(
//                 Constants.Arm.kMaxVelocity, Constants.Arm.kMaxAcceleration),
//             Constants.Arm.kDt),
//         30);

//     m_pidController = getController();
//     m_pidController.setTolerance(Constants.Arm.kPosTolerance);

//     m_mainController.setIdleMode(IdleMode.kBrake);
//     m_followerController.setIdleMode(IdleMode.kBrake);
//     m_followerController.follow(m_mainController);
//     armConfig();
//     zeroingProtocol();
//     setGoal(Constants.Arm.MIN_POS);
//     m_limitSwitch.enableLimitSwitch(true);
//   }

//   private void armConfig() {
//     CANCoderConfiguration armconfig = new CANCoderConfiguration();
//     armconfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
//     armconfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
//     armconfig.sensorTimeBase = SensorTimeBase.PerSecond;
//     armconfig.sensorCoefficient = 0.087890625;

//     m_rotationEncoder.configFactoryDefault();
//     m_rotationEncoder.configAllSettings(armconfig);
//     CANCoderUtil.setCANCoderBusUsage(m_rotationEncoder, CANCoderUtil.CANCoderUsage.kMinimal);

//     m_rotationEncoder.setPositionToAbsolute();
//   }

//   private void zeroingProtocol() {
//     // SmartDashboard.putBoolean("zeroing init", true);
//     m_limitSwitch.enableLimitSwitch(false);
//     while (!m_limitSwitch.isPressed()) {
//       m_mainController.set(-0.05);
//     }
//     m_mainController.set(0);
//     m_rotationEncoder.setPosition(0);
//   }

//   @Override
//   public double getMeasurement() {
//     // TODO: set distance conversion
//     return m_rotationEncoder.getPosition();
//   }

//   @Override
//   public void useOutput(double output, TrapezoidProfile.State setpoint) {
//     // Calculate the feedforward from the sepoint
//     double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
//     // Add the feedforward to the PID output to get the motor output
//     m_mainController.setVoltage(output + feedforward);
//   }

//   public void incrementing(DoubleSupplier yAxis) {
//     setGoal(yAxis.getAsDouble());
//   }
// }
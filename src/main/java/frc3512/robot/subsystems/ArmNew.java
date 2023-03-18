package frc3512.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.CANCoderUtil;
import frc3512.robot.Constants;
import frc3512.robot.Constants.Arm;
import frc3512.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.Encoder
;

import java.util.function.DoubleSupplier;

public class ArmNew extends SubsystemBase {
//motors
  public static final CANSparkMax m_mainController =
      new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
  public static final CANSparkMax m_followerController =
      new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);
//limit switch
  private static SparkMaxLimitSwitch m_limitSwitch =
      m_mainController.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
//Encoder
  public static final Encoder m_rotationEncoder = new Encoder(0,1);

 // public static double getDistance = (m_rotationEncoder.get()/2048) * (360/53.2);

  private ProfiledPIDController m_pidController = new ProfiledPIDController(
    Constants.Arm.kP,
    Constants.Arm.kI,
    Constants.Arm.kD,
    new TrapezoidProfile.Constraints(
        Constants.Arm.kMaxVelocity, Constants.Arm.kMaxAcceleration),
    Constants.Arm.kDt);
  // need to do sysid profiling
  // Feedforward
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          Constants.Arm.kS, Constants.Arm.kG,
          Constants.Arm.kV, Constants.Arm.kA);

   private final TrapezoidProfile.State High = new State(ArmConstants.highGoalPosition, 0);
   private final TrapezoidProfile.State Mid = new State(ArmConstants.midGoalPosition, 0);
   private final TrapezoidProfile.State Low = new State(ArmConstants.lowGoalPosition, 0);
   private final TrapezoidProfile.State Retracted = new State(10, 0);  


  public ArmNew() {
    //m_rotationEncoder.setDistancePerPulse(1/2048);
    m_pidController.setTolerance(Constants.Arm.kPosTolerance, Constants.Arm.kVelTolerance);

    m_mainController.setIdleMode(IdleMode.kBrake);
    m_followerController.setIdleMode(IdleMode.kBrake);
    m_followerController.follow(m_mainController);
    armConfig();
    //zeroingProtocol();
    m_limitSwitch.enableLimitSwitch(true);
  }

  private void armConfig() {
    // CANCoderConfiguration armconfig = new CANCoderConfiguration();
    // armconfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    // armconfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    // armconfig.sensorTimeBase = SensorTimeBase.PerSecond;
    // armconfig.sensorCoefficient = 0.087890625;

    // m_rotationEncoder.configFactoryDefault();
    // m_rotationEncoder.configAllSettings(armconfig);
    // CANCoderUtil.setCANCoderBusUsage(m_rotationEncoder, CANCoderUtil.CANCoderUsage.kMinimal);

    // m_rotationEncoder.setPositionToAbsolute();

  }

  public void zeroingProtocol() {
     //SmartDashboard.putBoolean("zeroing init", true);
    m_limitSwitch.enableLimitSwitch(false);
    while (!m_limitSwitch.isPressed()) {
      m_mainController.set(-.1);
    }
    m_mainController.set(0);
    m_rotationEncoder.reset();
      
  }

  public static double getMeasurement() {
    // TODO: set distance conversion
    return m_rotationEncoder.get();
  }

  
  public void useOutput(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    // double feedforward = m_feedforward.calculate(setpoint.position * (Math.PI / 180), setpoint.velocity);

    // Add the feedforward to the PID output to get the motor output
    double test = m_pidController.calculate(getDistance(), setpoint);
    //SmartDashboard.putNumber("PID VOLTAGE", test);
    // SmartDashboard.putNumber("FEEDFOWARD VOLTAGE", feedforward);
    if ((test <= 1 && test >= -1)) test = 0;
    if (test >= 8) test = 8;
    if (test <= -8) test = -8;
    m_mainController.setVoltage(test + 0);
  }

  public void incrementing(DoubleSupplier yAxis) {

  }
  public double getDistance() {
    return (m_rotationEncoder.get()/2048) * (360/53.2);
  }

  public void setHigh() {
    useOutput(High);
  }
  public void setMid() {
    useOutput(Mid);
  }
  public void setLow() {
    useOutput(Low);
  }
  public void setRetract(){
    useOutput(Retracted);
  }

  public Command setLowCommand(){
    return run(() -> setLow());
  }
  public Command setMidCommand(){
    return run(() -> setMid());
  }
  public Command setHighCommand(){
    return run(() -> setHigh());
  }
}